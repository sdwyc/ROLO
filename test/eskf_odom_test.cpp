#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "rolo/eskf/eskf.hpp"

class EskfOdomTestNode {
public:
  EskfOdomTestNode() : nh_(), pnh_("~") {
    loadParams();

    raw_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(raw_odom_topic_, 10);
    filtered_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(filtered_odom_topic_, 10);
    raw_path_pub_ = nh_.advertise<nav_msgs::Path>(raw_path_topic_, 10);
    filtered_path_pub_ = nh_.advertise<nav_msgs::Path>(filtered_path_topic_, 10);

    odom_sub_ = nh_.subscribe(input_topic_, 2000, &EskfOdomTestNode::odomCallback, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("ESKF odom test subscribed to " << input_topic_);
  }

private:
  void loadParams() {
    pnh_.param<std::string>("input_topic", input_topic_, "/odometry/lidar_incremental");
    pnh_.param<std::string>("raw_odom_topic", raw_odom_topic_, "/rolo/eskf/raw_odom");
    pnh_.param<std::string>("filtered_odom_topic", filtered_odom_topic_, "/rolo/eskf/filtered_odom");
    pnh_.param<std::string>("raw_path_topic", raw_path_topic_, "/rolo/eskf/raw_path");
    pnh_.param<std::string>("filtered_path_topic", filtered_path_topic_, "/rolo/eskf/filtered_path");
    pnh_.param<std::string>("world_frame", world_frame_, std::string("odometry"));
    pnh_.param<std::string>("child_frame", child_frame_, std::string("eskf_lidar"));
    pnh_.param<std::string>("tf_parent_frame", tf_parent_frame_, std::string("odometry"));
    pnh_.param<std::string>("raw_tf_child_frame", raw_tf_child_frame_, std::string("lidar_raw"));
    pnh_.param<std::string>("filtered_tf_child_frame", filtered_tf_child_frame_, std::string("lidar_filtered"));
    pnh_.param<int>("max_path_size", max_path_size_, 10000);
    pnh_.param<bool>("use_input_covariance", use_input_covariance_, true);

    rolo::eskf::PoseESEKF::Options options;
    pnh_.param<double>("max_dt", options.max_dt, options.max_dt);
    pnh_.param<double>("q_linear_jerk_std", options.q_linear_jerk_std, options.q_linear_jerk_std);
    pnh_.param<double>("q_angular_jerk_std", options.q_angular_jerk_std, options.q_angular_jerk_std);
    pnh_.param<double>("r_position_std", options.r_position_std, options.r_position_std);
    pnh_.param<double>("r_rotation_std", options.r_rotation_std, options.r_rotation_std);
    pnh_.param<double>("init_position_std", options.init_position_std, options.init_position_std);
    pnh_.param<double>("init_rotation_std", options.init_rotation_std, options.init_rotation_std);
    pnh_.param<double>("init_velocity_std", options.init_velocity_std, options.init_velocity_std);
    pnh_.param<double>("init_angular_velocity_std", options.init_angular_velocity_std, options.init_angular_velocity_std);
    pnh_.param<double>("init_acceleration_std", options.init_acceleration_std, options.init_acceleration_std);
    pnh_.param<double>("init_angular_acceleration_std", options.init_angular_acceleration_std, options.init_angular_acceleration_std);
    pnh_.param<int>("maximum_iteration", options.maximum_iteration, options.maximum_iteration);
    pnh_.param<double>("convergence_limit", options.convergence_limit, options.convergence_limit);
    filter_.setOptions(options);
  }

  void odomCallback(nav_msgs::Odometry::ConstPtr msg) {
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
    );

    rolo::eskf::PoseESEKF::MeasurementNoiseCov R = filter_.defaultMeasurementNoise();
    if(use_input_covariance_) {
      fillMeasurementNoiseFromMsg(*msg, R);
    }

    bool accepted = filter_.processMeasurement(msg->header.stamp.toSec(), position, orientation, R);
    if(!accepted) {
      return;
    }

    publishRaw(*msg);
    publishFiltered(*msg);
    publishTf(*msg);
  }

  void fillMeasurementNoiseFromMsg(nav_msgs::Odometry const& msg, rolo::eskf::PoseESEKF::MeasurementNoiseCov& R) {
    bool valid = true;
    for(int i = 0; i < 6; i++) {
      double v = msg.pose.covariance[i * 6 + i];
      if(!std::isfinite(v) || v <= 0.0) {
        valid = false;
        break;
      }
    }

    if(!valid) {
      return;
    }

    for(int r = 0; r < 6; r++) {
      for(int c = 0; c < 6; c++) {
        double v = msg.pose.covariance[r * 6 + c];
        R(r, c) = std::isfinite(v) ? v : 0.0;
      }
    }
  }

  void publishRaw(nav_msgs::Odometry const& input) {
    nav_msgs::Odometry raw = input;
    if(raw.header.frame_id.empty()) {
      raw.header.frame_id = world_frame_;
    }
    raw_odom_pub_.publish(raw);

    geometry_msgs::PoseStamped pose;
    pose.header = raw.header;
    pose.pose = raw.pose.pose;
    appendPath(raw_path_, pose, raw_path_pub_);
  }

  void publishFiltered(nav_msgs::Odometry const& input) {
    Eigen::Vector3d position = filter_.position();
    Eigen::Quaterniond orientation = filter_.orientation();
    Eigen::Vector3d velocity = filter_.velocity();
    Eigen::Vector3d angular_velocity = filter_.angularVelocity();
    Eigen::Matrix<double, 6, 6> pose_cov = filter_.poseCovariance();

    nav_msgs::Odometry odom;
    odom.header.stamp = input.header.stamp;
    odom.header.frame_id = input.header.frame_id.empty() ? world_frame_ : input.header.frame_id;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();
    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    odom.twist.twist.angular.x = angular_velocity.x();
    odom.twist.twist.angular.y = angular_velocity.y();
    odom.twist.twist.angular.z = angular_velocity.z();

    for(int r = 0; r < 6; r++) {
      for(int c = 0; c < 6; c++) {
        odom.pose.covariance[r * 6 + c] = pose_cov(r, c);
      }
    }

    filtered_odom_pub_.publish(odom);

    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    appendPath(filtered_path_, pose, filtered_path_pub_);
  }

  void publishTf(nav_msgs::Odometry const& input) {
    geometry_msgs::TransformStamped raw_tf;
    raw_tf.header.stamp = input.header.stamp;
    raw_tf.header.frame_id = tf_parent_frame_;
    raw_tf.child_frame_id = raw_tf_child_frame_;
    raw_tf.transform.translation.x = input.pose.pose.position.x;
    raw_tf.transform.translation.y = input.pose.pose.position.y;
    raw_tf.transform.translation.z = input.pose.pose.position.z;
    raw_tf.transform.rotation = input.pose.pose.orientation;
    tf_broadcaster_.sendTransform(raw_tf);

    Eigen::Vector3d position = filter_.position();
    Eigen::Quaterniond orientation = filter_.orientation();
    geometry_msgs::TransformStamped filtered_tf;
    filtered_tf.header.stamp = input.header.stamp;
    filtered_tf.header.frame_id = tf_parent_frame_;
    filtered_tf.child_frame_id = filtered_tf_child_frame_;
    filtered_tf.transform.translation.x = position.x();
    filtered_tf.transform.translation.y = position.y();
    filtered_tf.transform.translation.z = position.z();
    filtered_tf.transform.rotation.x = orientation.x();
    filtered_tf.transform.rotation.y = orientation.y();
    filtered_tf.transform.rotation.z = orientation.z();
    filtered_tf.transform.rotation.w = orientation.w();
    tf_broadcaster_.sendTransform(filtered_tf);
  }

  void appendPath(nav_msgs::Path& path, geometry_msgs::PoseStamped const& pose, ros::Publisher& pub) {
    path.header = pose.header;
    path.poses.push_back(pose);
    if(max_path_size_ > 0 && static_cast<int>(path.poses.size()) > max_path_size_) {
      path.poses.erase(path.poses.begin(), path.poses.begin() + (path.poses.size() - max_path_size_));
    }
    pub.publish(path);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Publisher raw_odom_pub_;
  ros::Publisher filtered_odom_pub_;
  ros::Publisher raw_path_pub_;
  ros::Publisher filtered_path_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::string input_topic_;
  std::string raw_odom_topic_;
  std::string filtered_odom_topic_;
  std::string raw_path_topic_;
  std::string filtered_path_topic_;
  std::string world_frame_;
  std::string child_frame_;
  std::string tf_parent_frame_;
  std::string raw_tf_child_frame_;
  std::string filtered_tf_child_frame_;
  int max_path_size_ = 10000;
  bool use_input_covariance_ = true;

  nav_msgs::Path raw_path_;
  nav_msgs::Path filtered_path_;
  rolo::eskf::PoseESEKF filter_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "eskf_odom_test");
  EskfOdomTestNode node;
  ros::spin();
  return 0;
}
