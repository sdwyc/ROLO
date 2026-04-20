#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <xmlrpcpp/XmlRpcValue.h>

#include <Eigen/Geometry>

#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "rolo/pose_solver.hpp"
#include "rolo/CloudInfoStamp.h"

namespace {

bool XmlRpcToDouble(const XmlRpc::XmlRpcValue &value, double *out) {
  if (!out) {
    return false;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *out = static_cast<double>(value);
    return true;
  }
  return false;
}

bool LoadWheelXY(ros::NodeHandle &pnh, std::vector<Eigen::Vector2d> *out) {
  if (!out) {
    return false;
  }
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam("wheel_xy", list)) {
    return false;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("wheel_xy should be a list of [x, y] pairs.");
    return false;
  }

  out->clear();
  for (int i = 0; i < list.size(); ++i) {
    const XmlRpc::XmlRpcValue &entry = list[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2) {
      ROS_WARN("wheel_xy entry %d is invalid.", i);
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    if (!XmlRpcToDouble(entry[0], &x) || !XmlRpcToDouble(entry[1], &y)) {
      ROS_WARN("wheel_xy entry %d is not numeric.", i);
      return false;
    }
    out->emplace_back(x, y);
  }

  return !out->empty();
}

bool LoadMeshWheelPoints(ros::NodeHandle &pnh, std::vector<Eigen::Vector3d> *out) {
  if (!out) {
    return false;
  }
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam("mesh_wheel_points", list)) {
    return false;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_WARN("mesh_wheel_points should be a list of [x, y, z] points.");
    return false;
  }

  out->clear();
  for (int i = 0; i < list.size(); ++i) {
    const XmlRpc::XmlRpcValue &entry = list[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 3) {
      ROS_WARN("mesh_wheel_points entry %d is invalid.", i);
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!XmlRpcToDouble(entry[0], &x) || !XmlRpcToDouble(entry[1], &y) || !XmlRpcToDouble(entry[2], &z)) {
      ROS_WARN("mesh_wheel_points entry %d is not numeric.", i);
      return false;
    }
    out->emplace_back(x, y, z);
  }

  return !out->empty();
}

Eigen::Vector3d LoadVector3(ros::NodeHandle &pnh, const std::string &name,
                            const Eigen::Vector3d &fallback) {
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam(name, list)) {
    return fallback;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray || list.size() < 3) {
    ROS_WARN("%s should be a list of 3 numbers.", name.c_str());
    return fallback;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  if (!XmlRpcToDouble(list[0], &x) || !XmlRpcToDouble(list[1], &y) || !XmlRpcToDouble(list[2], &z)) {
    ROS_WARN("%s entries are not numeric.", name.c_str());
    return fallback;
  }
  return Eigen::Vector3d(x, y, z);
}

Eigen::Matrix3d LoadMatrix3(ros::NodeHandle &pnh, const std::string &name,
                            const Eigen::Matrix3d &fallback) {
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam(name, list)) {
    return fallback;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray || list.size() < 9) {
    ROS_WARN("%s should be a flat list of 9 numbers.", name.c_str());
    return fallback;
  }

  Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
  for (int i = 0; i < 9; ++i) {
    double value = 0.0;
    if (!XmlRpcToDouble(list[i], &value)) {
      ROS_WARN("%s entry %d is not numeric.", name.c_str(), i);
      return fallback;
    }
    mat(i / 3, i % 3) = value;
  }
  return mat;
}

bool ComputeRigidAlignment(const std::vector<Eigen::Vector3d> &src,
                           const std::vector<Eigen::Vector3d> &dst,
                           Eigen::Isometry3d *T_out) {
  if (!T_out || src.size() != dst.size() || src.size() < 3) {
    return false;
  }

  Eigen::Vector3d src_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d dst_mean = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < src.size(); ++i) {
    src_mean += src[i];
    dst_mean += dst[i];
  }
  src_mean /= static_cast<double>(src.size());
  dst_mean /= static_cast<double>(dst.size());

  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < src.size(); ++i) {
    H += (src[i] - src_mean) * (dst[i] - dst_mean).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0.0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1.0;
    R = V * svd.matrixU().transpose();
  }

  Eigen::Vector3d t = dst_mean - R * src_mean;

  T_out->setIdentity();
  T_out->linear() = R;
  T_out->translation() = t;
  return true;
}

geometry_msgs::Pose ToPoseMsg(const Eigen::Isometry3d &T) {
  geometry_msgs::Pose pose;
  pose.position.x = T.translation().x();
  pose.position.y = T.translation().y();
  pose.position.z = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  tf2::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
  q_tf.normalize();
  pose.orientation = tf2::toMsg(q_tf);
  return pose;
}

geometry_msgs::Point ToPointMsg(const Eigen::Vector3d &p) {
  geometry_msgs::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
  return msg;
}

}  // namespace

namespace ground_factor {

class GroundFactorNode {
 public:
  GroundFactorNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh) {
    pnh.param<std::string>("pcd_topic", pcd_topic_, "/voxel_map");
    pnh.param<std::string>("pose_topic", pose_topic_, "/predicted_pose");
    pnh.param<std::string>("pose_cov_topic", pose_cov_topic_, "/initialpose");
    nh_.param<std::string>("rolo/odomTopic", odom_topic_, "odometry/imu");
    pnh.param<std::string>("frame_id", frame_id_, "map");
    pnh.param<std::string>("child_frame_id", child_frame_id_, "vehicle");
    pnh.param<std::string>("mesh_resource", mesh_resource_, "package://rolo/resource/meshes/vehicle.dae");
    pnh.param<double>("marker_scale", marker_scale_, 1.0);

    pnh.param<double>("vehicle_size_xy", vehicle_size_xy_, 2.0);
    pnh.param<double>("vehicle_com_z", vehicle_com_z_, 1.0);
    pnh.param<double>("k_spring", k_spring_, 20.0);
    pnh.param<double>("g", g_, 1.0);
    pnh.param<int>("max_iters", max_iters_, 60);
    pnh.param<double>("lm_lambda", lm_lambda_, 1e-2);
    pnh.param<double>("tol_cost", tol_cost_, 1e-12);
    pnh.param<double>("tol_step", tol_step_, 1e-10);
    pnh.param<double>("ground_avg_radius", ground_avg_radius_, 0.3);
    pnh.param<int>("ground_min_neighbors", ground_min_neighbors_, 5);
    pnh.param<double>("tolerance_z_min", tolerance_z_min_, -10.0);
    pnh.param<double>("tolerance_z_max", tolerance_z_max_, 10.0);
    pnh.param<double>("tolerance_roll", tolerance_roll_, 1.0);
    pnh.param<double>("tolerance_pitch", tolerance_pitch_, 1.0);
    pnh.param<double>("tolerance_wheel_distance", tolerance_wheel_distance_, 1.0);
    pnh.param<bool>("publish_tf", publish_tf_, true);
    pnh.param<bool>("publish_model_marker", publish_model_marker_, true);
    pnh.param<double>("model_marker_width", model_marker_width_, 0.05);
    pnh.param<bool>("verbose", verbose_, false);
    nh_.param<double>("prior_factor/groundPatchSize", ground_patch_size_, 2.0);

    std::vector<Eigen::Vector2d> wheel_xy_cw;
    if (LoadWheelXY(pnh, &wheel_xy_cw)) {
      vehicle_model_ = VehiclePyramidModel(wheel_xy_cw, vehicle_com_z_);
    } else {
      vehicle_model_ = VehiclePyramidModel::FromSquare(vehicle_size_xy_, vehicle_com_z_);
    }

    mesh_alignment_.setIdentity();
    std::vector<Eigen::Vector3d> mesh_wheels;
    if (LoadMeshWheelPoints(pnh, &mesh_wheels) &&
        mesh_wheels.size() == vehicle_model_.NumContacts()) {
      if (!ComputeRigidAlignment(mesh_wheels, vehicle_model_.base_vertices_b(), &mesh_alignment_)) {
        ROS_WARN("mesh_wheel_points alignment failed, using mesh_offset/mesh_rpy.");
      } else {
        mesh_alignment_valid_ = true;
      }
    }

    if (!mesh_alignment_valid_) {
      const Eigen::Vector3d mesh_offset = LoadVector3(pnh, "mesh_offset", Eigen::Vector3d::Zero());
      const Eigen::Vector3d mesh_rpy = LoadVector3(pnh, "mesh_rpy", Eigen::Vector3d::Zero());
      tf2::Quaternion q;
      q.setRPY(mesh_rpy.x(), mesh_rpy.y(), mesh_rpy.z());
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      mesh_alignment_.setIdentity();
      mesh_alignment_.translation() = mesh_offset;
      mesh_alignment_.linear() = q_eigen.normalized().toRotationMatrix();
    }

    body_to_lidar_.setIdentity();
    body_to_lidar_.translation() = LoadVector3(pnh, "lidarOffsetTrans", Eigen::Vector3d::Zero());
    body_to_lidar_.linear() = LoadMatrix3(pnh, "lidarOffsetRot", Eigen::Matrix3d::Identity());

    cloud_sub_ = nh_.subscribe(pcd_topic_, 1, &GroundFactorNode::CloudCallback, this);
    pose_sub_ = nh_.subscribe(pose_cov_topic_, 1, &GroundFactorNode::PoseCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odom_topic_ + "_incremental", 1, &GroundFactorNode::OdomCallback, this);

    prior_pub_ = nh_.advertise<rolo::CloudInfoStamp>("vehicle_prior_info", 1, true);
    extracted_patch_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("extracted_patch_prior", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_marker", 1, true);
    model_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("vehicle_model_marker", 1, true);
  }

 private:
  void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    ground_.UpdateFromCloud(*msg);
  }

  void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    pose_queue_.push(*msg);
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double odom_time = msg->header.stamp.toSec();
    while (!pose_queue_.empty()) {
      geometry_msgs::PoseWithCovarianceStamped &pose_msg = pose_queue_.front();
      double pose_time = pose_msg.header.stamp.toSec();
      double time_diff = pose_time - odom_time;
      if (std::abs(time_diff) < 2e-2) {
          // printf("Predicted pose synced...\n");
          const geometry_msgs::PoseWithCovarianceStamped synced_pose = pose_msg;
          pose_queue_.pop();
          HandlePose(synced_pose.pose.pose, synced_pose.header.stamp);
          return;
      }

      if (time_diff < 0.0) {
                pose_queue_.pop();
                continue;
      }

      return;
    }
  }

  void HandlePose(const geometry_msgs::Pose &pose, const ros::Time &stamp) {
    if (!ground_.IsReady()) {
      ROS_WARN_THROTTLE(1.0, "Ground point cloud not ready yet.");
      return;
    }

    const double x = pose.position.x;
    const double y = pose.position.y;
    tf2::Quaternion q_in;
    tf2::fromMsg(pose.orientation, q_in);
    const double yaw = tf2::getYaw(q_in);

    PoseSolver solver(vehicle_model_, ground_,
                      tolerance_z_min_, tolerance_z_max_,
                      tolerance_roll_, tolerance_pitch_,
                      tolerance_wheel_distance_);
    bool success;
    const SolverResult sol = solver.Solve(x, y, yaw, k_spring_, g_, max_iters_,
                                          lm_lambda_, tol_cost_, tol_step_,
                                          ground_avg_radius_, ground_min_neighbors_,
                                          success, verbose_);

    // ROS_WARN("Solved pose result: %d \n", success);

    if (success){
      tf2::Quaternion q_out;
      q_out.setRPY(sol.roll, sol.pitch, yaw);
      q_out.normalize();
      Eigen::Quaterniond q_eigen(q_out.w(), q_out.x(), q_out.y(), q_out.z());

      Eigen::Isometry3d T_world_body = Eigen::Isometry3d::Identity();
      T_world_body.linear() = q_eigen.toRotationMatrix();
      T_world_body.translation() = Eigen::Vector3d(x, y, sol.z);
      Eigen::Isometry3d T_world_lidar = T_world_body * body_to_lidar_;

      Eigen::Quaterniond q_lidar_eigen(T_world_lidar.rotation());
      tf2::Quaternion q_lidar_tf(q_lidar_eigen.x(), q_lidar_eigen.y(),
                                 q_lidar_eigen.z(), q_lidar_eigen.w());
      q_lidar_tf.normalize();
      double roll_lidar = 0.0;
      double pitch_lidar = 0.0;
      double yaw_lidar = 0.0;
      tf2::Matrix3x3(q_lidar_tf).getRPY(roll_lidar, pitch_lidar, yaw_lidar);

      geometry_msgs::PoseStamped out_pose;
      out_pose.header.stamp = stamp;
      out_pose.header.frame_id = frame_id_;
      out_pose.pose = ToPoseMsg(T_world_lidar);

      rolo::CloudInfoStamp prior_info;
      prior_info.header = out_pose.header;
      prior_info.initialGuessX = static_cast<float>(out_pose.pose.position.x);
      prior_info.initialGuessY = static_cast<float>(out_pose.pose.position.y);
      prior_info.initialGuessZ = static_cast<float>(out_pose.pose.position.z);
      prior_info.initialGuessRoll = static_cast<float>(roll_lidar);
      prior_info.initialGuessPitch = static_cast<float>(pitch_lidar);
      prior_info.initialGuessYaw = static_cast<float>(yaw_lidar);

      pcl::PointCloud<PointType>::Ptr ground_patch(new pcl::PointCloud<PointType>());
      if (ground_.ExtractPatch(Eigen::Vector2d(out_pose.pose.position.x, out_pose.pose.position.y),
                               ground_patch_size_, ground_patch)) {
        pcl::PointCloud<PointType>::Ptr ground_patch_in_pose_frame(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*ground_patch,
                                 *ground_patch_in_pose_frame,
                                 T_world_lidar.inverse().matrix().cast<float>());
        pcl::toROSMsg(*ground_patch_in_pose_frame, prior_info.extracted_ground);
      }
      prior_info.extracted_ground.header = out_pose.header;
      prior_pub_.publish(prior_info);
      extracted_patch_pub_.publish(prior_info.extracted_ground);

      Eigen::Isometry3d T_world_model = T_world_lidar * body_to_lidar_.inverse();
      Eigen::Isometry3d T_world_mesh = T_world_model * mesh_alignment_;

      visualization_msgs::Marker marker;
      marker.header = out_pose.header;
      marker.ns = "prior_pose";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = ToPoseMsg(T_world_mesh);
      marker.scale.x = marker_scale_;
      marker.scale.y = marker_scale_;
      marker.scale.z = marker_scale_;
      marker.color.r = 0.2f;
      marker.color.g = 0.6f;
      marker.color.b = 0.9f;
      marker.color.a = 0.95f;
      marker.mesh_resource = mesh_resource_;
      marker.mesh_use_embedded_materials = false;
      marker_pub_.publish(marker);

      if (publish_model_marker_) {
        visualization_msgs::Marker model_marker;
        model_marker.header = out_pose.header;
        model_marker.ns = "prior_vehicle_model";
        model_marker.id = 0;
        model_marker.type = visualization_msgs::Marker::LINE_LIST;
        model_marker.action = visualization_msgs::Marker::ADD;
        model_marker.pose.orientation.w = 1.0;
        model_marker.scale.x = model_marker_width_;
        model_marker.color.r = 0.95f;
        model_marker.color.g = 0.35f;
        model_marker.color.b = 0.10f;
        model_marker.color.a = 0.9f;

        const auto &base = vehicle_model_.base_vertices_b();
        const Eigen::Vector3d apex = vehicle_model_.apex_b();
        const size_t n = base.size();
        if (n >= 3) {
          for (size_t i = 0; i < n; ++i) {
            const Eigen::Vector3d v0 = T_world_model * base[i];
            const Eigen::Vector3d v1 = T_world_model * base[(i + 1) % n];
            model_marker.points.push_back(ToPointMsg(v0));
            model_marker.points.push_back(ToPointMsg(v1));

            const Eigen::Vector3d apex_w = T_world_model * apex;
            model_marker.points.push_back(ToPointMsg(v0));
            model_marker.points.push_back(ToPointMsg(apex_w));
          }
        }
        model_marker_pub_.publish(model_marker);
      }

      if (publish_tf_) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header = out_pose.header;
        tf_msg.child_frame_id = child_frame_id_;
        tf_msg.transform.translation.x = out_pose.pose.position.x;
        tf_msg.transform.translation.y = out_pose.pose.position.y;
        tf_msg.transform.translation.z = out_pose.pose.position.z;
        tf_msg.transform.rotation = out_pose.pose.orientation;
        tf_broadcaster_.sendTransform(tf_msg);
      }
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher prior_pub_;
  ros::Publisher extracted_patch_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher model_marker_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::queue<geometry_msgs::PoseWithCovarianceStamped> pose_queue_;

  std::string pcd_topic_;
  std::string pose_topic_;
  std::string pose_cov_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string mesh_resource_;
  double marker_scale_{1.0};

  double vehicle_size_xy_{2.0};
  double vehicle_com_z_{1.0};
  double k_spring_{20.0};
  double g_{1.0};
  int max_iters_{60};
  double lm_lambda_{1e-2};
  double tol_cost_{1e-12};
  double tol_step_{1e-10};
  double ground_avg_radius_{0.3};
  int ground_min_neighbors_{5};
  double tolerance_z_min_{-10.0};
  double tolerance_z_max_{10.0};
  double tolerance_roll_{1.0};
  double tolerance_pitch_{1.0};
  double tolerance_wheel_distance_{1.0};
  double ground_patch_size_{2.0};
  bool publish_tf_{true};
  bool publish_model_marker_{true};
  double model_marker_width_{0.05};
  bool verbose_{false};

  VehiclePyramidModel vehicle_model_;
  GroundModel ground_;
  Eigen::Isometry3d mesh_alignment_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d body_to_lidar_{Eigen::Isometry3d::Identity()};
  bool mesh_alignment_valid_{false};
};

}  // namespace ground_factor

int main(int argc, char **argv) {
  ros::init(argc, argv, "ground_factor_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ground_factor::GroundFactorNode node(nh, pnh);
  ros::spin();
  return 0;
}
