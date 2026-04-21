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

#include <Eigen/Geometry>

#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "rolo/utility.h"
#include "rolo/pose_solver.hpp"
#include "rolo/CloudInfoStamp.h"

namespace {

bool ComputeRigidAlignment(const std::vector<Eigen::Vector3d> &src,
                           const std::vector<Eigen::Vector3d> &dst,
                           Eigen::Affine3d *T_out) {
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

geometry_msgs::Pose ToPoseMsg(const Eigen::Affine3d &T) {
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

class PriorPoseNode : public ParamLoader {
 public:
  PriorPoseNode()
      : ParamLoader(false) {
    if (!priorWheelXY.empty()) {
      vehicle_model_ = VehicleModel(priorWheelXY, priorVehicleComZ,
                                    priorLidarOffsetTrans, priorLidarOffsetRot);
    } else {
      vehicle_model_ = VehicleModel::FromSquare(priorVehicleSizeXY, priorVehicleComZ,
                                                priorLidarOffsetTrans, priorLidarOffsetRot);
    }

    mesh_alignment_.setIdentity();
    if (!priorMeshWheelPoints.empty() &&
        priorMeshWheelPoints.size() == vehicle_model_.NumContacts()) {
      if (!ComputeRigidAlignment(priorMeshWheelPoints, vehicle_model_.wheel_points_body(), &mesh_alignment_)) {
        ROS_WARN("mesh_wheel_points alignment failed, using mesh_offset/mesh_rpy.");
      } else {
        mesh_alignment_valid_ = true;
      }
    }

    if (!mesh_alignment_valid_) {
      tf2::Quaternion q;
      q.setRPY(priorMeshRPY.x(), priorMeshRPY.y(), priorMeshRPY.z());
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      mesh_alignment_.setIdentity();
      mesh_alignment_.translation() = priorMeshOffset;
      mesh_alignment_.linear() = q_eigen.normalized().toRotationMatrix();
    }

    cloud_sub_ = nh.subscribe(priorPoseNodePcdTopic, 1, &PriorPoseNode::CloudCallback, this);
    pose_sub_ = nh.subscribe(priorPoseNodePoseCovTopic, 1, &PriorPoseNode::PoseCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
        odomTopic + "_incremental", 1, &PriorPoseNode::OdomCallback, this);

    prior_pub_ = nh.advertise<rolo::CloudInfoStamp>("vehicle_prior_info", 1, true);
    extracted_patch_pub_ = nh.advertise<sensor_msgs::PointCloud2>("extracted_patch_prior", 1, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("vehicle_marker", 1, true);
    model_marker_pub_ = nh.advertise<visualization_msgs::Marker>("vehicle_model_marker", 1, true);
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
                      priorToleranceZMin, priorToleranceZMax,
                      priorToleranceRoll, priorTolerancePitch,
                      priorToleranceWheelDistance);
    bool success;
    const SolverResult sol = solver.Solve(x, y, yaw, priorKSpring, priorGravity, priorMaxIters,
                                          priorLmLambda, priorTolCost, priorTolStep,
                                          priorGroundAvgRadius, priorGroundMinNeighbors,
                                          success, priorVerbose);

    // ROS_WARN("Solved pose result: %d \n", success);

    if (success){
      tf2::Quaternion q_out;
      q_out.setRPY(sol.roll, sol.pitch, yaw);
      q_out.normalize();
      Eigen::Quaterniond q_eigen(q_out.w(), q_out.x(), q_out.y(), q_out.z());

      Eigen::Affine3d T_world_body = Eigen::Affine3d::Identity();
      T_world_body.linear() = q_eigen.toRotationMatrix();
      T_world_body.translation() = Eigen::Vector3d(x, y, sol.z);
      Eigen::Affine3d T_world_lidar = T_world_body * vehicle_model_.body_to_lidar();

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
      out_pose.header.frame_id = priorPoseNodeFrameId;
      out_pose.pose = ToPoseMsg(T_world_lidar);

      rolo::CloudInfoStamp prior_info;
      prior_info.header = out_pose.header;
      prior_info.initialGuessX = static_cast<float>(out_pose.pose.position.x);
      prior_info.initialGuessY = static_cast<float>(out_pose.pose.position.y);
      prior_info.initialGuessZ = static_cast<float>(out_pose.pose.position.z);
      prior_info.initialGuessRoll = static_cast<float>(roll_lidar);
      prior_info.initialGuessPitch = static_cast<float>(pitch_lidar);
      prior_info.initialGuessYaw = static_cast<float>(yaw_lidar);

      pcl::PointCloud<::GroundPatchType>::Ptr ground_patch(new pcl::PointCloud<::GroundPatchType>());
      if (ground_.ExtractPatch(Eigen::Vector2d(out_pose.pose.position.x, out_pose.pose.position.y),
                               static_cast<double>(groundPatchSize), ground_patch)) {
        pcl::PointCloud<::GroundPatchType>::Ptr ground_patch_in_pose_frame(new pcl::PointCloud<::GroundPatchType>());
        pcl::transformPointCloud(*ground_patch,
                                 *ground_patch_in_pose_frame,
                                 T_world_lidar.inverse().matrix().cast<float>());
        pcl::toROSMsg(*ground_patch_in_pose_frame, prior_info.extracted_ground);
      }
      prior_info.extracted_ground.header = out_pose.header;
      prior_pub_.publish(prior_info);
      extracted_patch_pub_.publish(prior_info.extracted_ground);

      Eigen::Affine3d T_world_model = T_world_body;
      Eigen::Affine3d T_world_mesh = T_world_model * mesh_alignment_;

      visualization_msgs::Marker marker;
      marker.header = out_pose.header;
      marker.ns = "prior_pose";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = ToPoseMsg(T_world_mesh);
      marker.scale.x = priorPoseNodeMarkerScale;
      marker.scale.y = priorPoseNodeMarkerScale;
      marker.scale.z = priorPoseNodeMarkerScale;
      marker.color.r = 0.2f;
      marker.color.g = 0.6f;
      marker.color.b = 0.9f;
      marker.color.a = 0.95f;
      marker.mesh_resource = priorPoseNodeMeshResource;
      marker.mesh_use_embedded_materials = false;
      marker_pub_.publish(marker);

      if (priorPublishModelMarker) {
        visualization_msgs::Marker model_marker;
        model_marker.header = out_pose.header;
        model_marker.ns = "prior_vehicle_model";
        model_marker.id = 0;
        model_marker.type = visualization_msgs::Marker::LINE_LIST;
        model_marker.action = visualization_msgs::Marker::ADD;
        model_marker.pose.orientation.w = 1.0;
        model_marker.scale.x = priorModelMarkerWidth;
        model_marker.color.r = 0.95f;
        model_marker.color.g = 0.35f;
        model_marker.color.b = 0.10f;
        model_marker.color.a = 0.9f;

        const auto &base = vehicle_model_.wheel_points_body();
        const Eigen::Vector3d apex = vehicle_model_.body_origin();
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

      if (priorPublishTF) {
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header = out_pose.header;
        tf_msg.child_frame_id = priorPoseNodeChildFrameId;
        tf_msg.transform.translation.x = out_pose.pose.position.x;
        tf_msg.transform.translation.y = out_pose.pose.position.y;
        tf_msg.transform.translation.z = out_pose.pose.position.z;
        tf_msg.transform.rotation = out_pose.pose.orientation;
        tf_broadcaster_.sendTransform(tf_msg);
      }
    }
  }

  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher prior_pub_;
  ros::Publisher extracted_patch_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher model_marker_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::queue<geometry_msgs::PoseWithCovarianceStamped> pose_queue_;

  VehicleModel vehicle_model_;
  GroundModel ground_;
  Eigen::Affine3d mesh_alignment_{Eigen::Affine3d::Identity()};
  bool mesh_alignment_valid_{false};
};

}  // namespace ground_factor

int main(int argc, char **argv) {
  ros::init(argc, argv, "prior_pose_node");
  ground_factor::PriorPoseNode node;
  ros::spin();
  return 0;
}
