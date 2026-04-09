#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

#include <sensor_msgs/PointCloud2.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace ground_factor {

Eigen::Matrix3d RotX(double a);
Eigen::Matrix3d RotY(double a);
Eigen::Matrix3d RotZ(double a);
Eigen::Matrix3d Skew(const Eigen::Vector3d &v);

struct VehiclePyramidModel {
  VehiclePyramidModel() = default;
  VehiclePyramidModel(const std::vector<Eigen::Vector2d> &base_xy_cw,
                      double com_to_base_z);

  static VehiclePyramidModel FromSquare(double size_xy, double com_to_base_z);

  size_t NumContacts() const;
  const std::vector<Eigen::Vector3d> &base_vertices_b() const;
  const Eigen::Vector3d &apex_b() const;
  double com_to_base_z() const;

 private:
  double com_to_base_z_{0.0};
  std::vector<Eigen::Vector3d> base_vertices_b_;
  Eigen::Vector3d apex_b_{Eigen::Vector3d::Zero()};
};

class GroundModel {
 public:
  void UpdateFromCloud(const sensor_msgs::PointCloud2 &msg);
  bool IsReady() const;

  Eigen::Vector3d NearestPointXY(const Eigen::Vector2d &xy) const;
  bool AverageHeightAt(const Eigen::Vector2d &xy, double radius,
                       int min_neighbors, double *height_out) const;
  bool FitLocalSurface(const Eigen::Vector2d& xy, double radius, 
                        double outlier_threshold, int min_points,
                        Eigen::Vector3d& fitted_point) const;

 private:
  bool NearestIndexXY(const Eigen::Vector2d &xy, int *index_out) const;
  bool FitPlane(const std::vector<Eigen::Vector3d>& points, 
                 Eigen::Vector4d& plane_coeffs) const;
  void RemoveOutliers(const std::vector<Eigen::Vector3d>& points,
                       double threshold, 
                       std::vector<Eigen::Vector3d>& inliers) const;
  mutable std::mutex mutex_;
  std::vector<Eigen::Vector3d> points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud_;
  mutable pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  bool ready_{false};
};

struct SolverResult {
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  double cost{0.0};
  std::vector<double> wheel_signed_distances;
  int64_t solve_time_ms{0};
  std::string end_reason;
};

class PoseSolver {
 public:
  PoseSolver(const VehiclePyramidModel &vehicle, const GroundModel &ground,
             double z_min, double z_max,
             double roll_abs_max, double pitch_abs_max,
             double wheel_distance_abs_max);

  SolverResult Solve(double x, double y, double yaw,
                     double k_spring, double g,
                     int max_iters, double lm_lambda,
                     double tol_cost, double tol_step,
                     double ground_avg_radius,
                     int ground_min_neighbors,
                     bool &success,
                     bool verbose) const;

 private:
  double InitialZ(double x, double y, double yaw,
                  double ground_avg_radius,
                  int ground_min_neighbors) const;
  double YawFromR(const Eigen::Matrix3d &R) const;
  Eigen::Matrix3d EnforceFixedYaw(const Eigen::Matrix3d &R, double yaw_fixed) const;
  void ComputeRollPitchFromFixedYaw(const Eigen::Matrix3d &R, double yaw_fixed,
                                    double &roll, double &pitch) const;

  void ComputeResidualAndJacobian(double x, double y, double yaw,
                                  double z, const Eigen::Matrix3d &R,
                                  double k_spring, double g,
                                  Eigen::Vector3d &residual,
                                  Eigen::Matrix3d &jacobian) const;
              
  bool FailureDetection(const SolverResult &res) const;

  const VehiclePyramidModel &vehicle_;
  const GroundModel &ground_;
  double z_min_{0.0};
  double z_max_{0.0};
  double roll_max_{0.0};
  double pitch_max_{0.0};
  double wheel_distance_max_{0.0};
};

}  // namespace ground_factor
#endif
