#include "rolo/pose_solver.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace ground_factor {

Eigen::Matrix3d RotX(double a) {
  const double ca = std::cos(a);
  const double sa = std::sin(a);
  Eigen::Matrix3d R;
  R << 1.0, 0.0, 0.0,
       0.0, ca, -sa,
       0.0, sa, ca;
  return R;
}

Eigen::Matrix3d RotY(double a) {
  const double ca = std::cos(a);
  const double sa = std::sin(a);
  Eigen::Matrix3d R;
  R << ca, 0.0, sa,
       0.0, 1.0, 0.0,
      -sa, 0.0, ca;
  return R;
}

Eigen::Matrix3d RotZ(double a) {
  const double ca = std::cos(a);
  const double sa = std::sin(a);
  Eigen::Matrix3d R;
  R << ca, -sa, 0.0,
       sa,  ca, 0.0,
       0.0, 0.0, 1.0;
  return R;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S <<   0.0, -v.z(),  v.y(),
       v.z(),   0.0, -v.x(),
      -v.y(),  v.x(),   0.0;
  return S;
}

VehiclePyramidModel::VehiclePyramidModel(const std::vector<Eigen::Vector2d> &base_xy_cw,
                                         double com_to_base_z)
    : com_to_base_z_(com_to_base_z) {
  base_vertices_b_.clear();
  base_vertices_b_.reserve(base_xy_cw.size());
  for (const auto &xy : base_xy_cw) {
    base_vertices_b_.emplace_back(xy.x(), xy.y(), -com_to_base_z_);
  }
  apex_b_.setZero();
}

VehiclePyramidModel VehiclePyramidModel::FromSquare(double size_xy, double com_to_base_z) {
  const double half = size_xy / 2.0;
  std::vector<Eigen::Vector2d> base_xy_cw = {
    Eigen::Vector2d(-half, +half),
    Eigen::Vector2d(+half, +half),
    Eigen::Vector2d(+half, -half),
    Eigen::Vector2d(-half, -half)
  };
  return VehiclePyramidModel(base_xy_cw, com_to_base_z);
}

size_t VehiclePyramidModel::NumContacts() const {
  return base_vertices_b_.size();
}

const std::vector<Eigen::Vector3d> &VehiclePyramidModel::base_vertices_b() const {
  return base_vertices_b_;
}

const Eigen::Vector3d &VehiclePyramidModel::apex_b() const {
  return apex_b_;
}

double VehiclePyramidModel::com_to_base_z() const {
  return com_to_base_z_;
}

void GroundModel::UpdateFromCloud(const sensor_msgs::PointCloud2 &msg, bool update_xy) {
  pcl::PointCloud<PointType> cloud;
  pcl::fromROSMsg(msg, cloud);

  std::lock_guard<std::mutex> lock(mutex_);
  ground_cloud_.reset(new pcl::PointCloud<PointType>());
  *ground_cloud_ = cloud;
  ready_ = !ground_cloud_->empty();

  if (!update_xy) {
    xy_cloud_.reset();
    return;
  }

  xy_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  xy_cloud_->reserve(cloud.size());

  for (const auto &pt : ground_cloud_->points) {
    xy_cloud_->push_back(pcl::PointXYZ(pt.x, pt.y, 0.0f));
  }

  if (!xy_cloud_->empty()) {
    kdtree_.setInputCloud(xy_cloud_);
  }
}

bool GroundModel::IsReady() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_ && ground_cloud_ != nullptr && !ground_cloud_->empty();
}

Eigen::Vector3d GroundModel::NearestPointXY(const Eigen::Vector2d &xy) const {
  std::lock_guard<std::mutex> lock(mutex_);
  int idx = -1;
  if (!NearestIndexXY(xy, &idx)) {
    return Eigen::Vector3d::Zero();
  }
  const auto &pt = ground_cloud_->points[idx];
  return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

bool GroundModel::AverageHeightAt(const Eigen::Vector2d &xy, double radius,
                                  int min_neighbors, double *height_out) const {
  if (!height_out) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  int idx = -1;
  if (!NearestIndexXY(xy, &idx)) {
    return false;
  }

  const auto &nearest = ground_cloud_->points[idx];
  const float center_x = nearest.x;
  const float center_y = nearest.y;

  if (radius <= 0.0) {
    *height_out = nearest.z;
    return true;
  }

  pcl::PointXYZ center(center_x, center_y, 0.0f);
  std::vector<int> indices;
  std::vector<float> dists;
  if (kdtree_.radiusSearch(center, radius, indices, dists) <= 0) {
    *height_out = nearest.z;
    return true;
  }

  if (static_cast<int>(indices.size()) < min_neighbors) {
    *height_out = nearest.z;
    return true;
  }

  double sum = 0.0;
  for (int i : indices) {
    if (i >= 0 && i < static_cast<int>(ground_cloud_->size())) {
      sum += ground_cloud_->points[i].z;
    }
  }
  *height_out = sum / static_cast<double>(indices.size());
  return true;
}

bool GroundModel::FitLocalSurface(const Eigen::Vector2d& xy, double radius, 
                                 double outlier_threshold, int min_points,
                                 Eigen::Vector3d& fitted_point) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!ready_ || ground_cloud_ == nullptr || ground_cloud_->empty() ||
        xy_cloud_ == nullptr || xy_cloud_->empty()) {
        return false;
    }

    // 1. 半径搜索获取邻近点
    pcl::PointXYZ center(static_cast<float>(xy.x()), static_cast<float>(xy.y()), 0.0f);
    std::vector<int> indices;
    std::vector<float> dists;
    
    int num_neighbors = kdtree_.radiusSearch(center, radius, indices, dists);
    if (num_neighbors < min_points) {
        return false;
    }

    // 2. 提取点云数据
    std::vector<Eigen::Vector3d> neighbors;
    neighbors.reserve(indices.size());
    for (int idx : indices) {
        if (idx >= 0 && idx < static_cast<int>(ground_cloud_->size())) {
            const auto &pt = ground_cloud_->points[idx];
            neighbors.emplace_back(pt.x, pt.y, pt.z);
        }
    }

    // 3. 按照z值剔除离群点
    std::vector<Eigen::Vector3d> inliers;
    RemoveOutliers(neighbors, outlier_threshold, inliers);
    
    if (inliers.size() < min_points) {
        return false;
    }

    // 4. 拟合平面
    Eigen::Vector4d plane_coeffs;
    if (!FitPlane(inliers, plane_coeffs)) {
        return false;
    }

    // 5. 计算轮子位置对应的z值
    double a = plane_coeffs[0], b = plane_coeffs[1];
    double c = plane_coeffs[2], d = plane_coeffs[3];
    
    if (std::abs(c) < 1e-6) {
        return false;
    }
    
    double z_value = -(a * xy.x() + b * xy.y() + d) / c;
    fitted_point = Eigen::Vector3d(xy.x(), xy.y(), z_value);
    
    return true;
}

bool GroundModel::ExtractPatch(const Eigen::Vector2d& xy, double patch_size,
                               pcl::PointCloud<PointType>::Ptr &ground_patch) const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ready_ || ground_cloud_ == nullptr || ground_cloud_->empty()) {
        return false;
    }

    if (ground_patch == nullptr) {
        ground_patch.reset(new pcl::PointCloud<PointType>());
    }
    ground_patch->clear();

    const double half_size = patch_size * 0.5;
    pcl::PointCloud<PointType>::Ptr x_filtered(new pcl::PointCloud<PointType>());

    pcl::PassThrough<PointType> pass_x;
    pass_x.setInputCloud(ground_cloud_);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(xy.x() - half_size, xy.x() + half_size);
    pass_x.filter(*x_filtered);

    pcl::PassThrough<PointType> pass_y;
    pass_y.setInputCloud(x_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(xy.y() - half_size, xy.y() + half_size);
    pass_y.filter(*ground_patch);

    ground_patch->width = ground_patch->points.size();
    ground_patch->height = 1;
    ground_patch->is_dense = ground_cloud_->is_dense;
    return !ground_patch->empty();
}

void GroundModel::RemoveOutliers(const std::vector<Eigen::Vector3d>& points,
                                double threshold, 
                                std::vector<Eigen::Vector3d>& inliers) const {
    if (points.empty()) return;
    
    // 计算z值的均值和标准差
    double sum_z = 0.0;
    for (const auto& pt : points) {
        sum_z += pt.z();
    }
    double mean_z = sum_z / points.size();
    
    double sum_sq = 0.0;
    for (const auto& pt : points) {
        sum_sq += (pt.z() - mean_z) * (pt.z() - mean_z);
    }
    double std_z = std::sqrt(sum_sq / points.size());
    
    // 剔除离群点
    double lower_bound = mean_z - threshold * std_z;
    double upper_bound = mean_z + threshold * std_z;
    
    for (const auto& pt : points) {
        if (pt.z() >= lower_bound && pt.z() <= upper_bound) {
            inliers.push_back(pt);
        }
    }
}

bool GroundModel::FitPlane(const std::vector<Eigen::Vector3d>& points,
                          Eigen::Vector4d& plane_coeffs) const {
    if (points.size() < 3) return false;
    
    // 计算质心
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& pt : points) {
        centroid += pt;
    }
    centroid /= points.size();
    
    // 构建协方差矩阵
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& pt : points) {
        Eigen::Vector3d centered = pt - centroid;
        cov += centered * centered.transpose();
    }
    
    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    Eigen::Vector3d normal = eig.eigenvectors().col(0);
    
    // 平面方程: n·(p - p0) = 0 => n·p - n·p0 = 0
    double d = -normal.dot(centroid);
    plane_coeffs << normal[0], normal[1], normal[2], d;
    
    return true;
}

bool GroundModel::NearestIndexXY(const Eigen::Vector2d &xy, int *index_out) const {
  if (!index_out) {
    return false;
  }
  if (!ready_ || ground_cloud_ == nullptr || ground_cloud_->empty() ||
      xy_cloud_ == nullptr || xy_cloud_->empty()) {
    return false;
  }

  pcl::PointXYZ query(static_cast<float>(xy.x()), static_cast<float>(xy.y()), 0.0f);
  std::vector<int> indices(1);
  std::vector<float> dists(1);
  if (kdtree_.nearestKSearch(query, 1, indices, dists) > 0) {
    const int idx = indices[0];
    if (idx >= 0 && idx < static_cast<int>(ground_cloud_->size())) {
      *index_out = idx;
      return true;
    }
  }

  return false;
}

// ... 其余代码保持不变 ...
PoseSolver::PoseSolver(const VehiclePyramidModel &vehicle, const GroundModel &ground,
                       double z_min, double z_max,
                       double roll_abs_max, double pitch_abs_max,
                       double wheel_distance_abs_max)
    : vehicle_(vehicle),
      ground_(ground),
      z_min_(z_min),
      z_max_(z_max),
      roll_max_(roll_abs_max),
      pitch_max_(pitch_abs_max),
      wheel_distance_max_(wheel_distance_abs_max) {}

SolverResult PoseSolver::Solve(double x, double y, double yaw,
                               double k_spring, double g,
                               int max_iters, double lm_lambda,
                               double tol_cost, double tol_step,
                               double ground_avg_radius,
                               int ground_min_neighbors,
                               bool &success,
                               bool verbose) const {
  SolverResult result;

  const auto t_start = std::chrono::steady_clock::now();
  Eigen::Matrix3d R = RotZ(yaw);
  double z = InitialZ(x, y, yaw, ground_avg_radius, ground_min_neighbors);

  double last_cost = std::numeric_limits<double>::infinity();
  double best_cost = std::numeric_limits<double>::infinity();
  Eigen::Matrix3d best_R = R;
  double best_z = z;

  for (int it = 0; it < max_iters; ++it) {
    Eigen::Vector3d residual;
    Eigen::Matrix3d jacobian;
    ComputeResidualAndJacobian(x, y, yaw, z, R, k_spring, g, residual, jacobian);

    const double c0 = residual.dot(residual);
    if (c0 < best_cost) {
      best_cost = c0;
      best_R = R;
      best_z = z;
    }

    Eigen::Matrix3d A = jacobian.transpose() * jacobian + lm_lambda * Eigen::Matrix3d::Identity();
    Eigen::Vector3d b = -jacobian.transpose() * residual;

    Eigen::Vector3d delta;
    Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
    if (!lu.isInvertible()) {
      lm_lambda *= 10.0;
      continue;
    }
    delta = lu.solve(b);

    const double dz = delta(0);
    const double droll = delta(1);
    const double dpitch = delta(2);
    const double step_norm = delta.norm();

    const double z_new = z + dz;
    Eigen::Matrix3d R_new = RotX(droll) * RotY(dpitch) * R;
    R_new = EnforceFixedYaw(R_new, yaw);

    Eigen::Vector3d residual_new;
    Eigen::Matrix3d jacobian_tmp;
    ComputeResidualAndJacobian(x, y, yaw, z_new, R_new, k_spring, g, residual_new, jacobian_tmp);
    const double c1 = residual_new.dot(residual_new);

    if (verbose) {
      double roll_est = 0.0;
      double pitch_est = 0.0;
      ComputeRollPitchFromFixedYaw(R, yaw, roll_est, pitch_est);
      std::ostringstream oss;
      oss << "[pose_solver] "
          << "iter " << std::setw(2) << it
          << " | cost=" << std::scientific << std::setprecision(6) << c0
          << " | trial=" << std::scientific << std::setprecision(6) << c1
          << " | z=" << std::fixed << std::setprecision(6) << z
          << " | roll=" << std::fixed << std::setprecision(6) << roll_est
          << " | pitch=" << std::fixed << std::setprecision(6) << pitch_est
          << " | step=" << std::scientific << std::setprecision(3) << step_norm
          << " | lambda=" << std::scientific << std::setprecision(3) << lm_lambda;
      std::cout << (oss.str()) << std::endl;
    }

    if (c1 < c0) {
      z = z_new;
      R = R_new;
      lm_lambda = std::max(lm_lambda / 2.0, 1e-8);
      if (std::abs(last_cost - c1) < tol_cost && step_norm < tol_step) {
        result.end_reason = "converged";
        break;
      }
      last_cost = c1;
    } else {
      lm_lambda *= 5.0;
      last_cost = c0;
    }

    if (it == max_iters - 1) {
      result.end_reason = "max_iters";
    }
  }

  result.z = best_z;
  result.R = best_R;
  ComputeRollPitchFromFixedYaw(best_R, yaw, result.roll, result.pitch);
  result.cost = best_cost;
  result.wheel_signed_distances.resize(vehicle_.NumContacts(), 0.0);
  {
    const Eigen::Vector3d ez(0.0, 0.0, 1.0);
    const Eigen::Vector3d t(x, y, best_z);
    const Eigen::Vector3d n_car_w = best_R * ez;
    const auto &base = vehicle_.base_vertices_b();
    for (size_t i = 0; i < base.size(); ++i) {
      const Eigen::Vector3d pw_w = best_R * base[i] + t;
      const Eigen::Vector3d p_n = ground_.NearestPointXY(Eigen::Vector2d(pw_w.x(), pw_w.y()));
      const double d_i = (pw_w - p_n).dot(n_car_w);
      result.wheel_signed_distances[i] = d_i;
    }
  }
  const auto t_end = std::chrono::steady_clock::now();
  result.solve_time_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  if (result.end_reason.empty()) {
    result.end_reason = "max_iters";
  }

  if (verbose) {
    std::ostringstream oss;
    oss << "[pose_solver] Done!! \n cost=" << std::scientific << std::setprecision(6) << result.cost
        << " , z=" << std::fixed << std::setprecision(6) << result.z
        << " , roll=" << std::fixed << std::setprecision(6) << result.roll
        << " , pitch=" << std::fixed << std::setprecision(6) << result.pitch
        << " , time_ms=" << result.solve_time_ms << "ms"
        << " , is_converged=" << result.end_reason;
    ROS_INFO_STREAM(oss.str());
    if (!result.wheel_signed_distances.empty()) {
      std::ostringstream ds;
      ds << "[pose_solver] distance_wheel_ground=[";
      for (size_t i = 0; i < result.wheel_signed_distances.size(); ++i) {
        ds << std::fixed << std::setprecision(6) << result.wheel_signed_distances[i];
        if (i + 1 < result.wheel_signed_distances.size()) {
          ds << ", ";
        }
      }
      ds << "]";
      ROS_INFO_STREAM(ds.str());
    }
  }

  success = FailureDetection(result);

  return result;
}

bool PoseSolver::FailureDetection(const SolverResult &res) const {
  if (res.z < z_min_ || res.z > z_max_) {
    return false;
  }

  if (std::abs(res.roll) > roll_max_ || std::abs(res.pitch) > pitch_max_) {
    return false;
  }

  for (const double distance : res.wheel_signed_distances) {
    if (std::abs(distance) > wheel_distance_max_) {
      return false;
    }
  }

  if (res.end_reason == "max_iters") {
    return false;
  }

  return true;
}

double PoseSolver::InitialZ(double x, double y, double yaw,
                            double ground_avg_radius,
                            int ground_min_neighbors) const {
  const Eigen::Matrix3d Rz = RotZ(yaw);
  double min_h = std::numeric_limits<double>::infinity();

  for (const auto &vertex_b : vehicle_.base_vertices_b()) {
    Eigen::Vector3d wheel_xy = Rz * vertex_b;
    Eigen::Vector2d query_xy(x + wheel_xy.x(), y + wheel_xy.y());
    double h = 0.0;
    if (ground_.AverageHeightAt(query_xy, ground_avg_radius, ground_min_neighbors, &h)) {
      min_h = std::min(min_h, h);
    }
  }

  if (!std::isfinite(min_h)) {
    return 0.0;
  }

  return min_h + vehicle_.com_to_base_z() - 1.0;
}

double PoseSolver::YawFromR(const Eigen::Matrix3d &R) const {
  return std::atan2(R(1, 0), R(0, 0));
}

Eigen::Matrix3d PoseSolver::EnforceFixedYaw(const Eigen::Matrix3d &R, double yaw_fixed) const {
  const double yaw_cur = YawFromR(R);
  Eigen::Matrix3d R_tilt = RotZ(-yaw_cur) * R;
  return RotZ(yaw_fixed) * R_tilt;
}

void PoseSolver::ComputeRollPitchFromFixedYaw(const Eigen::Matrix3d &R, double yaw_fixed,
                                              double &roll, double &pitch) const {
  Eigen::Matrix3d R_tilt = RotZ(-yaw_fixed) * R;
  roll = std::atan2(R_tilt(2, 1), R_tilt(2, 2));
  pitch = std::atan2(-R_tilt(2, 0), R_tilt(0, 0));
}

void PoseSolver::ComputeResidualAndJacobian(double x, double y, double yaw,
                                            double z, const Eigen::Matrix3d &R,
                                            double k_spring, double g,
                                            Eigen::Vector3d &residual,
                                            Eigen::Matrix3d &jacobian) const {
  const Eigen::Vector3d ex(1.0, 0.0, 0.0);
  const Eigen::Vector3d ey(0.0, 1.0, 0.0);
  const Eigen::Vector3d ez(0.0, 0.0, 1.0);

  const Eigen::Matrix3d Sx = Skew(ex);
  const Eigen::Matrix3d Sy = Skew(ey);

  const size_t k = vehicle_.base_vertices_b().size();
  Eigen::MatrixXd wrench_map(3, k);
  Eigen::VectorXd contact_forces(k);
  Eigen::VectorXd df_dz(k);
  Eigen::VectorXd df_droll(k);
  Eigen::VectorXd df_dpitch(k);

  for (size_t i = 0; i < k; ++i) {
    const Eigen::Vector3d r_b = vehicle_.base_vertices_b()[i] - vehicle_.apex_b();
    wrench_map(0, i) = 1.0;
    wrench_map(1, i) = r_b.y();
    wrench_map(2, i) = -r_b.x();
  }

  const Eigen::Vector3d t(x, y, z);
  const Eigen::Vector3d vehicle_normal_w = R * ez;
  const Eigen::Vector3d gravity_wrench(vehicle_normal_w.dot(ez), 0.0, 0.0);

  for (size_t i = 0; i < k; ++i) {
    const Eigen::Vector3d p_i_b = vehicle_.base_vertices_b()[i];
    const Eigen::Vector3d pw_w = R * p_i_b + t;
    // 替换原来的最近点查询
Eigen::Vector3d p_n;
double fit_radius = 0.6;  // 拟合半径
double outlier_threshold = 3; // 3倍标准差
int min_points = 15;

if (ground_.FitLocalSurface(Eigen::Vector2d(pw_w.x(), pw_w.y()), 
                           fit_radius, outlier_threshold, min_points, p_n)) {
    // 使用拟合的曲面点
} else {
    // 回退到原来的最近点方法
    p_n = ground_.NearestPointXY(Eigen::Vector2d(pw_w.x(), pw_w.y()));
}

    const Eigen::Vector3d a = pw_w - p_n;
    const Eigen::Vector3d b = vehicle_normal_w;
    const double d_i = a.dot(b);

    const bool active = (d_i < 0.0);
    const double f_i = active ? (k_spring * d_i) : 0.0;
    contact_forces(static_cast<int>(i)) = f_i;

    const double dd_dz = ez.dot(b);
    const Eigen::Vector3d Rp = R * p_i_b;
    const double dd_droll = (Sx * Rp).dot(b) + a.dot(Sx * b);
    const double dd_dpitch = (Sy * Rp).dot(b) + a.dot(Sy * b);

    if (active) {
      df_dz(static_cast<int>(i)) = k_spring * dd_dz;
      df_droll(static_cast<int>(i)) = k_spring * dd_droll;
      df_dpitch(static_cast<int>(i)) = k_spring * dd_dpitch;
    } else {
      df_dz(static_cast<int>(i)) = 0.0;
      df_droll(static_cast<int>(i)) = 0.0;
      df_dpitch(static_cast<int>(i)) = 0.0;
    }
  }

  residual = wrench_map * contact_forces + g * gravity_wrench;

  jacobian.setZero();
  jacobian.col(0) = wrench_map * df_dz;
  jacobian.col(1) = wrench_map * df_droll;
  jacobian.col(2) = wrench_map * df_dpitch;

  const double dWg_droll = ez.dot(Sx * vehicle_normal_w);
  const double dWg_dpitch = ez.dot(Sy * vehicle_normal_w);
  jacobian(0, 1) += g * dWg_droll;
  jacobian(0, 2) += g * dWg_dpitch;
}

}  // namespace ground_factor
