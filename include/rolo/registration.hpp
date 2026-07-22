#ifndef ROLO_REGISTRATION_HPP
#define ROLO_REGISTRATION_HPP

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef USE_UNORDERED_MAP
#undef USE_UNORDERED_MAP
#endif
#define USE_UNORDERED_MAP 0

#ifdef _OPENMP
#include <omp.h>
#endif

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

namespace rolo {

enum class RegularizationMethod { NONE, MIN_EIG, NORMALIZED_MIN_EIG, PLANE, FROBENIUS, PLANE_S };
enum class NeighborSearchMethod { DIRECT27, DIRECT7, DIRECT1 };
enum class VoxelAccumulationMode { ADDITIVE, ADDITIVE_WEIGHTED, MULTIPLICATIVE };
enum class VoxelType { POLAR, UNIFORM };

inline Eigen::Matrix3d skewd(const Eigen::Vector3d& x) {
  Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
  skew(0, 1) = -x[2];
  skew(0, 2) = x[1];
  skew(1, 0) = x[2];
  skew(1, 2) = -x[0];
  skew(2, 0) = -x[1];
  skew(2, 1) = x[0];
  return skew;
}

inline Eigen::Quaterniond so3Exp(const Eigen::Vector3d& omega) {
  const double theta_sq = omega.dot(omega);
  double imag_factor;
  double real_factor;

  if (theta_sq < 1e-10) {
    const double theta_quad = theta_sq * theta_sq;
    imag_factor = 0.5 - theta_sq / 48.0 + theta_quad / 3840.0;
    real_factor = 1.0 - theta_sq / 8.0 + theta_quad / 384.0;
  } else {
    const double theta = std::sqrt(theta_sq);
    const double half_theta = 0.5 * theta;
    imag_factor = std::sin(half_theta) / theta;
    real_factor = std::cos(half_theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());
}

inline Eigen::Isometry3d se3Exp(const Eigen::Matrix<double, 6, 1>& a) {
  const Eigen::Vector3d omega = a.head<3>();
  const double theta = std::sqrt(omega.dot(omega));
  const Eigen::Quaterniond so3 = so3Exp(omega);
  const Eigen::Matrix3d omega_mat = skewd(omega);
  const Eigen::Matrix3d omega_sq = omega_mat * omega_mat;

  Eigen::Matrix3d V;
  if (theta < 1e-10) {
    V = so3.matrix();
  } else {
    const double theta_sq = theta * theta;
    V = Eigen::Matrix3d::Identity() +
        (1.0 - std::cos(theta)) / theta_sq * omega_mat +
        (theta - std::sin(theta)) / (theta_sq * theta) * omega_sq;
  }

  Eigen::Isometry3d se3 = Eigen::Isometry3d::Identity();
  se3.linear() = so3.toRotationMatrix();
  se3.translation() = V * a.tail<3>();
  return se3;
}

inline std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>
neighborOffsets(NeighborSearchMethod method) {
  if (method == NeighborSearchMethod::DIRECT1) {
    return {Eigen::Vector3i(0, 0, 0)};
  }
  if (method == NeighborSearchMethod::DIRECT7) {
    return {
        Eigen::Vector3i(0, 0, 0),  Eigen::Vector3i(1, 0, 0),  Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0),  Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1),
        Eigen::Vector3i(0, 0, -1)};
  }

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> offsets;
  offsets.reserve(27);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        offsets.push_back(Eigen::Vector3i(i - 1, j - 1, k - 1));
      }
    }
  }
  return offsets;
}

struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i& x) const {
    std::size_t seed = 0;
    for (int i = 0; i < 3; ++i) {
      seed ^= std::hash<int>{}(x[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct VmfVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<VmfVoxel>;

  virtual ~VmfVoxel() = default;
  virtual void append(const Eigen::Vector4d& point, const Eigen::Matrix4d& covariance) = 0;
  virtual void finalize() = 0;

  int num_points = 0;
  Eigen::Vector4d mean_dir = Eigen::Vector4d::Zero();
  Eigen::Vector4d dir_reg = Eigen::Vector4d::Zero();
  double r_bar = 0.0;
  double kappa = 0.0;
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
};

struct AdditiveVmfVoxel : VmfVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void append(const Eigen::Vector4d& point, const Eigen::Matrix4d& covariance) override {
    ++num_points;
    mean_dir += point;
    r_bar = mean_dir.norm() / num_points;
    cov += covariance;
  }

  void finalize() override {
    mean_dir /= num_points;
    dir_reg = mean_dir / mean_dir.norm();
    if (r_bar < 1e-8) {
      kappa = 0.0;
    } else if (r_bar < 0.6) {
      kappa = 3.0 * r_bar * (1.0 + 0.6 * std::pow(r_bar, 2) + 99.0 / 175.0 * std::pow(r_bar, 4));
    } else {
      kappa = r_bar * (3.0 - std::pow(r_bar, 2)) / (1.0 - std::pow(r_bar, 2));
    }
    cov /= num_points;
  }
};

template <typename PointT>
class VmfVoxelMap {
public:
  using VoxelMap = std::unordered_map<
      Eigen::Vector3i, VmfVoxel::Ptr, Vector3iHash, std::equal_to<Eigen::Vector3i>,
      Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, VmfVoxel::Ptr>>>;

  VmfVoxelMap(double resolution, Eigen::Vector3d polar_resolution,
              VoxelAccumulationMode mode, VoxelType type)
      : voxel_resolution_(resolution),
        polar_resolution_(std::move(polar_resolution)),
        voxel_mode_(mode),
        voxel_type_(type) {}

  void createVoxelMap(const pcl::PointCloud<PointT>& cloud,
                      const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
    voxels_.clear();
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const Eigen::Vector4d point = cloud.at(i).getVector4fMap().template cast<double>();
      const Eigen::Vector3i coord = voxel_type_ == VoxelType::POLAR ? polarCoord(point) : voxelCoord(point);
      auto found = voxels_.find(coord);
      if (found == voxels_.end()) {
        found = voxels_.emplace(coord, std::make_shared<AdditiveVmfVoxel>()).first;
      }
      found->second->append(point, covs[i]);
    }

    for (auto& voxel : voxels_) {
      voxel.second->finalize();
    }
  }

  Eigen::Vector3i voxelCoord(const Eigen::Vector4d& x) const {
    return (x.array() / voxel_resolution_ - 0.5).floor().template cast<int>().template head<3>();
  }

  Eigen::Vector3i polarCoord(const Eigen::Vector4d& x) const {
    const double r = x.template head<3>().norm();
    return (Eigen::Vector3d(std::atan2(x[1], x[0]) + M_PI, std::acos(x[2] / r), r)
                .array() /
            polar_resolution_.array())
        .floor()
        .template cast<int>();
  }

  VmfVoxel::Ptr lookupVoxel(const Eigen::Vector3i& coord) const {
    const auto found = voxels_.find(coord);
    return found == voxels_.end() ? nullptr : found->second;
  }

private:
  double voxel_resolution_;
  Eigen::Vector3d polar_resolution_;
  VoxelAccumulationMode voxel_mode_;
  VoxelType voxel_type_;
  VoxelMap voxels_;
};

template <typename PointSource, typename PointTarget>
class SVGICP {
public:
  using Matrix4 = Eigen::Matrix4f;
  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;
  using CovarianceVector = std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SVGICP() {
#ifdef _OPENMP
    num_threads_ = omp_get_max_threads();
#else
    num_threads_ = 1;
#endif
  }

  void setResolution(double resolution) {
    voxel_resolution_ = resolution;
    voxel_type_ = VoxelType::UNIFORM;
    voxelmap_.reset();
  }

  void setPolarResolution(double theta_res, double phi_res, double r_res) {
    polar_resolution_ << theta_res, phi_res, r_res;
    voxel_type_ = VoxelType::POLAR;
    voxelmap_.reset();
  }

  void setNumThreads(int n) {
#ifdef _OPENMP
    num_threads_ = n == 0 ? omp_get_max_threads() : std::max(1, n);
#else
    num_threads_ = 1;
#endif
  }

  void setCorrespondenceRandomness(int k) {
    k_correspondences_ = std::max(3, k);
  }

  void setRegularizationMethod(RegularizationMethod method) {
    regularization_method_ = method;
  }

  void setNeighborSearchMethod(NeighborSearchMethod method) {
    search_method_ = method;
  }

  void setVoxelAccumulationMode(VoxelAccumulationMode mode) {
    voxel_mode_ = mode;
    voxelmap_.reset();
  }

  void setMaximumIterations(int iterations) {
    max_iterations_ = std::max(1, iterations);
  }

  void setTransformationEpsilon(double eps) {
    transformation_epsilon_ = eps;
  }

  void setRotationEpsilon(double eps) {
    rotation_epsilon_ = eps;
  }

  void clearSource() {
    input_.reset();
    source_covs_.clear();
    voxel_correspondences_.clear();
    voxel_mahalanobis_.clear();
  }

  void clearTarget() {
    target_.reset();
    target_covs_.clear();
    voxelmap_.reset();
    voxel_correspondences_.clear();
    voxel_mahalanobis_.clear();
  }

  void setInputSource(const PointCloudSourceConstPtr& cloud) {
    if (input_ == cloud) {
      return;
    }
    input_ = cloud;
    search_source_.reset(new pcl::search::KdTree<PointSource>);
    search_source_->setInputCloud(input_);
    source_covs_.clear();
    voxel_correspondences_.clear();
    voxel_mahalanobis_.clear();
  }

  void setInputTarget(const PointCloudTargetConstPtr& cloud) {
    if (target_ == cloud) {
      return;
    }
    target_ = cloud;
    search_target_.reset(new pcl::search::KdTree<PointTarget>);
    search_target_->setInputCloud(target_);
    target_covs_.clear();
    voxelmap_.reset();
    voxel_correspondences_.clear();
    voxel_mahalanobis_.clear();
  }

  void align(PointCloudSource& output, const Matrix4& guess = Matrix4::Identity()) {
    if (!input_ || !target_) {
      throw std::invalid_argument("SVGICP: source and target clouds must be set before align()");
    }
    if (output.points.data() == input_->points.data() || output.points.data() == target_->points.data()) {
      throw std::invalid_argument("SVGICP: output cloud cannot alias source or target");
    }

    voxelmap_.reset();
    ensureCovariances();

    Eigen::Isometry3d x0(guess.cast<double>());
    lm_lambda_ = -1.0;
    converged_ = false;

    for (int i = 0; i < max_iterations_ && !converged_; ++i) {
      nr_iterations_ = i;
      Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
      if (!rotationStepLM(x0, delta)) {
        break;
      }
      converged_ = isRotationConverged(delta);
    }

    final_transformation_ = x0.cast<float>().matrix();
    pcl::transformPointCloud(*input_, output, final_transformation_);
  }

  const Matrix4& getFinalTransformation() const {
    return final_transformation_;
  }

  bool hasConverged() const {
    return converged_;
  }

  void computeTranslation(PointCloudSource& output, Eigen::Vector3d& trans,
                          const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0,
                          double interval_tn, double interval_tn_1, float ct_lambda) {
    if (!input_ || !target_) {
      throw std::invalid_argument("SVGICP: source and target clouds must be set before computeTranslation()");
    }

    lambda_ = ct_lambda;
    ensureCovariances();
    if (voxelmap_ == nullptr) {
      voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, polar_resolution_, voxel_mode_, voxel_type_));
      voxelmap_->createVoxelMap(*target_, target_covs_);
    }
    updateCorrespondences(Eigen::Isometry3d::Identity());

    Eigen::Vector3d t0 = trans;
    lm_lambda_ = -1.0;
    bool translation_converged = false;
    for (int i = 0; i < max_iterations_ && !translation_converged; ++i) {
      nr_iterations_ = i;
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      if (!translationStepLM(t0, delta, init_guess, last_t0, interval_tn, interval_tn_1)) {
        break;
      }
      translation_converged = isTranslationConverged(delta);
    }

    Eigen::Affine3f final_translation = Eigen::Affine3f::Identity();
    final_translation.translation() = t0.cast<float>();
    pcl::transformPointCloud(*input_, output, final_translation);
    trans = t0;
  }

private:
  int threadIndex() const {
#ifdef _OPENMP
    return omp_get_thread_num();
#else
    return 0;
#endif
  }

  void ensureCovariances() {
    if (source_covs_.size() != input_->size()) {
      calculateCovariances(input_, *search_source_, source_covs_);
    }
    if (target_covs_.size() != target_->size()) {
      calculateCovariances(target_, *search_target_, target_covs_);
    }
  }

  void updateCorrespondences(const Eigen::Isometry3d& trans) {
    voxel_correspondences_.clear();
    const auto offsets = neighborOffsets(search_method_);
    std::vector<std::vector<std::pair<int, typename VmfVoxel::Ptr>>> corrs(num_threads_);
    for (auto& c : corrs) {
      c.reserve((input_->size() * offsets.size()) / std::max(1, num_threads_));
    }

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
    for (std::size_t i = 0; i < input_->size(); ++i) {
      const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
      const Eigen::Vector4d transed_mean_A = trans * mean_A;
      const Eigen::Vector3i coord = voxel_type_ == VoxelType::POLAR
                                        ? voxelmap_->polarCoord(transed_mean_A)
                                        : voxelmap_->voxelCoord(transed_mean_A);

      for (const auto& offset : offsets) {
        auto voxel = voxelmap_->lookupVoxel(coord + offset);
        if (voxel != nullptr) {
          corrs[threadIndex()].push_back(std::make_pair(static_cast<int>(i), voxel));
        }
      }
    }

    for (const auto& c : corrs) {
      voxel_correspondences_.insert(voxel_correspondences_.end(), c.begin(), c.end());
    }

    voxel_mahalanobis_.resize(voxel_correspondences_.size());
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
    for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
      const auto& corr = voxel_correspondences_[i];
      Eigen::Matrix4d RCR = corr.second->cov + trans.matrix() * source_covs_[corr.first] * trans.matrix().transpose();
      RCR(3, 3) = 1.0;
      voxel_mahalanobis_[i] = RCR.inverse();
      voxel_mahalanobis_[i](3, 3) = 0.0;
    }
  }

  double so3Linearize(const Eigen::Isometry3d& trans, Eigen::Matrix3d* H, Eigen::Vector3d* b) {
    if (voxelmap_ == nullptr) {
      voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, polar_resolution_, voxel_mode_, voxel_type_));
      voxelmap_->createVoxelMap(*target_, target_covs_);
    }

    updateCorrespondences(trans);
    double sum_errors = 0.0;
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> Hs(num_threads_, Eigen::Matrix3d::Zero());
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> bs(num_threads_, Eigen::Vector3d::Zero());

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
    for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
      const auto& corr = voxel_correspondences_[i];
      const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
      const Eigen::Vector4d transed_mean_A = trans * mean_A;
      const Eigen::Vector4d error = corr.second->mean_dir - transed_mean_A;
      const double w = std::sqrt(corr.second->num_points);

      sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error;
      if (H == nullptr || b == nullptr) {
        continue;
      }

      const Eigen::Matrix3d jacobian = skewd(transed_mean_A.head<3>());
      const Eigen::Matrix3d mahalanobis = voxel_mahalanobis_[i].block<3, 3>(0, 0);
      const int thread_num = threadIndex();
      Hs[thread_num] += w * jacobian.transpose() * mahalanobis * jacobian;
      bs[thread_num] += w * jacobian.transpose() * mahalanobis * error.head<3>();
    }

    if (H && b) {
      H->setZero();
      b->setZero();
      for (int i = 0; i < num_threads_; ++i) {
        *H += Hs[i];
        *b += bs[i];
      }
    }
    return sum_errors;
  }

  double computeRotationError(const Eigen::Isometry3d& trans) const {
    double sum_errors = 0.0;
#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
    for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
      const auto& corr = voxel_correspondences_[i];
      const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
      const Eigen::Vector4d error = corr.second->mean_dir - trans * mean_A;
      sum_errors += std::sqrt(corr.second->num_points) * error.transpose() * voxel_mahalanobis_[i] * error;
    }
    return sum_errors;
  }

  bool rotationStepLM(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
    Eigen::Matrix3d H;
    Eigen::Vector3d b;
    const double y0 = so3Linearize(x0, &H, &b);
    if (lm_lambda_ < 0.0) {
      lm_lambda_ = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
    }

    double nu = 2.0;
    for (int i = 0; i < lm_max_iterations_; ++i) {
      Eigen::LDLT<Eigen::Matrix3d> solver(H + lm_lambda_ * Eigen::Matrix3d::Identity());
      const Eigen::Vector3d d = solver.solve(-b);
      delta.setIdentity();
      delta.linear() = so3Exp(d).toRotationMatrix();

      Eigen::Isometry3d xi = delta * x0;
      const double yi = computeRotationError(xi);
      const double rho = (y0 - yi) / d.dot(lm_lambda_ * d - b);
      if (rho < 0.0) {
        if (isRotationConverged(delta)) {
          return true;
        }
        lm_lambda_ *= nu;
        nu *= 2.0;
        continue;
      }

      x0 = xi;
      lm_lambda_ *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3));
      so3_final_hessian_ = H;
      return true;
    }
    return false;
  }

  bool isRotationConverged(const Eigen::Isometry3d& delta) const {
    const Eigen::Matrix3d r_delta = (delta.linear() - Eigen::Matrix3d::Identity()).array().abs() / rotation_epsilon_;
    return r_delta.maxCoeff() < 1.0;
  }

  double translationLinearize(const Eigen::Vector3d& trans, const Eigen::Vector3d& init_guess,
                              const Eigen::Vector3d& last_t0, double interval_tn, double interval_tn_1,
                              Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) const {
    double sum_errors = 0.0;
    const std::size_t pt_size = std::max<std::size_t>(1, voxel_correspondences_.size());
    std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(
        num_threads_, Eigen::Matrix<double, 6, 6>::Zero());
    std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(
        num_threads_, Eigen::Matrix<double, 6, 1>::Zero());

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
    for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
      const auto& corr = voxel_correspondences_[i];
      const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();

      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translation() = trans;
      Eigen::Isometry3d propagation_transform = Eigen::Isometry3d::Identity();
      propagation_transform.translation() = init_guess;
      Eigen::Vector4d last_transform = Eigen::Vector4d::Zero();
      last_transform.head<3>() = last_t0;

      const Eigen::Vector4d transed_mean_A = transform * mean_A;
      const Eigen::Vector4d begin_mean_A = propagation_transform.inverse() * mean_A;
      const Eigen::Vector4d error = corr.second->mean_dir - transed_mean_A;
      const Eigen::Vector4d ct_error = (begin_mean_A - transed_mean_A) / interval_tn - last_transform / interval_tn_1;
      const double w = std::sqrt(corr.second->num_points);

      sum_errors += w * (error.transpose() * voxel_mahalanobis_[i] * error +
                         lambda_ / pt_size * ct_error.transpose() * voxel_mahalanobis_[i] * ct_error)
                            .value();
      if (H == nullptr || b == nullptr) {
        continue;
      }

      Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
      dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
      dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
      const Eigen::Matrix<double, 4, 6> j1 = dtdx0;
      const Eigen::Matrix<double, 4, 6> j2 = dtdx0 / interval_tn;
      const int thread_num = threadIndex();
      Hs[thread_num] += w * (j1.transpose() * voxel_mahalanobis_[i] * j1 +
                             lambda_ / pt_size * j2.transpose() * voxel_mahalanobis_[i] * j2);
      bs[thread_num] += w * (j1.transpose() * voxel_mahalanobis_[i] * error +
                             lambda_ / pt_size * j2.transpose() * voxel_mahalanobis_[i] * ct_error);
    }

    if (H && b) {
      H->setZero();
      b->setZero();
      for (int i = 0; i < num_threads_; ++i) {
        *H += Hs[i];
        *b += bs[i];
      }
    }
    return sum_errors;
  }

  double computeTranslationError(const Eigen::Vector3d& trans, const Eigen::Vector3d& init_guess,
                                 const Eigen::Vector3d& last_t0, double interval_tn, double interval_tn_1) const {
    double sum_errors = 0.0;
    const std::size_t pt_size = std::max<std::size_t>(1, voxel_correspondences_.size());
#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
    for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
      const auto& corr = voxel_correspondences_[i];
      const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translation() = trans;
      Eigen::Isometry3d propagation_transform = Eigen::Isometry3d::Identity();
      propagation_transform.translation() = init_guess;
      Eigen::Vector4d last_transform = Eigen::Vector4d::Zero();
      last_transform.head<3>() = last_t0;

      const Eigen::Vector4d transed_mean_A = transform * mean_A;
      const Eigen::Vector4d begin_mean_A = propagation_transform.inverse() * mean_A;
      const Eigen::Vector4d error = corr.second->mean_dir - transed_mean_A;
      const Eigen::Vector4d ct_error = (begin_mean_A - transed_mean_A) / interval_tn - last_transform / interval_tn_1;
      sum_errors += std::sqrt(corr.second->num_points) *
                    (error.transpose() * voxel_mahalanobis_[i] * error +
                     lambda_ / pt_size * ct_error.transpose() * voxel_mahalanobis_[i] * ct_error)
                        .value();
    }
    return sum_errors;
  }

  bool translationStepLM(Eigen::Vector3d& x0, Eigen::Vector3d& delta, const Eigen::Vector3d& init_guess,
                         const Eigen::Vector3d& last_t0, double interval_tn, double interval_tn_1) {
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> b;
    const double y0 = translationLinearize(x0, init_guess, last_t0, interval_tn, interval_tn_1, &H, &b);
    if (lm_lambda_ < 0.0) {
      lm_lambda_ = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
    }

    double nu = 2.0;
    for (int i = 0; i < lm_max_iterations_; ++i) {
      Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(
          H + lm_lambda_ * Eigen::Matrix<double, 6, 6>::Identity());
      const Eigen::Matrix<double, 6, 1> d = solver.solve(-b);
      delta = se3Exp(d).translation();
      const Eigen::Vector3d xi = x0 + delta;
      const double yi = computeTranslationError(xi, init_guess, last_t0, interval_tn, interval_tn_1);
      const double rho = (y0 - yi) / d.dot(lm_lambda_ * d - b);

      if (rho < 0.0) {
        if (isTranslationConverged(delta)) {
          return true;
        }
        lm_lambda_ *= nu;
        nu *= 2.0;
        continue;
      }

      x0 = xi;
      lm_lambda_ *= std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3));
      return true;
    }
    return false;
  }

  bool isTranslationConverged(const Eigen::Vector3d& delta) const {
    return (delta.array().abs() / transformation_epsilon_).maxCoeff() < 1.0;
  }

  template <typename PointT>
  bool calculateCovariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                            pcl::search::Search<PointT>& kdtree,
                            CovarianceVector& covariances) const {
    if (kdtree.getInputCloud() != cloud) {
      kdtree.setInputCloud(cloud);
    }
    covariances.resize(cloud->size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      std::vector<int> k_indices;
      std::vector<float> k_sq_distances;
      kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);

      const int neighbor_count = std::max(1, static_cast<int>(k_indices.size()));
      Eigen::Matrix<double, 4, Eigen::Dynamic> neighbors(4, neighbor_count);
      for (int j = 0; j < neighbor_count; ++j) {
        neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
      }
      neighbors.colwise() -= neighbors.rowwise().mean().eval();

      Eigen::Matrix4d cov = neighbors * neighbors.transpose() / neighbor_count;
      covariances[i].setZero();
      if (regularization_method_ == RegularizationMethod::NONE) {
        covariances[i] = cov;
        continue;
      }

      Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d values;
      switch (regularization_method_) {
        case RegularizationMethod::MIN_EIG:
          values = svd.singularValues().array().max(1e-3);
          break;
        case RegularizationMethod::NORMALIZED_MIN_EIG:
          values = (svd.singularValues() / svd.singularValues().maxCoeff()).array().max(1e-3);
          break;
        case RegularizationMethod::PLANE_S:
          values = svd.singularValues() / svd.singularValues().sum();
          values(2) = 1e-3;
          break;
        case RegularizationMethod::FROBENIUS: {
          Eigen::Matrix3d C = cov.block<3, 3>(0, 0) + 1e-3 * Eigen::Matrix3d::Identity();
          Eigen::Matrix3d C_inv = C.inverse();
          covariances[i].block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
          continue;
        }
        case RegularizationMethod::PLANE:
        default:
          values = Eigen::Vector3d(1.0, 1.0, 1e-3);
          break;
      }
      covariances[i].block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
    }
    return true;
  }

private:
  int num_threads_ = 1;
  int k_correspondences_ = 20;
  int max_iterations_ = 64;
  int nr_iterations_ = 0;
  int lm_max_iterations_ = 10;
  double lm_init_lambda_factor_ = 1e-9;
  double lm_lambda_ = -1.0;
  double rotation_epsilon_ = 2e-3;
  double transformation_epsilon_ = 5e-4;
  double voxel_resolution_ = 1.0;
  double lambda_ = 1.0;
  bool converged_ = false;

  RegularizationMethod regularization_method_ = RegularizationMethod::PLANE;
  NeighborSearchMethod search_method_ = NeighborSearchMethod::DIRECT1;
  VoxelAccumulationMode voxel_mode_ = VoxelAccumulationMode::ADDITIVE;
  VoxelType voxel_type_ = VoxelType::POLAR;

  PointCloudSourceConstPtr input_;
  PointCloudTargetConstPtr target_;
  std::shared_ptr<pcl::search::KdTree<PointSource>> search_source_{new pcl::search::KdTree<PointSource>};
  std::shared_ptr<pcl::search::KdTree<PointTarget>> search_target_{new pcl::search::KdTree<PointTarget>};
  CovarianceVector source_covs_;
  CovarianceVector target_covs_;
  Eigen::Vector3d polar_resolution_ = Eigen::Vector3d::Identity();
  Matrix4 final_transformation_ = Matrix4::Identity();
  Eigen::Matrix3d so3_final_hessian_ = Eigen::Matrix3d::Identity();

  std::unique_ptr<VmfVoxelMap<PointTarget>> voxelmap_;
  std::vector<std::pair<int, typename VmfVoxel::Ptr>> voxel_correspondences_;
  CovarianceVector voxel_mahalanobis_;
};

}  // namespace rolo

#endif  // ROLO_REGISTRATION_HPP
