#ifndef ROT_VGICP_IMPL_HPP
#define ROT_VGICP_IMPL_HPP

#include <atomic>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/registration.h>

#include <rot_gicp/so3/so3.hpp>
#include <rot_gicp/gicp/rot_vgicp.hpp>
#include <rot_gicp/gicp/vmp_voxel.hpp>

namespace fast_gicp {

template <typename PointSource, typename PointTarget>
RotVGICP<PointSource, PointTarget>::RotVGICP() : LsqRegistration<PointSource, PointTarget>() {
#ifdef _OPENMP
  num_threads_ = omp_get_max_threads();
#else
  num_threads_ = 1;
#endif

  this->reg_name_ = "RotVGICP";
  k_correspondences_ = 20;

  regularization_method_ = RegularizationMethod::PLANE;
  search_source_.reset(new SearchMethodSource);
  search_target_.reset(new SearchMethodTarget);
  voxel_resolution_ = 1.0;
  lambda_ = 1.0;
  search_method_ = NeighborSearchMethod::DIRECT1;
  voxel_mode_ = VoxelAccumulationMode::ADDITIVE;
  voxel_type_ = VoxelType::POLAR;
  polar_resolution_ = Eigen::Vector3d::Identity();
}

template <typename PointSource, typename PointTarget>
RotVGICP<PointSource, PointTarget>::~RotVGICP() {}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setResolution(double resolution) {
  voxel_resolution_ = resolution;
  voxel_type_ = VoxelType::UNIFORM;
  voxelmap_.reset();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setPolarResolution(double theta_res, double phi_res, double r_res) {
  polar_resolution_ << theta_res, phi_res, r_res;
  voxel_type_ = VoxelType::POLAR;
  voxelmap_.reset();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setNeighborSearchMethod(NeighborSearchMethod method) {
  search_method_ = method;
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setVoxelAccumulationMode(VoxelAccumulationMode mode) {
  voxel_mode_ = mode;
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::swapSourceAndTarget() {
  input_.swap(target_);
  search_source_.swap(search_target_);
  source_covs_.swap(target_covs_);
  voxelmap_.reset();
  voxel_correspondences_.clear();
  voxel_mahalanobis_.clear();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setNumThreads(int n) {
  num_threads_ = n;

#ifdef _OPENMP
  if (n == 0) {
    num_threads_ = omp_get_max_threads();
  }
#endif
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setCorrespondenceRandomness(int k) {
  k_correspondences_ = k;
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setRegularizationMethod(RegularizationMethod method) {
  regularization_method_ = method;
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::clearSource() {
  input_.reset();
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::clearTarget() {
  target_.reset();
  target_covs_.clear();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setInputSource(const PointCloudSourceConstPtr& cloud) {
  if (input_ == cloud) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputSource(cloud);
  search_source_->setInputCloud(cloud);
  source_covs_.clear();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  source_covs_ = covs;
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs) {
  target_covs_ = covs;
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setInputTarget(const PointCloudTargetConstPtr& cloud) {
  if (target_ == cloud) {
    return;
  }

  pcl::Registration<PointSource, PointTarget, Scalar>::setInputTarget(cloud);
  search_target_->setInputCloud(cloud);
  target_covs_.clear();

  voxelmap_.reset();
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  voxelmap_.reset();
  // std::cout << "guess: " << guess.matrix() << std::endl;

  if (output.points.data() == input_->points.data() || output.points.data() == target_->points.data()) {
    throw std::invalid_argument("RotVGICP: destination cloud cannot be identical to source or target");
  }
  if (source_covs_.size() != input_->size()) {
    calculate_covariances(input_, *search_source_, source_covs_);
  }
  if (target_covs_.size() != target_->size()) {
    calculate_covariances(target_, *search_target_, target_covs_);
  }  
  LsqRegistration<PointSource, PointTarget>::computeTransformation(output, guess);
}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::computeTranslation(PointCloudSource& output, Eigen::Vector3d& trans,
                        const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0, 
                        const double interval_tn, const double interval_tn_1, const float ct_lambda) {
  lambda_ = ct_lambda;
  LsqRegistration<PointSource, PointTarget>::computeTranslation(output, trans, init_guess, last_t0, interval_tn, interval_tn_1, ct_lambda);
  // voxelmap_.reset();
}


template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& trans) {
  voxel_correspondences_.clear();
  auto offsets = neighbor_offsets(search_method_);
  // Parallel chunk processing
  std::vector<std::vector<std::pair<int, VmfVoxel::Ptr>>> corrs(num_threads_); // Per-thread correspondence bins
  for (auto& c : corrs) {
    c.reserve((input_->size() * offsets.size()) / num_threads_); // Reserve correspondence storage
  }

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (std::size_t i = 0; i < input_->size(); ++i) {
    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    Eigen::Vector4d transed_mean_A = trans * mean_A;
    Eigen::Vector3i coord = voxel_type_ == VoxelType::POLAR ? voxelmap_->polar_coord(transed_mean_A) : voxelmap_->voxel_coord(transed_mean_A); // Voxel index

    for (const auto& offset : offsets) {
      auto voxel = voxelmap_->lookup_voxel(coord + offset); // Neighbor voxel lookup
      if (voxel != nullptr) {
        corrs[omp_get_thread_num()].push_back(std::make_pair(i, voxel)); // Add voxel correspondence
      }
    }
  }

  voxel_correspondences_.reserve(input_->size() * offsets.size());
  for (const auto& c : corrs) { // Add voxel correspondence
    voxel_correspondences_.insert(voxel_correspondences_.end(), c.begin(), c.end());
  }

  // precompute combined covariances
  voxel_mahalanobis_.resize(voxel_correspondences_.size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
    const auto& corr = voxel_correspondences_[i]; // Point index and voxel pair
    const auto& cov_A = source_covs_[corr.first];
    const auto& cov_B = corr.second->cov; // Voxel covariance
    // const auto& cov_B = corr.second->kappa; // Voxel covariance
    // std::cout << "cov_A: " << cov_A << std::endl;
    // std::cout << "cov_B: " << cov_B << std::endl;
    // std::cout << "num: " << corr.second->num_points << std::endl;

    // GICP covariance sum
    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0;
    // Mahalanobis distance
    voxel_mahalanobis_[i] = RCR.inverse();
    voxel_mahalanobis_[i](3, 3) = 0.0;
    // std::cout << "voxel_mahalanobis_: " << voxel_mahalanobis_[i] << std::endl;
  }
}

template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  // Build voxel map if needed
  if (voxelmap_ == nullptr) {
    voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, polar_resolution_, voxel_mode_, voxel_type_));
    voxelmap_->create_voxelmap(*target_, target_covs_);
  }

  update_correspondences(trans); // Update correspondences after pose change
  // Hessian matrix
  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // Voxel mean and covariance
    // const Eigen::Vector4d dir_B = mean_B.array().acos(); // Direction cosine

    const Eigen::Vector4d transed_mean_A = trans * mean_A; // Transform point
    // const Eigen::Vector4d dir_A = (transed_mean_A / transed_mean_A.norm()).array().acos();  // Direction cosine
    const Eigen::Vector4d error = mean_B - transed_mean_A; // Mean residual

    double w = std::sqrt(target_voxel->num_points); // Denser voxels get higher weight
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error; // Residual computation

    if (H == nullptr || b == nullptr) {
      continue;
    }
    // Pose Jacobian from Lie perturbation
    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0; // Jacobian matrix
    // Hessian matrix
    Eigen::Matrix<double, 6, 6> Hi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * jlossexp;
    // Gradient vector
    Eigen::Matrix<double, 6, 1> bi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * error;

    int thread_num = omp_get_thread_num();
    Hs[thread_num] += Hi;
    bs[thread_num] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::so3_linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 3, 3>* H, Eigen::Matrix<double, 3, 1>* b) {
  // Build voxel map if needed
  if (voxelmap_ == nullptr) {
    voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, polar_resolution_, voxel_mode_, voxel_type_));
    voxelmap_->create_voxelmap(*target_, target_covs_);
  }
  // std::cout << target_covs_.size() << std::endl;
  // std::cout << target_covs_[0] << std::endl;
  // std::cout << "trans: " << trans.matrix() << std::endl;

  update_correspondences(trans); // Update correspondences after pose change
  // Hessian matrix
  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    // std::cout << "mean_A" << mean_A << std::endl;

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // Voxel mean and covariance
    // const auto& cov_B = corr.second->cov;
    const Eigen::Vector4d transed_mean_A = trans * mean_A; // Transform point
    const Eigen::Vector4d error = mean_B - transed_mean_A; // Mean residual

    // Eigen::Vector4d mean_A_dir;
    // mean_A_dir << mean_A.head<3>().normalized(), 1.0;
    // const Eigen::Vector4d mean_B_dir = corr.second->dir_reg; // Direction cosine
    // const Eigen::Vector4d transed_mean_A = trans * mean_A_dir; // Transform point
    // const Eigen::Vector4d error = mean_B_dir - transed_mean_A;  // Direction cosine residual

    double w = std::sqrt(target_voxel->num_points); // Denser voxels get higher weight
    // double w = target_voxel->kappa; // Denser voxels get higher weight
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error; // Residual computation

    if (H == nullptr || b == nullptr) {
      continue;
    }

    // std::cout << "transed_mean_A" << transed_mean_A << std::endl;  
    // std::cout << "voxel_mahalanobis_so3" << voxel_mahalanobis_[i] << std::endl;  

    // Pose Jacobian from Lie perturbation
    Eigen::Matrix<double, 3, 3> dtdx0 = Eigen::Matrix<double, 3, 3>::Zero();
    // std::cout << "transed_mean_A: " << transed_mean_A << std::endl; 
    dtdx0 = skewd(transed_mean_A.head<3>());
    // dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 3> jlossexp = dtdx0; // Jacobian matrix
    Eigen::Matrix3d voxel_mahalanobis_so3 = voxel_mahalanobis_[i].block<3,3>(0,0).matrix();
    // std::cout << "dtdx0" << dtdx0 << std::endl;
    // std::cout << "voxel_mahalanobis_so3" << voxel_mahalanobis_so3 << std::endl;
    // std::cout << "error" << error << std::endl;
    if(std::isnan(w)){
      std::cout << "target_voxel->num_points: " << target_voxel->num_points << std::endl;
      std::cout << "target_voxel->kappa: " << target_voxel->kappa << std::endl;
      std::cout << "target_voxel->r_bar: " << target_voxel->r_bar << std::endl;
      std::cout << "target_voxel->cov: " << target_voxel->cov << std::endl;
      std::cout << "target_voxel->mean_dir: " << target_voxel->mean_dir << std::endl;
    }

    // Hessian matrix
    Eigen::Matrix<double, 3, 3> Hi = w * jlossexp.transpose() * voxel_mahalanobis_so3 * jlossexp;
    // Gradient vector
    Eigen::Matrix<double, 3, 1> bi = w * jlossexp.transpose() * voxel_mahalanobis_so3 * error.head<3>();
    // std::cout << "Hi" << Hi << std::endl;  
    // std::cout << "bi" << bi << std::endl;  

    int thread_num = omp_get_thread_num();
    Hs[thread_num] += Hi;
    bs[thread_num] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }
  // std::cout << "H1: " << *H << std::endl;
  // std::cout << "b1: " << *b << std::endl;
  // std::cout << "sum_errors: " << sum_errors << std::endl;

  return sum_errors;
}
// //! Residual computation
template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;
#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors)
  for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    Eigen::Vector4d mean_A_dir;
    mean_A_dir << mean_A.head<3>().normalized(), 1.0;
    
    // const Eigen::Vector4d mean_B_dir = corr.second->dir_reg; // Voxel mean and covariance
    // const Eigen::Vector4d transed_mean_A = trans * mean_A_dir; // Transform point
    // const Eigen::Vector4d error = mean_B_dir - transed_mean_A; // Mean residual

    const Eigen::Vector4d mean_B = corr.second->mean_dir;
    const Eigen::Vector4d transed_mean_A = trans * mean_A; // Transform point
    const Eigen::Vector4d error = mean_B - transed_mean_A; // Mean residual

    double w = std::sqrt(target_voxel->num_points); // Denser voxels get higher weight
    // double w = target_voxel->kappa; // Denser voxels get higher weight
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error; // Residual computation
  }

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
template <typename PointT>
bool RotVGICP<PointSource, PointTarget>::calculate_covariances(
  const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
  pcl::search::Search<PointT>& kdtree,
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances) {
  if (kdtree.getInputCloud() != cloud) {
    kdtree.setInputCloud(cloud);
  }
  covariances.resize(cloud->size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (std::size_t i = 0; i < cloud->size(); ++i) { // Compute covariance per input point
    std::vector<int> k_indices;
    std::vector<float> k_sq_distances;  // Find nearest neighbors with KD-tree
    // std::cout << "size: " << cloud->size() << ", index: " << i << std::endl;
    // std::cout << "point: " << cloud->at(i).x << ", " << cloud->at(i).y << ", " << cloud->at(i).z << std::endl;
    kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);

    Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
    for (std::size_t j = 0; j < k_indices.size(); ++j) {  // Direction cosine sample
      Eigen::Vector4d neibor = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
      // neibor /= neibor.head<3>().norm(); // Normalize point direction
      // neibor(3) = 1.0;
      neighbors.col(j) = neibor;
    }
    // // Direction cosine
    // Eigen::Vector4d mean_dir = cloud->at(i).getVector4fMap().template cast<double>().normalized().acos();
    // Average direction vector
    // Eigen::Vector4d mean_dir = neighbors.rowwise().sum();
    // mean_dir /= mean_dir.head<3>().norm(); // Normalize point direction
    // mean_dir(3) = 1.0; // Mean dir
    // mean_dir = mean_dir.array().acos().eval();
    // // Average direction vector
    // neighbors.colwise() -= mean_dir; // P-bar(P)
    neighbors.colwise() -= neighbors.rowwise().mean().eval(); // P-bar(P)

    Eigen::Matrix4d cov = neighbors * neighbors.transpose() / k_correspondences_;

    if (regularization_method_ == RegularizationMethod::NONE) {
      covariances[i] = cov;
    } else if (regularization_method_ == RegularizationMethod::FROBENIUS) {
      double lambda = 1e-3;
      Eigen::Matrix3d C = cov.block<3, 3>(0, 0).cast<double>() + lambda * Eigen::Matrix3d::Identity();
      Eigen::Matrix3d C_inv = C.inverse();
      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = (C_inv / C_inv.norm()).inverse();
    } else {
      // SVD decomposition
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d values;

      switch (regularization_method_) {
        default:
          std::cerr << "here must not be reached" << std::endl;
          abort();
        case RegularizationMethod::PLANE:
          values = Eigen::Vector3d(1, 1, 1e-3); // Project covariance to xy plane
          break;
        case RegularizationMethod::MIN_EIG:
          values = svd.singularValues().array().max(1e-3);
          break;
        case RegularizationMethod::NORMALIZED_MIN_EIG:
          values = svd.singularValues() / svd.singularValues().maxCoeff();
          values = values.array().max(1e-3);
          break;
        case RegularizationMethod::PLANE_S:
          values = svd.singularValues() / svd.singularValues().sum();
          values(2) = 1e-3;
      }

      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
    }
  }

  return true;
}

template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::t3_linearize(const Eigen::Vector3d& trans, const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0, 
                                                        const double interval_tn, const double interval_tn_1,
                                                        Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  // Build voxel map if needed
  // if (voxelmap_ == nullptr) {
  //   voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, voxel_mode_));
  //   voxelmap_->create_voxelmap(*target_, target_covs_);
  // }


  // Eigen::Isometry3d tranform = Eigen::Isometry3d::Identity();
  // tranform.matrix().col(3).head<3>() = trans;
  // update_correspondences(tranform);
  
  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }
  size_t pt_size = voxel_correspondences_.size();

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // Voxel mean and covariance

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform(0,3) =  trans(0);
    transform(1,3) =  trans(1);
    transform(2,3) =  trans(2);
    
    Eigen::Isometry3d propagation_transform = Eigen::Isometry3d::Identity();
    propagation_transform.matrix().col(3).head<3>() = init_guess;
    Eigen::Vector4d last_transform = Eigen::Vector4d::Zero();
    last_transform.matrix().col(3).head<3>() = last_t0;

    const Eigen::Vector4d transed_mean_A = transform * mean_A; // Transform point

    const Eigen::Vector4d begin_mean_A = propagation_transform.inverse() * mean_A; // Pre-interpolation point

    const Eigen::Vector4d error = mean_B - transed_mean_A; // Mean residual

    const Eigen::Vector4d ct_error = (begin_mean_A - transed_mean_A)/interval_tn - last_transform/interval_tn_1;

    double w = std::sqrt(target_voxel->num_points); // Denser voxels get higher weight
    // double w = target_voxel->kappa; // Denser voxels get higher weight
    // double iter_error = w * (error.transpose() * voxel_mahalanobis_[i] * error + lambda/pt_size * C_vel.transpose()*C_vel).value();
    // sum_errors += iter_error; // Residual computation
    sum_errors += w * (error.transpose() * voxel_mahalanobis_[i] * error + lambda_/pt_size * ct_error.transpose() * voxel_mahalanobis_[i] * ct_error).value();

    if (H == nullptr || b == nullptr) {
      continue;
    }

    // std::cout << "transed_mean_A" << transed_mean_A << std::endl;  
    // std::cout << "voxel_mahalanobis_so3" << voxel_mahalanobis_[i] << std::endl;  
  
    if(std::isnan(w)){
      std::cout << "target_voxel->num_points: " << target_voxel->num_points << std::endl;
      std::cout << "target_voxel->kappa: " << target_voxel->kappa << std::endl;
      std::cout << "target_voxel->r_bar: " << target_voxel->r_bar << std::endl;
      std::cout << "target_voxel->cov: " << target_voxel->cov << std::endl;
      std::cout << "target_voxel->mean_dir: " << target_voxel->mean_dir << std::endl;
    }

    // Pose Jacobian from Lie perturbation
    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp1 = dtdx0; // Jacobian matrix
    Eigen::Matrix<double, 4, 6> jlossexp2 = 1.0 / interval_tn * dtdx0; // Jacobian matrix

    // Hessian matrix
    Eigen::Matrix<double, 6, 6> Hi = w * (jlossexp1.transpose() * voxel_mahalanobis_[i] * jlossexp1 + lambda_/pt_size * jlossexp2.transpose() * voxel_mahalanobis_[i] * jlossexp2);
    // Gradient vector
    Eigen::Matrix<double, 6, 1> bi = w * (jlossexp1.transpose() * voxel_mahalanobis_[i] * error + lambda_/pt_size * jlossexp2.transpose() * voxel_mahalanobis_[i] * ct_error);
    // std::cout << "Hi" << Hi << std::endl;  
    // std::cout << "bi" << bi << std::endl;  

    int thread_num = omp_get_thread_num();
    Hs[thread_num] += Hi;
    bs[thread_num] += bi;
  }

  if (H && b) {
    H->setZero();
    b->setZero();
    for (int i = 0; i < num_threads_; i++) {
      (*H) += Hs[i];
      (*b) += bs[i];
    }
  }

  // std::cout << "H1: " << *H << std::endl;
  // std::cout << "b1: " << *b << std::endl;
  // std::cout << "sum_errors: " << sum_errors << std::endl;

  return sum_errors;
}

template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::compute_t_error(const Eigen::Vector3d& trans, const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0, 
                                                           const double& interval_tn, const double& interval_tn_1){

  // Eigen::Isometry3d tranform = Eigen::Isometry3d::Identity();
  // tranform.matrix().col(3).head<3>() = trans;
  // update_correspondences(tranform);

  double sum_errors = 0.0;
  size_t pt_size = voxel_correspondences_.size();

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors)
  for (std::size_t i = 0; i < voxel_correspondences_.size(); ++i) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // Voxel mean and covariance

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform(0,3) =  trans(0);
    transform(1,3) =  trans(1);
    transform(2,3) =  trans(2);
    
    Eigen::Isometry3d propagation_transform = Eigen::Isometry3d::Identity();
    propagation_transform.matrix().col(3).head<3>() = init_guess;
    Eigen::Vector4d last_transform = Eigen::Vector4d::Identity();
    last_transform.matrix().col(3).head<3>() = last_t0;

    const Eigen::Vector4d transed_mean_A = transform * mean_A; // Transform point

    const Eigen::Vector4d begin_mean_A = propagation_transform.inverse() * mean_A; // Pre-interpolation point

    const Eigen::Vector4d error = mean_B - transed_mean_A; // Mean residual

    const Eigen::Vector4d ct_error = (begin_mean_A - transed_mean_A)/interval_tn - last_transform/interval_tn_1;

    double w = std::sqrt(target_voxel->num_points); // Denser voxels get higher weight
    // double w = target_voxel->kappa; // Denser voxels get higher weight
    // double iter_error = w * (error.transpose() * voxel_mahalanobis_[i] * error + lambda/pt_size * C_vel.transpose()*C_vel).value();
    // sum_errors += iter_error; // Residual computation
    sum_errors += w * (error.transpose() * voxel_mahalanobis_[i] * error + lambda_/pt_size * ct_error.transpose() * voxel_mahalanobis_[i] * ct_error).value();

  }

  return sum_errors;
}

}  // namespace fast_gicp

#endif  // ROT_VGICP_IMPL_HPP
