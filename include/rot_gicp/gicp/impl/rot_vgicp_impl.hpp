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
  search_method_ = NeighborSearchMethod::DIRECT1;
  voxel_mode_ = VoxelAccumulationMode::ADDITIVE;
}

template <typename PointSource, typename PointTarget>
RotVGICP<PointSource, PointTarget>::~RotVGICP() {}

template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::setResolution(double resolution) {
  voxel_resolution_ = resolution;
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
                        const double interval_tn, const double interval_tn_1) {
  LsqRegistration<PointSource, PointTarget>::computeTranslation(output, trans, init_guess, last_t0, interval_tn, interval_tn_1);
  // voxelmap_.reset();
}


template <typename PointSource, typename PointTarget>
void RotVGICP<PointSource, PointTarget>::update_correspondences(const Eigen::Isometry3d& trans) {
  voxel_correspondences_.clear();
  auto offsets = neighbor_offsets(search_method_);
  // 多线程分块处理
  std::vector<std::vector<std::pair<int, VmfVoxel::Ptr>>> corrs(num_threads_); // 第一维度是线程池
  for (auto& c : corrs) {
    c.reserve((input_->size() * offsets.size()) / num_threads_); // 预分配空间（元素数）
  }

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < input_->size(); i++) {
    const Eigen::Vector4d mean_A = input_->at(i).getVector4fMap().template cast<double>();
    Eigen::Vector4d transed_mean_A = trans * mean_A;
    Eigen::Vector3i coord = voxelmap_->voxel_coord(transed_mean_A); // 取体素索引

    for (const auto& offset : offsets) {
      auto voxel = voxelmap_->lookup_voxel(coord + offset); // 寻找该点周围的体素
      if (voxel != nullptr) {
        corrs[omp_get_thread_num()].push_back(std::make_pair(i, voxel)); // 建立corrs对
      }
    }
  }

  voxel_correspondences_.reserve(input_->size() * offsets.size());
  for (const auto& c : corrs) { // 将correspondence逐一添加到voxel_correspondences_
    voxel_correspondences_.insert(voxel_correspondences_.end(), c.begin(), c.end());
  }

  // precompute combined covariances
  voxel_mahalanobis_.resize(voxel_correspondences_.size());

#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i]; // 点的pcl索引：voxel
    const auto& cov_A = source_covs_[corr.first];
    const auto& cov_B = corr.second->cov; // 取体素存储的协方差
    // const auto& cov_B = corr.second->kappa; // 取体素存储的协方差
    // std::cout << "cov_A: " << cov_A << std::endl;
    // std::cout << "cov_B: " << cov_B << std::endl;
    // std::cout << "num: " << corr.second->num_points << std::endl;

    // 计算B+TAT^{T}
    Eigen::Matrix4d RCR = cov_B + trans.matrix() * cov_A * trans.matrix().transpose();
    RCR(3, 3) = 1.0;
    // mahalanobis-马氏距离
    voxel_mahalanobis_[i] = RCR.inverse();
    voxel_mahalanobis_[i](3, 3) = 0.0;
    // std::cout << "voxel_mahalanobis_: " << voxel_mahalanobis_[i] << std::endl;
  }
}

template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {
  // 若没有构建体素地图，先构建体素地图
  if (voxelmap_ == nullptr) {
    voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, voxel_mode_));
    voxelmap_->create_voxelmap(*target_, target_covs_);
  }

  update_correspondences(trans); // 执行一次位姿更新后的correspondence
  // 以下分别是总体误差，海森矩阵，偏置
  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[corr.first]; // 取协方差矩阵  

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // 体素栅格内的均值和协方差
    // const Eigen::Vector4d dir_B = mean_B.array().acos(); // 取方向余弦
    const auto& cov_B = corr.second->cov;

    const Eigen::Vector4d transed_mean_A = trans * mean_A; // 变换点
    // const Eigen::Vector4d dir_A = (transed_mean_A / transed_mean_A.norm()).array().acos();  // 取方向余弦
    const Eigen::Vector4d error = mean_B - transed_mean_A; // 均值误差

    double w = std::sqrt(target_voxel->num_points); // 体素内点越多且越密集，权重越大
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error; // 残差计算

    if (H == nullptr || b == nullptr) {
      continue;
    }
    // 利用李代数扰动模型，对位姿进行求导，得到雅可比矩阵
    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0; // 雅可比矩阵
    // 海森矩阵
    Eigen::Matrix<double, 6, 6> Hi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * jlossexp;
    // 偏置
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
  // 若没有构建体素地图，先构建体素地图
  if (voxelmap_ == nullptr) {
    voxelmap_.reset(new VmfVoxelMap<PointTarget>(voxel_resolution_, voxel_mode_));
    voxelmap_->create_voxelmap(*target_, target_covs_);
  }
  // std::cout << target_covs_.size() << std::endl;
  // std::cout << target_covs_[0] << std::endl;
  // std::cout << "trans: " << trans.matrix() << std::endl;

  update_correspondences(trans); // 执行一次位姿更新后的correspondence
  // 以下分别是总体误差，海森矩阵，偏置
  double sum_errors = 0.0;
  std::vector<Eigen::Matrix<double, 3, 3>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    // std::cout << "mean_A" << mean_A << std::endl;
    const auto& cov_A = source_covs_[corr.first]; // 取协方差矩阵  

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // 体素栅格内的均值和协方差
    // const Eigen::Vector4d dir_B = mean_B.array().acos(); // 取方向余弦
    // const auto& cov_B = corr.second->cov;

    const Eigen::Vector4d transed_mean_A = trans * mean_A; // 变换点
    // const Eigen::Vector3d transed_so3_A = transed_mean_A.head<3>();
    // Eigen::Vector4d dir_A = (transed_mean_A / transed_so3_A.norm()).array().acos();  // 取方向余弦
    // dir_A(3) = 0.0;
    // std::cout << "dir_A" << dir_A << std::endl;  
    // std::cout << "dir_B" << dir_B << std::endl;  

    // const Eigen::Vector4d error = dir_B - dir_A;
    const Eigen::Vector4d error = mean_B - transed_mean_A; // 均值误差

    double w = std::sqrt(target_voxel->num_points); // 体素内点越多且越密集，权重越大
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error; // 残差计算

    if (H == nullptr || b == nullptr) {
      continue;
    }

    // std::cout << "transed_mean_A" << transed_mean_A << std::endl;  
    // std::cout << "voxel_mahalanobis_so3" << voxel_mahalanobis_[i] << std::endl;  

    // 利用李代数扰动模型，对位姿进行求导，得到雅可比矩阵
    Eigen::Matrix<double, 3, 3> dtdx0 = Eigen::Matrix<double, 3, 3>::Zero();
    // std::cout << "transed_mean_A: " << transed_mean_A << std::endl; 
    dtdx0 = skewd(transed_mean_A.head<3>());
    // dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 3> jlossexp = dtdx0; // 雅可比矩阵
    Eigen::Matrix3d voxel_mahalanobis_so3 = voxel_mahalanobis_[i].block<3,3>(0,0).matrix();
    // std::cout << "dtdx0" << dtdx0 << std::endl;
    // std::cout << "voxel_mahalanobis_so3" << voxel_mahalanobis_so3 << std::endl;
    // std::cout << "error" << error << std::endl;
    if(isnan(w)){
      std::cout << "target_voxel->num_points: " << target_voxel->num_points << std::endl;
      std::cout << "target_voxel->kappa: " << target_voxel->kappa << std::endl;
      std::cout << "target_voxel->r_bar: " << target_voxel->r_bar << std::endl;
      std::cout << "target_voxel->cov: " << target_voxel->cov << std::endl;
      std::cout << "target_voxel->mean_dir: " << target_voxel->mean_dir << std::endl;
    }

    // 海森矩阵
    Eigen::Matrix<double, 3, 3> Hi = w * jlossexp.transpose() * voxel_mahalanobis_so3 * jlossexp;
    // 偏置
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
// //! 计算残差块
template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::compute_error(const Eigen::Isometry3d& trans) {
  double sum_errors = 0.0;
#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[corr.first]; // 取协方差矩阵  

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // 体素栅格内的均值和协方差

    const Eigen::Vector4d transed_mean_A = trans * mean_A; // 变换点

    const Eigen::Vector4d error = mean_B - transed_mean_A; // 均值误差

    double w = std::sqrt(target_voxel->num_points); // 体素内点越多且越密集，权重越大
    sum_errors += w * error.transpose() * voxel_mahalanobis_[i] * error; // 残差计算
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
  for (int i = 0; i < cloud->size(); i++) { // 为每个输入点云计算协方差
    std::vector<int> k_indices;
    std::vector<float> k_sq_distances;  // 利用kdtree 寻找最近的k_correspondences_个点
    // std::cout << "1111111111111111" << std::endl;
    // std::cout << "size: " << cloud->size() << ", index: " << i << std::endl;
    // std::cout << "point: " << cloud->at(i).x << ", " << cloud->at(i).y << ", " << cloud->at(i).z << std::endl;
    kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);

    Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
    for (int j = 0; j < k_indices.size(); j++) {  // 存的是方向余弦
      Eigen::Vector4d neibor = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
      // neibor /= neibor.head<3>().norm(); // 归一化
      // neibor(3) = 1.0;
      neighbors.col(j) = neibor;
    }
    // // 以第i个点为平均方向，求其他角的方向余弦插值
    // Eigen::Vector4d mean_dir = cloud->at(i).getVector4fMap().template cast<double>().normalized().acos();
    // 利用矢量和求平均方向向量
    // Eigen::Vector4d mean_dir = neighbors.rowwise().sum();
    // mean_dir /= mean_dir.head<3>().norm(); // 单位化
    // mean_dir(3) = 1.0; // se(3)影响
    // mean_dir = mean_dir.array().acos().eval();
    // // 协方差公式： dir-bar(dir)/K, bar(dir)是neibor的平均方向
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
      // SVD分解
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Vector3d values;

      switch (regularization_method_) {
        default:
          std::cerr << "here must not be reached" << std::endl;
          abort();
        case RegularizationMethod::PLANE:
          values = Eigen::Vector3d(1, 1, 1e-3); // 利用SVD矩阵将协方差降维到2维xy平面上
          break;
        case RegularizationMethod::MIN_EIG:
          values = svd.singularValues().array().max(1e-3);
          break;
        case RegularizationMethod::NORMALIZED_MIN_EIG:
          values = svd.singularValues() / svd.singularValues().maxCoeff();
          values = values.array().max(1e-3);
          break;
      }

      covariances[i].setZero();
      covariances[i].template block<3, 3>(0, 0) = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
    }
    // std::cout << "covariances: " << covariances[i] << std::endl;
  }

  return true;
}

template <typename PointSource, typename PointTarget>
double RotVGICP<PointSource, PointTarget>::t3_linearize(const Eigen::Vector3d& trans, const Eigen::Vector3d& init_guess, const Eigen::Vector3d& last_t0, 
                                                        const double interval_tn, const double interval_tn_1,
                                                        Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) {

  double sum_errors = 0.0;
  double lambda = 1.0;
  std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 3>>> Hs(num_threads_);
  std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1>>> bs(num_threads_);
  for (int i = 0; i < num_threads_; i++) {
    Hs[i].setZero();
    bs[i].setZero();
  }
  size_t pt_size = voxel_correspondences_.size();

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors) schedule(guided, 8)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[corr.first]; // 取协方差矩阵  

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // 体素栅格内的均值和协方差

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform(0,3) =  trans(0);
    transform(1,3) =  trans(1);
    transform(2,3) =  trans(2);
    const Eigen::Vector4d transed_mean_A = transform * mean_A; // 变换点

    const Eigen::Vector4d error = mean_B - transed_mean_A; // 均值误差

    double w = std::sqrt(target_voxel->num_points); // 体素内点越多且越密集，权重越大
    const Eigen::Vector3d C_vel = (init_guess+trans)/interval_tn - last_t0/interval_tn_1;
    // double iter_error = w * (error.transpose() * voxel_mahalanobis_[i] * error + lambda/pt_size * C_vel.transpose()*C_vel).value();
    // sum_errors += iter_error; // 残差计算
    sum_errors += w * (error.transpose() * voxel_mahalanobis_[i] * error).value();

    if (H == nullptr || b == nullptr) {
      continue;
    }

    // std::cout << "transed_mean_A" << transed_mean_A << std::endl;  
    // std::cout << "voxel_mahalanobis_so3" << voxel_mahalanobis_[i] << std::endl;  
    Eigen::Matrix3d voxel_mahalanobis_so3 = voxel_mahalanobis_[i].block<3,3>(0,0).matrix();

    const Eigen::Matrix3d Omega0 = voxel_mahalanobis_so3 + lambda/(pt_size*interval_tn*interval_tn) * Eigen::Matrix3d::Identity();
    const Eigen::Vector3d Omega1 = voxel_mahalanobis_so3 * error.head<3>() + lambda/(pt_size*interval_tn) * (init_guess/interval_tn - last_t0/interval_tn_1);
  
    if(isnan(w)){
      std::cout << "target_voxel->num_points: " << target_voxel->num_points << std::endl;
      std::cout << "target_voxel->kappa: " << target_voxel->kappa << std::endl;
      std::cout << "target_voxel->r_bar: " << target_voxel->r_bar << std::endl;
      std::cout << "target_voxel->cov: " << target_voxel->cov << std::endl;
      std::cout << "target_voxel->mean_dir: " << target_voxel->mean_dir << std::endl;
    }

    // 海森矩阵
    // Eigen::Matrix<double, 3, 3> Hi = w * Omega0;
    // Eigen::Matrix<double, 3, 3> Hi = w * voxel_mahalanobis_so3;
    // 偏置
    // Eigen::Matrix<double, 3, 1> bi = w * (Omega0*trans+Omega1);
    // Eigen::Matrix<double, 3, 1> bi = w * voxel_mahalanobis_so3 * (trans+error.head<3>());
    // 利用李代数扰动模型，对位姿进行求导，得到雅可比矩阵
    Eigen::Matrix<double, 4, 6> dtdx0 = Eigen::Matrix<double, 4, 6>::Zero();
    dtdx0.block<3, 3>(0, 0) = skewd(transed_mean_A.head<3>());
    dtdx0.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> jlossexp = dtdx0; // 雅可比矩阵
    // 海森矩阵
    Eigen::Matrix<double, 6, 6> Hi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * jlossexp;
    // 偏置
    Eigen::Matrix<double, 6, 1> bi = w * jlossexp.transpose() * voxel_mahalanobis_[i] * error;
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
  double sum_errors = 0.0;
  double lambda = 1.0;
  size_t pt_size = voxel_correspondences_.size();

#pragma omp parallel for num_threads(num_threads_) reduction(+ : sum_errors)
  for (int i = 0; i < voxel_correspondences_.size(); i++) {
    const auto& corr = voxel_correspondences_[i];
    auto target_voxel = corr.second;

    const Eigen::Vector4d mean_A = input_->at(corr.first).getVector4fMap().template cast<double>();
    const auto& cov_A = source_covs_[corr.first]; // 取协方差矩阵  

    const Eigen::Vector4d mean_B = corr.second->mean_dir; // 体素栅格内的均值和协方差

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.matrix().block<3, 1>(0, 3) = trans;
    const Eigen::Vector4d transed_mean_A = transform * mean_A; // 变换点

    const Eigen::Vector4d error = mean_B - transed_mean_A; // 均值误差

    double w = std::sqrt(target_voxel->num_points); // 体素内点越多且越密集，权重越大
    const Eigen::Vector3d C_vel = (init_guess+trans)/interval_tn - last_t0/interval_tn_1;
    // double iter_error = w * (error.transpose() * voxel_mahalanobis_[i] * error + lambda/pt_size * C_vel.transpose()*C_vel).value();
    // sum_errors += iter_error; // 残差计算
    sum_errors += w * (error.transpose() * voxel_mahalanobis_[i] * error).value();

  }

  return sum_errors;
}

}  // namespace fast_gicp

#endif  // ROT_VGICP_IMPL_HPP
