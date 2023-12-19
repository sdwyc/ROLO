#ifndef ROT_VGICP_HPP
#define ROT_VGICP_HPP

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/registration/registration.h>

#include <rot_gicp/gicp/lsq_registration.hpp>
#include <rot_gicp/gicp/gicp_settings.hpp>
#include <rot_gicp/gicp/vmp_voxel.hpp>

namespace fast_gicp {

/**
 * @brief Rotation GICP algorithm boosted with OpenMP
 */
template<typename PointSource, typename PointTarget>
class RotVGICP : public LsqRegistration<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

#if PCL_VERSION >= PCL_VERSION_CALC(1, 10, 0)
  using Ptr = pcl::shared_ptr<RotVGICP<PointSource, PointTarget>>;
  using ConstPtr = pcl::shared_ptr<const RotVGICP<PointSource, PointTarget>>;
#else
  using Ptr = boost::shared_ptr<RotVGICP<PointSource, PointTarget>>;
  using ConstPtr = boost::shared_ptr<const RotVGICP<PointSource, PointTarget>>;
#endif

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;

  // using FastGICP<PointSource, PointTarget>::num_threads_;
  // using FastGICP<PointSource, PointTarget>::k_correspondences_;
  // using FastGICP<PointSource, PointTarget>::regularization_method_;
  // using FastGICP<PointSource, PointTarget>::search_source_;
  // using FastGICP<PointSource, PointTarget>::search_target_;
  // using FastGICP<PointSource, PointTarget>::source_covs_;
  // using FastGICP<PointSource, PointTarget>::target_covs_;
  int num_threads_;
  int k_correspondences_;

  RegularizationMethod regularization_method_;
  typedef pcl::search::KdTree<PointSource> SearchMethodSource;
  typedef pcl::search::KdTree<PointTarget> SearchMethodTarget;
  std::shared_ptr<SearchMethodSource> search_source_;
  std::shared_ptr<SearchMethodTarget> search_target_;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> source_covs_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> target_covs_;


public:
  RotVGICP();
  virtual ~RotVGICP() override;

  void setResolution(double resolution);
  void setVoxelAccumulationMode(VoxelAccumulationMode mode);
  void setNeighborSearchMethod(NeighborSearchMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;

  void setNumThreads(int n);
  void setCorrespondenceRandomness(int k);
  void setRegularizationMethod(RegularizationMethod method);

  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);
  virtual void setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getSourceCovariances() const {
    return source_covs_;
  }

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getTargetCovariances() const {
    return target_covs_;
  }



protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;
  virtual void update_correspondences(const Eigen::Isometry3d& trans);
  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) override;
  virtual double so3_linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 3, 3>* H = nullptr, Eigen::Matrix<double, 3, 1>* b = nullptr) override;
  virtual double compute_error(const Eigen::Isometry3d& trans) override;
  
  template <typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, pcl::search::Search<PointT>& kdtree, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances);

protected:
  double voxel_resolution_;
  NeighborSearchMethod search_method_;
  VoxelAccumulationMode voxel_mode_;

  std::unique_ptr<VmfVoxelMap<PointTarget>> voxelmap_;

  std::vector<std::pair<int, VmfVoxel::Ptr>> voxel_correspondences_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> voxel_mahalanobis_;
};
}  // namespace fast_gicp

#endif  // ROT_VGICP_HPP
