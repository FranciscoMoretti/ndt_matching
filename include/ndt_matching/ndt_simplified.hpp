#ifndef NDT_MATCHING__NDT_SIMPLIFIED_HPP_
#define NDT_MATCHING__NDT_SIMPLIFIED_HPP_

#include "ndt_matching/visibility_control.h"
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <string>

namespace ndt_matching
{

class NdtSimplified
{
public:
  NdtSimplified();

  virtual ~NdtSimplified();

  void setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  void setTransformationEpsilon(float epsilon);

  void setStepSize(float step_size);

  void setResolution(float resolution);

  void setMaximumIterations(int max_iterations);

  void setInputSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  void align(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output, const Eigen::Matrix<float, 4,
    4> & guess);

private:
  pcl::VoxelGridCovariance<pcl::PointXYZ> target_voxels;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_ = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_ = nullptr;

  float resolution_;
  double step_size_;
  int max_iterations_;
  float epsilon_;

  void initiateVoxels();
};

} // namespace ndt_matching

#endif // NDT_MATCHING__NDT_LIB_HPP_
