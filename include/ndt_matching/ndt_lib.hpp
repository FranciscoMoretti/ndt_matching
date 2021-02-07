#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_

#include "ndt_matching/visibility_control.h"
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <string>

namespace ndt_matching
{

class NdtLib
{
public:
  NdtLib();

  virtual ~NdtLib();

  void set_initial_estimation(Eigen::Transform<float, 3, Eigen::Affine>& init_estimation);

  void
  point_cloud_map_callback(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);

  Eigen::Transform<float, 3, Eigen::Affine>
  point_cloud_scan_callback(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);

private:
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt =
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr _target_cloud = nullptr;
  Eigen::Matrix4f _current_estimation = Eigen::Matrix4f();
};

} // namespace ndt_matching

#endif // NDT_MATCHING__NDT_LIB_HPP_
