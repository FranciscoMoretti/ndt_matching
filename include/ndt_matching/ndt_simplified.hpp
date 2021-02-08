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
  pcl::VoxelGridCovariance<pcl::PointXYZ> target_voxels_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_ = nullptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_ = nullptr;
  Eigen::Matrix<float, 4, 4> previous_transformation_;
  Eigen::Matrix<float, 4, 4> transformation_;
  Eigen::Matrix<float, 4, 4> final_transformation_;

  float resolution_;
  double step_size_;
  int max_iterations_;
  float transformation_epsilon_;
  float outlier_ratio_;

  int nr_iterations_;
  bool converged_;

  double gauss_d1_, gauss_d2_;

  Eigen::Matrix<double, 8, 4> angular_jacobian_;
  Eigen::Matrix<double, 15, 4> angular_hessian_;
  Eigen::Matrix<double, 3, 6> point_jacobian_;
  Eigen::Matrix<double, 18, 6> point_hessian_;

  void computeTransformation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & output, const Eigen::Matrix<float, 4,
    4> & guess);

  void initiateVoxels();

  double computeDerivatives(
    Eigen::Matrix<double, 6, 1> & score_gradient,
    Eigen::Matrix<double, 6, 6> & hessian,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & trans_cloud,
    const Eigen::Matrix<double, 6, 1> & transform,
    bool compute_hessian = true);

  void computePointDerivatives(
    const Eigen::Vector3d & x, bool compute_hessian = true);

  void computeAngleDerivatives(
    const Eigen::Matrix<double, 6, 1> & transform, bool compute_hessian = true);

  double updateDerivatives(
    Eigen::Matrix<double, 6, 1> & score_gradient,
    Eigen::Matrix<double, 6, 6> & hessian,
    const Eigen::Vector3d & x_trans,
    const Eigen::Matrix3d & c_inv,
    bool compute_hessian = true) const;

  double
  computeStepLengthMT(
    const Eigen::Matrix<double, 6, 1> & transform,
    Eigen::Matrix<double, 6, 1> & step_dir,
    double step_init,
    double step_max,
    double step_min,
    double & score,
    Eigen::Matrix<double, 6, 1> & score_gradient,
    Eigen::Matrix<double, 6, 6> & hessian,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & trans_cloud);
};

} // namespace ndt_matching

#endif // NDT_MATCHING__NDT_LIB_HPP_
