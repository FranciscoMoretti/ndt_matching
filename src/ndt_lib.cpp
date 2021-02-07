#include "ndt_matching/ndt_lib.hpp"

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

namespace ndt_matching
{

NdtLib::NdtLib()
{
  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon(0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize(0.1);
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution(1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations(35);
}

void NdtLib::point_cloud_map_callback(
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
  std::cout << "Loaded " << target_cloud->size() <<
    " data points PointCloud map" << std::endl;
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(target_cloud);
}

void NdtLib::set_initial_estimation(Eigen::Transform<float, 3, Eigen::Affine>& init_estimation){
  // Set initial alignment estimate found using robot odometry.
  _current_estimation = init_estimation.matrix();
}

Eigen::Transform<float, 3, Eigen::Affine> NdtLib::point_cloud_scan_callback(
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
  std::cout << "Loaded " << input_cloud->size() <<
    " data points from PointCloud scan" << std::endl;

  // Filtering input scan to roughly 10% of original size to increase speed of
  // registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size() <<
    " data points from room_scan2.pcd" << std::endl;

  // Setting point cloud to be aligned.
  ndt.setInputSource(filtered_cloud);

  // Calculating required rigid transform to align the input cloud to the target
  // cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*output_cloud, _current_estimation);

  std::cout << "Normal Distributions Transform has converged:" <<
    ndt.hasConverged() << " score: " << ndt.getFitnessScore() <<
    std::endl;

  _current_estimation = ndt.getFinalTransformation();
  Eigen::Transform<float, 3, Eigen::Affine> t (_current_estimation);

  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << _current_estimation << std::endl;
  return t;

}

NdtLib::~NdtLib() {}

} // namespace ndt_matching
