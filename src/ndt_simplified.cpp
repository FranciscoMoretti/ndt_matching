#include "ndt_matching/ndt_simplified.hpp"

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/approximate_voxel_grid.h>

namespace ndt_matching
{

NdtSimplified::NdtSimplified()
{
  // Setting default NDT parameters
  // Setting minimum transformation difference for termination condition.
  epsilon_ = 0.01;
  // Setting maximum step size for More-Thuente line search.
  step_size_ = 0.1;
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  resolution_ = 1.0;

  // Setting max number of registration iterations.
  max_iterations_ = 35;
}

void NdtSimplified::setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud){
  // Save to compute later if resolution changes
  target_ = cloud;
  // Initiate voxel structure.
  initiateVoxels();
}

void NdtSimplified::setTransformationEpsilon(float epsilon){
  epsilon_ = epsilon;
}

void  NdtSimplified::setStepSize(float step_size){
  step_size_ = step_size;
}

void NdtSimplified::setResolution(float resolution){
  if(resolution_ != resolution){
    resolution_ = resolution;
    // Create voxels with new resolution
    initiateVoxels();
  };
}

void NdtSimplified::setMaximumIterations(int max_iterations){
  max_iterations_ = max_iterations;
}

void NdtSimplified::setInputSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud){
  // Save to compute when align method is called
  input_ = cloud;
}

void align(pcl::PointCloud<pcl::PointXYZ>::Ptr & output, const Eigen::Matrix<float, 4, 4> & guess){
  // undefined on this commit
}

NdtSimplified::~NdtSimplified() {}

} // namespace ndt_matching
