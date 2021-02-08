#include "ndt_matching/ndt_simplified.hpp"

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transformation_from_correspondences.h>

namespace ndt_matching
{


/** \brief Convert 6 element transformation vector to affine transformation.
 * \param[in] x transformation vector of the form [x, y, z, roll, pitch, yaw]
 * \param[out] trans affine transform corresponding to given transfomation vector
 */
static void
convertTransform(const Eigen::Matrix<double, 6, 1> & x, Eigen::Affine3f & trans)
{
  // Extracted from PCL
  trans = Eigen::Translation<float, 3>(x.head<3>().cast<float>()) *
    Eigen::AngleAxis<float>(float(x(3)), Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxis<float>(float(x(4)), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxis<float>(float(x(5)), Eigen::Vector3f::UnitZ());
}

/** \brief Convert 6 element transformation vector to transformation matrix.
 * \param[in] x transformation vector of the form [x, y, z, roll, pitch, yaw]
 * \param[out] trans 4x4 transformation matrix corresponding to given transfomation
 * vector
 */
static void
convertTransform(const Eigen::Matrix<double, 6, 1> & x, Eigen::Matrix4f & trans)
{
  Eigen::Affine3f _affine;
  convertTransform(x, _affine);
  trans = _affine.matrix();
}


NdtSimplified::NdtSimplified()
{
  // Setting default NDT parameters
  // Setting minimum transformation difference for termination condition.
  transformation_epsilon_ = 0.01;
  // Setting maximum step size for More-Thuente line search.
  step_size_ = 0.1;
  // Setting Resolution of NDT grid structure (VoxelGridCovariance).
  resolution_ = 1.0;

  outlier_ratio_ = 0.55;

  // Initializes the gaussian fitting parameters (eq. 6.8) [Magnusson 2009]
  const double gauss_c1 = 10.0 * (1 - outlier_ratio_);
  const double gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  const double gauss_d3 = -std::log(gauss_c2);
  gauss_d1_ = -std::log(gauss_c1 + gauss_c2) - gauss_d3;
  gauss_d2_ =
    -2 * std::log((-std::log(gauss_c1 * std::exp(-0.5) + gauss_c2) - gauss_d3) /
      gauss_d1_);

  transformation_epsilon_ = 0.1;
  max_iterations_ = 35;
}

void NdtSimplified::setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  // Save to compute later if resolution changes
  target_ = cloud;
  // Initiate voxel structure.
  initiateVoxels();
}

void NdtSimplified::setTransformationEpsilon(float epsilon)
{
  transformation_epsilon_ = epsilon;
}

void NdtSimplified::setStepSize(float step_size)
{
  step_size_ = step_size;
}

void NdtSimplified::setResolution(float resolution)
{
  if (resolution_ != resolution) {
    resolution_ = resolution;
    // Create voxels with new resolution
    initiateVoxels();
  }
}

void NdtSimplified::setMaximumIterations(int max_iterations)
{
  max_iterations_ = max_iterations;
}

void NdtSimplified::setInputSource(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  // Save to compute when align method is called
  input_ = cloud;
}

void NdtSimplified::computeTransformation(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output,
  const Eigen::Matrix<float, 4, 4> & guess)
{


  if (guess != Eigen::Matrix4f::Identity()) {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    pcl::transformPointCloud(output, output, guess);
  }

  // Initialize Point Gradient and Hessian
  point_jacobian_.setZero();
  point_jacobian_.block<3, 3>(0, 0).setIdentity();
  point_hessian_.setZero();

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix() = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> transform, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation();
  Eigen::Vector3f init_rotation = eig_transformation.rotation().eulerAngles(0, 1, 2);
  transform << init_translation.cast<double>(), init_rotation.cast<double>();

  Eigen::Matrix<double, 6, 6> hessian;

  // Calculate derivates of initial transform vector, subsequent derivative calculations
  // are done in the step length determination.
  double score = computeDerivatives(score_gradient, hessian, output, transform);


  // Algorithm description
  // while not converged do
  converged_ = false;
  nr_iterations_ = 0;
  while (!converged_) {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for decent direction using newton method, line 23 in Algorithm 2 [Magnusson
    // 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(
      hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    Eigen::Matrix<double, 6, 1> delta = sv.solve(-score_gradient);

    // Calculate step length with guarnteed sufficient decrease [More, Thuente 1994]
    double delta_norm = delta.norm();

    if (delta_norm == 0 || std::isnan(delta_norm)) {
      converged_ = delta_norm == 0;
      return;
    }

    delta /= delta_norm;

    // TODO Continue adding functions
    // There is more code to be added but I didn't have enought time.
    //
    // delta_norm = computeStepLengthMT(transform,
    //                                  delta,
    //                                  delta_norm,
    //                                  step_size_,
    //                                  transformation_epsilon_ / 2,
    //                                  score,
    //                                  score_gradient,
    //                                  hessian,
    //                                  output);
    delta *= delta_norm;

    // Convert delta into matrix form
    convertTransform(delta, transformation_);

    transform += delta;

    const double cos_angle =
      0.5 * transformation_.template block<3, 3>(0, 0).trace() - 1;
    const double translation_sqr =
      transformation_.template block<3, 1>(0, 3).squaredNorm();

    nr_iterations_++;

    if (nr_iterations_ >= max_iterations_ ||
      ((transformation_epsilon_ > 0 && translation_sqr <= transformation_epsilon_) &&
      (transformation_rotation_epsilon_ > 0 &&
      cos_angle >= transformation_rotation_epsilon_)) ||
      ((transformation_epsilon_ <= 0) &&
      (transformation_rotation_epsilon_ > 0 &&
      cos_angle >= transformation_rotation_epsilon_)) ||
      ((transformation_epsilon_ > 0 && translation_sqr <= transformation_epsilon_) &&
      (transformation_rotation_epsilon_ <= 0)))
    {
      converged_ = true;
    }
  }

}

void NdtSimplified::computeAngleDerivatives(
  const Eigen::Matrix<double, 6, 1> & transform, bool compute_hessian)
{
  // Function extracted from PCL library

  // Simplified math for near 0 angles
  const auto calculate_cos_sin = [](double angle, double & c, double & s) {
      if (std::abs(angle) < 10e-5) {
        c = 1.0;
        s = 0.0;
      } else {
        c = std::cos(angle);
        s = std::sin(angle);
      }
    };

  double cx, cy, cz, sx, sy, sz;
  calculate_cos_sin(transform(3), cx, sx);
  calculate_cos_sin(transform(4), cy, sy);
  calculate_cos_sin(transform(5), cz, sz);

  // Precomputed angular gradient components. Letters correspond to Equation 6.19
  // [Magnusson 2009]
  angular_jacobian_.setZero();
  angular_jacobian_.row(0).noalias() = Eigen::Vector4d(
    (-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy), 1.0);   // a
  angular_jacobian_.row(1).noalias() = Eigen::Vector4d(
    (cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy), 1.0);   // b
  angular_jacobian_.row(2).noalias() =
    Eigen::Vector4d((-sy * cz), sy * sz, cy, 1.0);   // c
  angular_jacobian_.row(3).noalias() =
    Eigen::Vector4d(sx * cy * cz, (-sx * cy * sz), sx * sy, 1.0);   // d
  angular_jacobian_.row(4).noalias() =
    Eigen::Vector4d((-cx * cy * cz), cx * cy * sz, (-cx * sy), 1.0);   // e
  angular_jacobian_.row(5).noalias() =
    Eigen::Vector4d((-cy * sz), (-cy * cz), 0, 1.0);   // f
  angular_jacobian_.row(6).noalias() =
    Eigen::Vector4d((cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0, 1.0);   // g
  angular_jacobian_.row(7).noalias() =
    Eigen::Vector4d((sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0, 1.0);   // h
  if (compute_hessian) {
    // Precomputed angular hessian components. Letters correspond to Equation 6.21 and
    // numbers correspond to row index [Magnusson 2009]
    angular_hessian_.setZero();
    angular_hessian_.row(0).noalias() = Eigen::Vector4d(
      (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy, 0.0f);   // a2
    angular_hessian_.row(1).noalias() = Eigen::Vector4d(
      (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), (-cx * cy), 0.0f);   // a3

    angular_hessian_.row(2).noalias() =
      Eigen::Vector4d((cx * cy * cz), (-cx * cy * sz), (cx * sy), 0.0f);   // b2
    angular_hessian_.row(3).noalias() =
      Eigen::Vector4d((sx * cy * cz), (-sx * cy * sz), (sx * sy), 0.0f);   // b3

    angular_hessian_.row(4).noalias() = Eigen::Vector4d(
      (-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0, 0.0f);   // c2
    angular_hessian_.row(5).noalias() = Eigen::Vector4d(
      (cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0, 0.0f);   // c3

    angular_hessian_.row(6).noalias() =
      Eigen::Vector4d((-cy * cz), (cy * sz), (sy), 0.0f);   // d1
    angular_hessian_.row(7).noalias() =
      Eigen::Vector4d((-sx * sy * cz), (sx * sy * sz), (sx * cy), 0.0f);   // d2
    angular_hessian_.row(8).noalias() =
      Eigen::Vector4d((cx * sy * cz), (-cx * sy * sz), (-cx * cy), 0.0f);   // d3

    angular_hessian_.row(9).noalias() =
      Eigen::Vector4d((sy * sz), (sy * cz), 0, 0.0f);   // e1
    angular_hessian_.row(10).noalias() =
      Eigen::Vector4d((-sx * cy * sz), (-sx * cy * cz), 0, 0.0f);   // e2
    angular_hessian_.row(11).noalias() =
      Eigen::Vector4d((cx * cy * sz), (cx * cy * cz), 0, 0.0f);   // e3

    angular_hessian_.row(12).noalias() =
      Eigen::Vector4d((-cy * cz), (cy * sz), 0, 0.0f);   // f1
    angular_hessian_.row(13).noalias() = Eigen::Vector4d(
      (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0, 0.0f);   // f2
    angular_hessian_.row(14).noalias() = Eigen::Vector4d(
      (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0, 0.0f);   // f3
  }

}

double NdtSimplified::computeDerivatives(
  Eigen::Matrix<double, 6, 1> & score_gradient,
  Eigen::Matrix<double, 6, 6> & hessian,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & trans_cloud,
  const Eigen::Matrix<double, 6, 1> & transform,
  bool compute_hessian)
{
  // Function extracted from PCL library

  score_gradient.setZero();
  hessian.setZero();
  double score = 0;

  // Precompute Angular Derivatives (eq. 6.19 and 6.21)[Magnusson 2009]
  computeAngleDerivatives(transform);

  // Update gradient and hessian for each point, line 17 in Algorithm 2 [Magnusson 2009]
  for (std::size_t idx = 0; idx < input_->size(); idx++) {
    // Transformed Point
    const auto & x_trans_pt = trans_cloud[idx];

    // Find neighbors (Radius search has been experimentally faster than direct neighbor
    // checking.
    std::vector<pcl::VoxelGridCovariance<pcl::PointXYZ>::LeafConstPtr> neighborhood;
    std::vector<float> distances;
    target_voxels_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

    for (const auto & cell : neighborhood) {
      // Original Point
      const auto & x_pt = (*input_)[idx];
      const Eigen::Vector3d x = x_pt.getVector3fMap().template cast<double>();

      // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
      const Eigen::Vector3d x_trans =
        x_trans_pt.getVector3fMap().template cast<double>() - cell->getMean();
      // Inverse Covariance of Occupied Voxel
      // Uses precomputed covariance for speed.
      const Eigen::Matrix3d c_inv = cell->getInverseCov();

      // Compute derivative of transform function w.r.t. transform vector, J_E and H_E
      // in Equations 6.18 and 6.20 [Magnusson 2009]
      computePointDerivatives(x);
      // Update score, gradient and hessian, lines 19-21 in Algorithm 2, according to
      // Equations 6.10, 6.12 and 6.13, respectively [Magnusson 2009]
      score +=
        updateDerivatives(score_gradient, hessian, x_trans, c_inv, compute_hessian);
    }
  }
  return score;
}

void NdtSimplified::computePointDerivatives(
  const Eigen::Vector3d & x, bool compute_hessian)
{
  // Function extracted from PCL library

  // Calculate first derivative of Transformation Equation 6.17 w.r.t. transform vector.
  // Derivative w.r.t. ith element of transform vector corresponds to column i,
  // Equation 6.18 and 6.19 [Magnusson 2009]
  Eigen::Matrix<double, 8, 1> point_angular_jacobian =
    angular_jacobian_ * Eigen::Vector4d(x[0], x[1], x[2], 0.0);
  point_jacobian_(1, 3) = point_angular_jacobian[0];
  point_jacobian_(2, 3) = point_angular_jacobian[1];
  point_jacobian_(0, 4) = point_angular_jacobian[2];
  point_jacobian_(1, 4) = point_angular_jacobian[3];
  point_jacobian_(2, 4) = point_angular_jacobian[4];
  point_jacobian_(0, 5) = point_angular_jacobian[5];
  point_jacobian_(1, 5) = point_angular_jacobian[6];
  point_jacobian_(2, 5) = point_angular_jacobian[7];

  if (compute_hessian) {
    Eigen::Matrix<double, 15, 1> point_angular_hessian =
      angular_hessian_ * Eigen::Vector4d(x[0], x[1], x[2], 0.0);

    // Vectors from Equation 6.21 [Magnusson 2009]
    const Eigen::Vector3d a(0, point_angular_hessian[0], point_angular_hessian[1]);
    const Eigen::Vector3d b(0, point_angular_hessian[2], point_angular_hessian[3]);
    const Eigen::Vector3d c(0, point_angular_hessian[4], point_angular_hessian[5]);
    const Eigen::Vector3d d = point_angular_hessian.block<3, 1>(6, 0);
    const Eigen::Vector3d e = point_angular_hessian.block<3, 1>(9, 0);
    const Eigen::Vector3d f = point_angular_hessian.block<3, 1>(12, 0);

    // Calculate second derivative of Transformation Equation 6.17 w.r.t. transform
    // vector. Derivative w.r.t. ith and jth elements of transform vector corresponds to
    // the 3x1 block matrix starting at (3i,j), Equation 6.20 and 6.21 [Magnusson 2009]
    point_hessian_.block<3, 1>(9, 3) = a;
    point_hessian_.block<3, 1>(12, 3) = b;
    point_hessian_.block<3, 1>(15, 3) = c;
    point_hessian_.block<3, 1>(9, 4) = b;
    point_hessian_.block<3, 1>(12, 4) = d;
    point_hessian_.block<3, 1>(15, 4) = e;
    point_hessian_.block<3, 1>(9, 5) = c;
    point_hessian_.block<3, 1>(12, 5) = e;
    point_hessian_.block<3, 1>(15, 5) = f;
  }
}

double NdtSimplified::updateDerivatives(
  Eigen::Matrix<double, 6, 1> & score_gradient,
  Eigen::Matrix<double, 6, 6> & hessian,
  const Eigen::Vector3d & x_trans,
  const Eigen::Matrix3d & c_inv,
  bool compute_hessian) const
{
  // Function extracted from PCL library

  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
  double e_x_cov_x = std::exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);
  // Calculate probability of transformed points existence, Equation 6.9 [Magnusson
  // 2009]
  const double score_inc = -gauss_d1_ * e_x_cov_x;

  e_x_cov_x = gauss_d2_ * e_x_cov_x;

  // Error checking for invalid values.
  if (e_x_cov_x > 1 || e_x_cov_x < 0 || std::isnan(e_x_cov_x)) {
    return 0;
  }

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  e_x_cov_x *= gauss_d1_;

  for (int i = 0; i < 6; i++) {
    // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13 [Magnusson
    // 2009]
    const Eigen::Vector3d cov_dxd_pi = c_inv * point_jacobian_.col(i);

    // Update gradient, Equation 6.12 [Magnusson 2009]
    score_gradient(i) += x_trans.dot(cov_dxd_pi) * e_x_cov_x;

    if (compute_hessian) {
      for (Eigen::Index j = 0; j < hessian.cols(); j++) {
        // Update hessian, Equation 6.13 [Magnusson 2009]
        hessian(i, j) +=
          e_x_cov_x * (-gauss_d2_ * x_trans.dot(cov_dxd_pi) *
          x_trans.dot(c_inv * point_jacobian_.col(j)) +
          x_trans.dot(c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
          point_jacobian_.col(j).dot(cov_dxd_pi));
      }
    }
  }

  return score_inc;
}

NdtSimplified::~NdtSimplified() {}

} // namespace ndt_matching
