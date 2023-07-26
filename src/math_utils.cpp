#include <cmath>

#include "eskf_gnss_imu_localization/math_utils.hpp"

namespace MathUtils {

Eigen::Matrix3d vec3ToSkewSymmetric(const Eigen::Vector3d& vec3) {
  Eigen::Matrix3d skewSymmetric;
  skewSymmetric << 0.0, -vec3.z(), vec3.y(), vec3.z(), 0.0, -vec3.x(), -vec3.y(), vec3.x(), 0.0;
  return skewSymmetric;
}

Eigen::Matrix4d quatToLeftProductMatrix(const Eigen::Quaterniond& quat) {
  Eigen::Matrix4d leftProductMatrix = Eigen::Matrix4d::Identity() * quat.w();
  leftProductMatrix.block<1, 3>(3, 0) += quat.vec();
  leftProductMatrix.block<3, 1>(0, 3) -= quat.vec();
  leftProductMatrix.block<3, 3>(1, 1) += vec3ToSkewSymmetric(quat.vec());
  return leftProductMatrix;
}

Eigen::Matrix4d quatToRightProductMatrix(const Eigen::Quaterniond& quat) {
  Eigen::Matrix4d RightProductMatrix = Eigen::Matrix4d::Identity() * quat.w();
  RightProductMatrix.block<1, 3>(3, 0) += quat.vec();
  RightProductMatrix.block<3, 1>(0, 3) += quat.vec();
  RightProductMatrix.block<3, 3>(1, 1) += vec3ToSkewSymmetric(quat.vec());
  return RightProductMatrix;
}

Eigen::Matrix3d rotationVectorToRotationMatrix(const Eigen::Vector3d& vec3) {
  double phi = vec3.norm();
  Eigen::Vector3d u = vec3.normalized();
  Eigen::Matrix3d uSkew = vec3ToSkewSymmetric(u);
  Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity() * std::cos(phi);
  rotationMatrix += uSkew * std::sin(phi);
  rotationMatrix += u * u.transpose() * (1.0 - std::cos(phi));
  return rotationMatrix;
}

Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d& vec3) {
  double phi_2 = vec3.norm() / 2.0;
  Eigen::Vector3d u = vec3.normalized() * std::sin(phi_2);
  Eigen::Quaterniond quaternion(std::cos(phi_2), u(0), u(1), u(2));
  return quaternion;
}

double degreeToRadian(double degree){
  return degree * M_PI / 180.0;
}

}  // namespace MathUtils