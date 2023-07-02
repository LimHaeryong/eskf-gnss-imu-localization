#ifndef _MATH_UTILS_HPP_
#define _MATH_UTILS_HPP_

#include <Eigen/Dense>

namespace MathUtils {

Eigen::Matrix3d vec3ToSkewSymmetric(const Eigen::Vector3d& vec3);
Eigen::Matrix4d quatToLeftProductMatrix(const Eigen::Quaterniond& quat);
Eigen::Matrix4d quatToRightProductMatrix(const Eigen::Quaterniond& quat);
Eigen::Matrix3d rotationVectorToRotationMatrix(const Eigen::Vector3d& vec3);
Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d& vec3);

}  // namespace MathUtils

#endif  // _MATH_UTILS_HPP_