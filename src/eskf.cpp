#include <cmath>

#include <spdlog/spdlog.h>

#include "eskf_gnss_imu_localization/eskf.hpp"
#include "eskf_gnss_imu_localization/math_utils.hpp"

ErrorStateKalmanFilter::ErrorStateKalmanFilter(const YAML::Node& imuCalibration)
    : nominal_pos_(Eigen::Vector3d::Zero()),
      nominal_vel_(Eigen::Vector3d::Zero()),
      nominal_attitude_(Eigen::Quaterniond::Identity()),
      nominal_accel_bias_(Eigen::Vector3d::Zero()),
      nominal_gyro_bias_(Eigen::Vector3d::Zero()),
      nominal_gravity_(0.0, 0.0, -9.8),
      error_x_(Vec18d::Zero()),
      P_(Mat18d::Identity()),
      F_x_(Mat18d::Identity()),
      F_i_(Eigen::Matrix<double, 18, 12>::Zero()),
      Q_i_(Mat12d::Identity()),
      H_x_(Eigen::Matrix<double, 6, 19>::Zero()),
      J_true_error_(Eigen::Matrix<double, 19, 18>::Zero()),
      V_(Mat6d::Identity()),
      G_(Mat18d::Identity()) {
  F_i_.block<12, 12>(3, 0) = Mat12d::Identity();
  H_x_.block<6, 6>(0, 0) = Mat6d::Identity();
  J_true_error_.block<6, 6>(0, 0) = Mat6d::Identity();
  J_true_error_.block<9, 9>(10, 9) = Mat9d::Identity();
  J_true_error_.block<4, 3>(6, 6) = computeQuatJacobiToErrorQuat();

  auto accel_noise = imuCalibration["accelerometer_noise"].as<double>();
  auto accel_bias_stability = imuCalibration["accelerometer_bias_stability"].as<double>();
  auto gyro_noise = MathUtils::degreeToRadian(imuCalibration["gyroscope_noise"].as<double>());
  auto gyro_bias_stability = MathUtils::degreeToRadian(imuCalibration["gyroscope_bias_stability"].as<double>());
  auto update_rate = imuCalibration["update_rate"].as<double>();

  Q_i_.block<3, 3>(0, 0) = std::pow(accel_noise, 2) * update_rate * Eigen::Matrix3d::Identity();
  Q_i_.block<3, 3>(3, 3) = std::pow(gyro_noise, 2) * update_rate * Eigen::Matrix3d::Identity();
  Q_i_.block<3, 3>(6, 6) = std::pow(accel_bias_stability * update_rate, 2) * Eigen::Matrix3d::Identity();
  Q_i_.block<3, 3>(9, 9) = std::pow(gyro_bias_stability * update_rate, 2) * Eigen::Matrix3d::Identity();
  
}

void ErrorStateKalmanFilter::printState() const {
  SPDLOG_INFO("state position : {:.3f}, {:.3f}, {:.3f}", nominal_pos_(0), nominal_pos_(1), nominal_pos_(2));
  SPDLOG_INFO("state velocity : {:.3f}, {:.3f}, {:.3f}", nominal_vel_(0), nominal_vel_(1), nominal_vel_(2));
  SPDLOG_INFO("state quaternion : {:.3f}, {:.3f}, {:.3f}, {:.3f}", nominal_attitude_.w(), nominal_attitude_.x(),
              nominal_attitude_.y(), nominal_attitude_.z());
  SPDLOG_INFO("state gravity : {:.3f}, {:.3f}, {:.3f}", nominal_gravity_(0), nominal_gravity_(1), nominal_gravity_(2));
}

void ErrorStateKalmanFilter::predictWithImu(std::shared_ptr<ImuMeasurement> imuData) {
  std::chrono::duration<double> duration = imuData->timestamp - prev_time_;
  prev_time_ = imuData->timestamp;
  double dt = duration.count();
  if (dt > 0.05) {
    dt = 0.01;
  }
  double dt2 = dt * dt;

  Eigen::Matrix3d R = nominal_attitude_.toRotationMatrix();
  Eigen::Vector3d accel_diff = imuData->acceleration - nominal_accel_bias_;
  Eigen::Vector3d gyro_diff = imuData->angularVelocity - nominal_gyro_bias_;

  nominal_pos_ += nominal_vel_ * dt + 0.5 * (R * accel_diff + nominal_gravity_) * dt2;
  nominal_vel_ += (R * accel_diff + nominal_gravity_) * dt;
  Eigen::Matrix3d dR = MathUtils::rotationVectorToRotationMatrix(gyro_diff * dt);
  nominal_attitude_ = Eigen::Quaterniond(R * dR);
  nominal_attitude_.normalize();

  F_x_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3, 3>(3, 6) = -1.0 * R * MathUtils::vec3ToSkewSymmetric(accel_diff) * dt;
  F_x_.block<3, 3>(3, 9) = -1.0 * R * dt;
  F_x_.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  F_x_.block<3, 3>(6, 6) = dR.transpose();
  F_x_.block<3, 3>(6, 12) = -1.0 * Eigen::Matrix3d::Identity() * dt;

  P_ = F_x_ * P_ * F_x_.transpose() + F_i_ * Q_i_ * F_i_.transpose() * dt2;
}

void ErrorStateKalmanFilter::updateWithGnss(std::shared_ptr<GnssMeasurement> gnssData) {
  V_.block<3, 3>(0, 0) = gnssData->positionCovariance;
  J_true_error_.block<4, 3>(6, 6) = computeQuatJacobiToErrorQuat();

  H_ = H_x_ * J_true_error_;
  Eigen::Matrix<double, 18, 6> H_t = H_.transpose();

  K_ = P_ * H_t * (H_ * P_ * H_t + V_).inverse();
  z_.block<3, 1>(0, 0) = llaToEnu(gnssData->position);
  z_.block<3, 1>(3, 0) = gnssData->linearVelocity;

  error_x_ = K_ * (z_ - computeHx());
  P_ = (Mat18d::Identity() - K_ * H_) * P_;

  injectErrorToNominal();
  resetErrorState();
}

Eigen::Matrix<double, 4, 3> ErrorStateKalmanFilter::computeQuatJacobiToErrorQuat() {
  Eigen::Matrix<double, 4, 3> quat_true_error = Eigen::Matrix<double, 4, 3>::Zero();
  quat_true_error.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity() * 0.5;
  quat_true_error = MathUtils::quatToLeftProductMatrix(nominal_attitude_) * quat_true_error;
  return quat_true_error;
}

Eigen::Matrix<double, 6, 1> ErrorStateKalmanFilter::computeHx() {
  Eigen::Matrix<double, 6, 1> hx;
  hx.block<3, 1>(0, 0) = nominal_pos_;
  hx.block<3, 1>(3, 0) = nominal_vel_;
  return hx;
}

Eigen::Vector3d ErrorStateKalmanFilter::llaToEnu(const Eigen::Vector3d& llaPosition) {
  if (!local_cartesian_initialized_) {
    local_cartesian_.Reset(llaPosition(0), llaPosition(1), llaPosition(2));
    local_cartesian_initialized_ = true;
  }
  Eigen::Vector3d enuPosition;
  local_cartesian_.Forward(llaPosition(0), llaPosition(1), llaPosition(2), enuPosition(0), enuPosition(1),
                           enuPosition(2));
  return enuPosition;
}

void ErrorStateKalmanFilter::injectErrorToNominal() {
  nominal_pos_ += error_x_.block<3, 1>(0, 0);
  nominal_vel_ += error_x_.block<3, 1>(3, 0);
  nominal_attitude_ = nominal_attitude_ * MathUtils::rotationVectorToQuaternion(error_x_.block<3, 1>(6, 0));
  nominal_attitude_.normalize();
  nominal_accel_bias_ += error_x_.block<3, 1>(9, 0);
  nominal_gyro_bias_ += error_x_.block<3, 1>(12, 0);
  nominal_gravity_ += error_x_.block<3, 1>(15, 0);
}

void ErrorStateKalmanFilter::resetErrorState() {
  G_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - 0.5 * MathUtils::vec3ToSkewSymmetric(error_x_.block<3, 1>(6, 0));
  P_ = G_ * P_ * G_.transpose();
  error_x_.setZero();
}
