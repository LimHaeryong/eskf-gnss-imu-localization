#ifndef _ESKF_HPP_
#define _ESKF_HPP_

#include <chrono>
#include <memory>

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>

#include "eskf_gnss_imu_localization/type.hpp"

class ErrorStateKalmanFilter {
 public:
  using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
  using Mat18d = Eigen::Matrix<double, 18, 18>;
  using Mat12d = Eigen::Matrix<double, 12, 12>;
  using Mat9d = Eigen::Matrix<double, 9, 9>;
  using Mat6d = Eigen::Matrix<double, 6, 6>;
  using Vec18d = Eigen::Matrix<double, 18, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;

  ErrorStateKalmanFilter();
  ~ErrorStateKalmanFilter() {}

  void predictWithImu(std::shared_ptr<ImuMeasurement> imuData);
  void updateWithGnss(std::shared_ptr<GnssMeasurement> gnssData);

  Eigen::Vector3d llaToEnu(const Eigen::Vector3d& llaPosition);

  void printState() const;
  Eigen::Vector3d getPosition() const { return nominal_pos_; }

 private:
  void injectErrorToNominal();
  void resetErrorState();

  Eigen::Matrix<double, 4, 3> computeQuatJacobiToErrorQuat();
  Eigen::Matrix<double, 6, 1> computeHx();

  TimePoint prev_time_;

  Eigen::Vector3d nominal_pos_;
  Eigen::Vector3d nominal_vel_;
  Eigen::Quaterniond nominal_attitude_;
  Eigen::Vector3d nominal_accel_bias_;
  Eigen::Vector3d nominal_gyro_bias_;
  Eigen::Vector3d nominal_gravity_;

  Vec18d error_x_;
  Mat18d P_;

  Mat18d F_x_;
  Eigen::Matrix<double, 18, 12> F_i_;
  Mat12d Q_i_;

  double velocity_noise_variance_;
  double orientation_noise_variance_;

  Vec6d z_;
  Eigen::Matrix<double, 6, 19> H_x_;
  Eigen::Matrix<double, 19, 18> J_true_error_;
  Eigen::Matrix<double, 6, 18> H_;
  Mat6d V_;
  Eigen::Matrix<double, 18, 6> K_;

  Mat18d G_;

  bool local_cartesian_initialized_ = false;
  GeographicLib::LocalCartesian local_cartesian_;
};

#endif  // _ESKF_HPP_