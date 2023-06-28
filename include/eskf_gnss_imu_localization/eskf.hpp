#ifndef _ESKF_HPP_
#define _ESKF_HPP_

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include "eskf_gnss_imu_localization/type.hpp"

class ErrorStateKalmanFilter
{
public:
    using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
    using Mat18d = Eigen::Matrix<double, 18, 18>;

    ErrorStateKalmanFilter();
    ~ErrorStateKalmanFilter() {}

    void predictWithImu(std::shared_ptr<ImuMeasurement> imuMeasurement);
    void updateWithGnss(std::shared_ptr<GnssMeasurement> gnssMeasurement);

    Eigen::Vector3d getPosition() const { return x.block<3, 1>(0, 0); }

private:
    void injectErrorToNominal();
    void resetErrorState();

    Eigen::Matrix<double, 4, 3> computeQ_error_theta(const Eigen::Matrix<double, 4, 1>& quaternion);
    Eigen::Matrix4d quaternionToLeftProductMatrix(const Eigen::Matrix<double, 4, 1>& quaternion);
    Eigen::Matrix4d quaternionToRightProductMatrix(const Eigen::Matrix<double, 4, 1>& quaternion);
    Eigen::Matrix3d vector3dToSkewSymmetric(const Eigen::Vector3d& vec3d);
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Matrix<double, 4, 1>& quaternion);

    TimePoint prevTime;

    Eigen::Matrix<double, 19, 1> x;
    Eigen::Matrix<double, 18, 1> error_x;
    Eigen::Matrix<double, 18, 18> P;

    Eigen::Matrix<double, 18, 18> F_x;
    Eigen::Matrix<double, 18, 12> F_i;
    Eigen::Matrix<double, 12, 12> Q_i;

    Eigen::Matrix<double, 6, 1> z;
    Eigen::Matrix<double, 6, 19> H_x;
    Eigen::Matrix<double, 19, 18> x_error_x;
    Eigen::Matrix<double, 6, 18> H;

    Eigen::Matrix<double, 6, 6> V; 
    Eigen::Matrix<double, 18, 6> K;

    Eigen::Matrix<double, 18, 18> G;
};

#endif // _ESKF_HPP_

// x(19 by 1) : nominal state : position(3), velocity(3), quaternion(4, Hamilton type = (w, x, y, z)), accel_bias(3), angular_velocity_bias(3), gravity(3)
// del_x(18 by 1) : error state  : position(3), velocity(3), theta(3, angle-axis), accel_bias(3), angular_velocity_bias(3), gravity(3)
// P(18 by 18) : state covariance 

// F_x(18 by 18), F_i(18 by 12) : jacobians of motion model repect to error, perturbation impulse 
// Q_i(12 by 12) : covariances matrix of the perturbation impulses(velocity, anglular, accel, angular vel) 
// velocity and angle covariance -> use Experimental value
// accel and angular velocity covariance-> use ImuMeasurement

// z(6 by 1) : measurement
// H_x(6 by 19), X_del_x(19 by 18), H(6 by 18) : observer jacobian 
// K(18 by 6) : kalman gain
// V(6 by 6) : measurement(gnss) noise covariance 
// position covariance -> use GnssMeasurement
// velocitiy covariance -> use Experimental value

// G(18 by 18) : for ESKF reset