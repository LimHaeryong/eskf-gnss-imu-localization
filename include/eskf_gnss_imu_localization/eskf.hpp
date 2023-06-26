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

    ErrorStateKalmanFilter() {}
    ~ErrorStateKalmanFilter() {}

    void predictWithImu(std::shared_ptr<ImuMeasurement> imuMeasurement);
    void updateWithGnss(std::shared_ptr<GnssMeasurement> gnssMeasurement);

private:
    void injectErrorToNominal();
    void resetErrorState();

    TimePoint prevTime;

    Eigen::Matrix<double, 18, 1> x;
    Eigen::Matrix<double, 19, 1> error_x;
    Eigen::Matrix<double, 18, 18> P;

    Eigen::Matrix<double, 18, 18> F_x;
    Eigen::Matrix<double, 18, 12> F_i;
    Eigen::Matrix<double, 12, 12> Q_i;

    Eigen::Matrix<double, 6, 19> H_x;
    Eigen::Matrix<double, 19, 18> x_error_x;

    Eigen::Matrix<double, 6, 6> V; 
    Eigen::Matrix<double, 18, 6> K;

    Eigen::Matrix3d identity3d = Eigen::Matrix3d::Identity();
        
};

#endif // _ESKF_HPP_

// x(19 by 1) : nominal state || variable
// del_x(18 by 1) : error state || variable
// P(18 by 18) : state covariance || variable

// F_x(18 by 18), F_i(18 by 12) : jacobians of motion model repect to error, perturbation impulse || variable
// Q_i(12 by 12) : covariances matrix of the perturbation impulses(velocity, anglular, accel, angular vel) || variable + constant
// accel and angular -> use ImuMeasurement
// velocity and angle -> use Experimental value

// H_x(6 by 19), X_del_x(19 by 18), H(6 by 18) : observer jacobian || variable + constant
// K(18 by 6) : kalman gain || variable
// V(6 by 6) : measurement(gnss) noise || variable(use gnssMeasurement + experimental)