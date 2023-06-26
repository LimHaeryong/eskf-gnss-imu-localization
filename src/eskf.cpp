#include "eskf_gnss_imu_localization/eskf.hpp"

void ErrorStateKalmanFilter::predictWithImu(std::shared_ptr<ImuMeasurement> imuMeasurement)
{
    std::chrono::duration<double> duration = imuMeasurement->timestamp - prevTime;
    prevTime = imuMeasurement->timestamp;
    double dt = duration.count();

    // compute F_x
    F_x.block<3, 3>(0, 3) = identity3d * dt;
    F_x.block<3, 3>(3, 15) = identity3d * dt;
    F_x.block<3, 3>(6, 12) = -1.0 * identity3d * dt;
}
