#ifndef _EKSF_GNSS_IMU_LOCALIZATION_TYPE_HPP_
#define _EKSF_GNSS_IMU_LOCALIZATION_TYPE_HPP_

#include <chrono>
#include <iostream>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

struct ImuMeasurement
{
    using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
    TimePoint timestamp;
    Eigen::Vector3d angularVelocity;
    Eigen::Matrix3d angularVelocityCovariance;
    Eigen::Vector3d acceleration;
    Eigen::Matrix3d accelerationCovariance;

    ImuMeasurement(sensor_msgs::msg::Imu::SharedPtr imuData)
        : timestamp(std::chrono::seconds(imuData->header.stamp.sec) + std::chrono::nanoseconds(imuData->header.stamp.nanosec))
        , angularVelocity(Eigen::Vector3d(imuData->angular_velocity.x, imuData->angular_velocity.y, imuData->angular_velocity.z))
        , angularVelocityCovariance(Eigen::Matrix3d(imuData->angular_velocity_covariance.data()))
        , acceleration(Eigen::Vector3d(imuData->linear_acceleration.x, imuData->linear_acceleration.y, imuData->linear_acceleration.z))
        , accelerationCovariance(Eigen::Matrix3d(imuData->linear_acceleration_covariance.data()))
    {}
};

struct GnssMeasurement
{
    using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
    TimePoint timestamp;
    Eigen::Vector3d position; // latitude, longitude, altitude
    Eigen::Matrix3d positionCovariance;
    Eigen::Vector3d linearVelocity;

    GnssMeasurement(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnssPosition, const geometry_msgs::msg::TwistStamped::ConstSharedPtr& gnssVelocity)
        : timestamp(std::chrono::seconds(gnssPosition->header.stamp.sec) + std::chrono::nanoseconds(gnssPosition->header.stamp.nanosec))
        , position(Eigen::Vector3d(gnssPosition->latitude, gnssPosition->longitude, gnssPosition->altitude))
        , positionCovariance(Eigen::Matrix3d(gnssPosition->position_covariance.data()))
        , linearVelocity(Eigen::Vector3d(gnssVelocity->twist.linear.x, gnssVelocity->twist.linear.y, gnssVelocity->twist.linear.z))
    {}
};

#endif // _EKSF_GNSS_IMU_LOCALIZATION_TYPE_HPP_