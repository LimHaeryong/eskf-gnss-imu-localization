#include "eskf_gnss_imu_localization/imu_subscriber.hpp"
#include "eskf_gnss_imu_localization/type.hpp"


ImuSubscriber::ImuSubscriber()
    : Node("ImuSubscriber")
{
    rclcpp::QoS qosProfile(10);
    mImuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", qosProfile, std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1));
}

void ImuSubscriber::imuCallback(sensor_msgs::msg::Imu::SharedPtr imuData) const
{
    RCLCPP_INFO(this->get_logger(), "sync imu message at %d.%08d", imuData->header.stamp.sec, imuData->header.stamp.nanosec);
    auto imuMeasurement = std::make_shared<ImuMeasurement>(std::move(imuData));
}