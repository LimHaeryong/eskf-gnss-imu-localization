#include "eskf_gnss_imu_localization/imu_subscriber.hpp"

ImuSubscriber::ImuSubscriber()
    : Node("ImuSubscriber"),
      mImuMeasurementQueue(std::make_shared<ThreadsafeQueue<std::shared_ptr<ImuMeasurement>>>(5)) {
  rclcpp::QoS qosProfile(10);
  mImuSubscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", qosProfile, std::bind(&ImuSubscriber::imuCallback, this, std::placeholders::_1));
}

void ImuSubscriber::imuCallback(sensor_msgs::msg::Imu::SharedPtr imuData) const {
  //RCLCPP_INFO(this->get_logger(), "sync imu message at %d.%08d", imuData->header.stamp.sec, imuData->header.stamp.nanosec);
  auto imuMeasurement = std::make_shared<ImuMeasurement>(std::move(imuData));
  mImuMeasurementQueue->push(std::move(imuMeasurement));
}