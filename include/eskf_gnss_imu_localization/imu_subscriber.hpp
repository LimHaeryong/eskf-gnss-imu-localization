#ifndef _IMU_SUBSCRIBER_HPP_
#define _IMU_SUBSCRIBER_HPP_

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "eskf_gnss_imu_localization/threadsafe_queue.hpp"
#include "eskf_gnss_imu_localization/type.hpp"

class ImuSubscriber : public rclcpp::Node {
 public:
  using QueueSharedPtr = std::shared_ptr<ThreadsafeQueue<std::shared_ptr<ImuMeasurement>>>;

  ImuSubscriber();
  ~ImuSubscriber() {}

  QueueSharedPtr getQueue() const { return mImuMeasurementQueue; }

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuData) const;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mImuSubscriber;

  QueueSharedPtr mImuMeasurementQueue;
};

#endif  // _IMU_SUBSCRIBER_HPP_