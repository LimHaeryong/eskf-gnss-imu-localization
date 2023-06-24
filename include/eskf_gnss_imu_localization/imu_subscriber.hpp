#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuSubscriber : public rclcpp::Node
{
public:
    ImuSubscriber();
    ~ImuSubscriber() {}

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuData) const;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mImuSubscriber;
};