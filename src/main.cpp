#include <rclcpp/rclcpp.hpp>

#include "eskf_gnss_imu_localization/gnss_subscriber.hpp"
#include "eskf_gnss_imu_localization/imu_subscriber.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto gnssSubscriber = std::make_shared<GnssSubscriber>();
    auto imuSubscriber = std::make_shared<ImuSubscriber>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(gnssSubscriber);
    executor.add_node(imuSubscriber);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

