#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <sophus/so3.hpp>
#include <Eigen/Dense>
#include <sophus/geometry.hpp>

#include "eskf_gnss_imu_localization/gnss_subscriber.hpp"
#include "eskf_gnss_imu_localization/imu_subscriber.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto localizationMainNode = rclcpp::Node::make_shared("localization_main_node");
    auto gnssSubscriber = std::make_shared<GnssSubscriber>();
    auto imuSubscriber = std::make_shared<ImuSubscriber>();
    
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(gnssSubscriber);
    executor->add_node(imuSubscriber);

    std::thread executor_thread([&executor](){
        executor->spin();
    });
    
    while(rclcpp::ok())
    {
        std::cout << "main\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    executor->cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}

