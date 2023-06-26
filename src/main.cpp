#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "eskf_gnss_imu_localization/gnss_subscriber.hpp"
#include "eskf_gnss_imu_localization/imu_subscriber.hpp"
#include "eskf_gnss_imu_localization/eskf.hpp"

#include "opengl_viewer/opengl_viewer.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto localizationMainNode = rclcpp::Node::make_shared("localization_main_node");
    auto gnssSubscriber = std::make_shared<GnssSubscriber>();
    auto gnssMeasurementQueue = gnssSubscriber->getQueue();
    auto imuSubscriber = std::make_shared<ImuSubscriber>();
    auto imuMeasurementQueue = imuSubscriber->getQueue();
    
    auto eskf = std::make_shared<ErrorStateKalmanFilter>();

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(gnssSubscriber);
    executor->add_node(imuSubscriber);
    std::thread executorThread([&executor](){
        executor->spin();
    });

    auto openglViewer = std::make_shared<OpenglViewer>();
    // std::thread openglViewerThread([&openglViewer](){
    //     openglViewer->run();
    // });

    std::shared_ptr<ImuMeasurement> imuMeasurement = nullptr;
    std::shared_ptr<GnssMeasurement> gnssMeasurement = nullptr;

    while(rclcpp::ok())
    {
        if(!imuMeasurementQueue->empty())
        {
            imuMeasurement = imuMeasurementQueue->pop();
            eskf->predictWithImu(std::move(imuMeasurement));
        }

        if(!gnssMeasurementQueue->empty())
        {
            gnssMeasurement = gnssMeasurementQueue->pop();
            std::cout << "gnss meas\n";
        }

        //std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    executor->cancel();
    executorThread.join();
    //openglViewerThread.join();
    rclcpp::shutdown();

    return 0;
}

