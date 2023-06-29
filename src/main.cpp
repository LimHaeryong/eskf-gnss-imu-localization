#include <chrono>
#include <memory>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include "eskf_gnss_imu_localization/gnss_subscriber.hpp"
#include "eskf_gnss_imu_localization/imu_subscriber.hpp"
#include "eskf_gnss_imu_localization/eskf.hpp"

#include "opengl_viewer/opengl_viewer.h"



int main(int argc, char** argv)
{
    SPDLOG_INFO("program start");
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

    SPDLOG_INFO("main loop start");
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
            auto gnssPosition = eskf->llaToEnu(gnssMeasurement->position);
            SPDLOG_INFO("gnss : {}, {}, {}", gnssPosition.x(), gnssPosition.y(), gnssPosition.z());
            eskf->updateWithGnss(std::move(gnssMeasurement));
            auto positionDiff = gnssPosition - eskf->getPosition();
            eskf->printState();
            SPDLOG_INFO("position diff gnss / filter : {}, {}, {}", positionDiff.x(), positionDiff.y(), positionDiff.z());
        }
    }

    executor->cancel();
    executorThread.join();
    //openglViewerThread.join();
    rclcpp::shutdown();

    return 0;
}

