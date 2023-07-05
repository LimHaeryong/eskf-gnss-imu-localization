#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include "eskf_gnss_imu_localization/eskf.hpp"
#include "eskf_gnss_imu_localization/gnss_subscriber.hpp"
#include "eskf_gnss_imu_localization/imu_subscriber.hpp"

#include "opengl_viewer/opengl_viewer.h"

int main(int argc, char** argv) {
  SPDLOG_INFO("program start");
  rclcpp::init(argc, argv);
  
  auto gnssSubscriber = std::make_shared<GnssSubscriber>();
  auto gnssMeasurementQueue = gnssSubscriber->getQueue();
  auto imuSubscriber = std::make_shared<ImuSubscriber>();
  auto imuMeasurementQueue = imuSubscriber->getQueue();

  auto eskf = std::make_shared<ErrorStateKalmanFilter>();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(gnssSubscriber);
  executor->add_node(imuSubscriber);
  std::thread executorThread([&executor]() { executor->spin(); });

  auto openglViewer = std::make_shared<OpenglViewer>();
  auto glPointQueue = openglViewer->getQueue();
  std::thread openglViewerThread([&openglViewer](){
      openglViewer->run();
  });

  std::shared_ptr<ImuMeasurement> imuMeasurement = nullptr;
  std::shared_ptr<GnssMeasurement> gnssMeasurement = nullptr;

  SPDLOG_INFO("main loop start");

  while (rclcpp::ok()) {
    if (!imuMeasurementQueue->empty()) {
      imuMeasurement = imuMeasurementQueue->pop();
      eskf->predictWithImu(std::move(imuMeasurement));
      auto filteredPosition = eskf->getPosition();
      glPointQueue->push(std::make_shared<Point>(filteredPosition(0), filteredPosition(1), filteredPosition(2), PointType::FILTERED));
    }

    if (!gnssMeasurementQueue->empty()) {
      gnssMeasurement = gnssMeasurementQueue->pop();
      auto gnssPosition = eskf->llaToEnu(gnssMeasurement->position);
      eskf->updateWithGnss(std::move(gnssMeasurement));
      auto filteredPosition = eskf->getPosition();
      auto positionDiff = gnssPosition - filteredPosition;
      eskf->printState();
      SPDLOG_INFO("position diff : {:.3f}, {:.3f}, {:.3f}", positionDiff(0), positionDiff(1), positionDiff(2));
      glPointQueue->push(std::make_shared<Point>(gnssPosition(0), gnssPosition(1), gnssPosition(2), PointType::GNSS));
      glPointQueue->push(std::make_shared<Point>(filteredPosition(0), filteredPosition(1), filteredPosition(2), PointType::FILTERED));
    }
  }

  executor->cancel();
  executorThread.join();
  openglViewerThread.join();
  rclcpp::shutdown();

  return 0;
}
