#ifndef _GNSS_SUBSCRIBER_HPP_
#define _GNSS_SUBSCRIBER_HPP_

#include <functional>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "eskf_gnss_imu_localization/threadsafe_queue.hpp"
#include "eskf_gnss_imu_localization/type.hpp"

class GnssSubscriber : public rclcpp::Node {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, geometry_msgs::msg::TwistStamped>;
  using QueueSharedPtr = std::shared_ptr<ThreadsafeQueue<std::shared_ptr<GnssMeasurement>>>;

  GnssSubscriber();
  ~GnssSubscriber() {}

  QueueSharedPtr getQueue() const { return mGnssMeasurementQueue; }

 private:
  void gnssSyncCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnssPosition,
                        const geometry_msgs::msg::TwistStamped::ConstSharedPtr& gnssVelocity) const;

  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> mGnssPositionSyncSubscriber;
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> mGnssVelocitySyncSubscriber;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> mGnssSynchronizer;

  QueueSharedPtr mGnssMeasurementQueue;
};

#endif  // _GNSS_SUBSCRIBER_HPP_