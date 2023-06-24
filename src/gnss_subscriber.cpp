#include "eskf_gnss_imu_localization/gnss_subscriber.hpp"

GnssSubscriber::GnssSubscriber()
    : Node("GnssSubscriber")
{
    rclcpp::QoS qosProfile(10);
    auto rmwQosProfile = qosProfile.get_rmw_qos_profile();

    mGnssPositionSyncSubscriber.subscribe(this, "fix", rmwQosProfile);
    mGnssVelocitySyncSubscriber.subscribe(this, "vel", rmwQosProfile);

    mGnssSynchronizer = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), mGnssPositionSyncSubscriber, mGnssVelocitySyncSubscriber);
    mGnssSynchronizer->registerCallback(std::bind(&GnssSubscriber::gnssSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void GnssSubscriber::gnssSyncCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnssPosition, const geometry_msgs::msg::TwistStamped::ConstSharedPtr& gnssVelocity) const
{
    RCLCPP_INFO(this->get_logger(), "sync gnss position message at %d.%08d", gnssPosition->header.stamp.sec, gnssPosition->header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "sync gnss velocity message at %d.%08d", gnssVelocity->header.stamp.sec, gnssVelocity->header.stamp.nanosec);
}