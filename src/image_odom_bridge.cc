#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <ddscxx/dds/dds.hpp>
#include "odom_pcl.hpp"

class ImageOdomBridge : public rclcpp::Node {
public:
  ImageOdomBridge()
  : Node("image_odom_bridge"),
    dds_participant_(0),
    dds_topic_(dds_participant_, "OdometryImage"),
    dds_publisher_(dds_participant_),
    dds_writer_(dds_publisher_, dds_topic_)
  {
    using namespace message_filters;

    // ROS 2 built-in QoS settings
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.depth = 10;

    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      this, "/rtabmap/odom", custom_qos_profile);

    image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, "/camera/camera/color/image_raw", custom_qos_profile);


    sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), *odom_sub_, *image_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.075));

    sync_->registerCallback(std::bind(&ImageOdomBridge::callback, this, std::placeholders::_1, std::placeholders::_2));

    status_pub_ = this->create_publisher<std_msgs::msg::String>("dds_bridge_status", 10);

    RCLCPP_INFO(this->get_logger(), "ImageOdomBridge initialized");
  }

private:
  void callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received synced messages");

    odom_msgs::OdometryImage dds_msg;
    auto &dds_odom = dds_msg.odom();

    // Header
    auto &header = dds_odom.header();
    header.stamp().sec(odom_msg->header.stamp.sec);
    header.stamp().nanosec(odom_msg->header.stamp.nanosec);
    header.frame_id(odom_msg->header.frame_id);
    dds_odom.child_frame_id(odom_msg->child_frame_id);

    // Pose
    auto &pose = dds_odom.pose().pose();
    pose.position().x(odom_msg->pose.pose.position.x);
    pose.position().y(odom_msg->pose.pose.position.y);
    pose.position().z(odom_msg->pose.pose.position.z);
    pose.orientation().x(odom_msg->pose.pose.orientation.x);
    pose.orientation().y(odom_msg->pose.pose.orientation.y);
    pose.orientation().z(odom_msg->pose.pose.orientation.z);
    pose.orientation().w(odom_msg->pose.pose.orientation.w);

    for (size_t i = 0; i < 36; ++i) {
      dds_odom.pose().covariance()[i] = odom_msg->pose.covariance[i];
    }

    // Twist
    auto &twist = dds_odom.twist().twist();
    twist.linear().x(odom_msg->twist.twist.linear.x);
    twist.linear().y(odom_msg->twist.twist.linear.y);
    twist.linear().z(odom_msg->twist.twist.linear.z);
    twist.angular().x(odom_msg->twist.twist.angular.x);
    twist.angular().y(odom_msg->twist.twist.angular.y);
    twist.angular().z(odom_msg->twist.twist.angular.z);

    for (size_t i = 0; i < 36; ++i) {
      dds_odom.twist().covariance()[i] = odom_msg->twist.covariance[i];
    }

    // Image
    auto &dds_image = dds_msg.image();
    dds_image.header().stamp().sec(image_msg->header.stamp.sec);
    dds_image.header().stamp().nanosec(image_msg->header.stamp.nanosec);
    dds_image.header().frame_id(image_msg->header.frame_id);
    dds_image.height(image_msg->height);
    dds_image.width(image_msg->width);
    dds_image.encoding(image_msg->encoding);
    dds_image.is_bigendian(image_msg->is_bigendian);
    dds_image.step(image_msg->step);
    dds_image.data().resize(image_msg->data.size());
    std::memcpy(dds_image.data().data(), image_msg->data.data(), image_msg->data.size());


    try {
      dds_writer_.write(dds_msg);
      RCLCPP_INFO(this->get_logger(), "DDS message written successfully");

      auto status_msg = std::make_shared<std_msgs::msg::String>();
      status_msg->data = "DDS write successful";
      status_pub_->publish(*status_msg);
    } catch (const dds::core::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "DDS write failed: %s", e.what());

    }
  }

  // Subscribers
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // DDS
  dds::domain::DomainParticipant dds_participant_;
  dds::topic::Topic<odom_msgs::OdometryImage> dds_topic_;
  dds::pub::Publisher dds_publisher_;
  dds::pub::DataWriter<odom_msgs::OdometryImage> dds_writer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageOdomBridge>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}