#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <ddscxx/dds/dds.hpp>
#include "odom_pcl.hpp"  // from generated/

class OdomPCLBridge : public rclcpp::Node {
public:
  OdomPCLBridge()
  : Node("odom_pcl_bridge"),
    dds_participant_(0),
    dds_topic_(dds_participant_, "OdometryPointCloudTopic"),
    dds_publisher_(dds_participant_),
    dds_writer_(dds_publisher_, dds_topic_)
  {
    using namespace message_filters;

    odom_sub_.subscribe(this, "/odom");
    pcl_sub_.subscribe(this, "/velodyne_points");

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), odom_sub_, pcl_sub_);


    sync_->registerCallback(std::bind(
      &OdomPCLBridge::callback, this, std::placeholders::_1, std::placeholders::_2));

    status_pub_ = this->create_publisher<std_msgs::msg::String>("dds_bridge_status", 10);

    RCLCPP_INFO(this->get_logger(), "OdometryPointCloud DDS bridge initialized.");
  }

private:
  void callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_msg)
  {
    odom_msgs::OdometryPointCloud dds_msg;

    // === Fill Odometry ===
    auto &dds_odom = dds_msg.odom();

    auto &hdr = dds_odom.header();
    hdr.stamp().sec(odom_msg->header.stamp.sec);
    hdr.stamp().nanosec(odom_msg->header.stamp.nanosec);
    hdr.frame_id(odom_msg->header.frame_id);

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

    // === Fill PointCloud2 ===
    auto &pc_hdr = dds_msg.pcl().header();
    pc_hdr.stamp().sec(pcl_msg->header.stamp.sec);
    pc_hdr.stamp().nanosec(pcl_msg->header.stamp.nanosec);
    pc_hdr.frame_id(pcl_msg->header.frame_id);

    auto &pc = dds_msg.pcl();
    pc.height(pcl_msg->height);
    pc.width(pcl_msg->width);

    pc.fields().clear();
    for (const auto &field : pcl_msg->fields) {
      odom_msgs::PointField f;
      f.name(field.name);
      f.offset(field.offset);
      f.datatype(field.datatype);
      f.count(field.count);
      pc.fields().push_back(f);
    }

    pc.is_bigendian(pcl_msg->is_bigendian);
    pc.point_step(pcl_msg->point_step);
    pc.row_step(pcl_msg->row_step);

    pc.data().resize(pcl_msg->data.size());
    std::memcpy(pc.data().data(), pcl_msg->data.data(), pcl_msg->data.size());

    pc.is_dense(pcl_msg->is_dense);

    // Write to DDS
    dds_writer_.write(dds_msg);

    // Publish ROS log
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "Published OdometryPointCloud DDS message";
    status_pub_->publish(status_msg);
  }

  // ROS & DDS members
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub_;
  // std::shared_ptr<message_filters::TimeSynchronizer<
  //   nav_msgs::msg::Odometry,
  //   sensor_msgs::msg::PointCloud2>> sync_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  nav_msgs::msg::Odometry,
  sensor_msgs::msg::PointCloud2>;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;


  dds::domain::DomainParticipant dds_participant_;
  dds::topic::Topic<odom_msgs::OdometryPointCloud> dds_topic_;
  dds::pub::Publisher dds_publisher_;
  dds::pub::DataWriter<odom_msgs::OdometryPointCloud> dds_writer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPCLBridge>());
  rclcpp::shutdown();
  return 0;
}
