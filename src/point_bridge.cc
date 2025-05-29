#include <memory>
#include <chrono>
#include <ddscxx/dds/dds.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "posestamped.hpp"  // from generated/

class PoseStampedBridge : public rclcpp::Node {
public:
  PoseStampedBridge()
  : Node("pose_stamped_bridge"),
    dds_participant_(0),
    dds_topic_(dds_participant_, "PoseStampedTopic"),
    dds_publisher_(dds_participant_),
    dds_writer_(dds_publisher_, dds_topic_)
  {
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose_stamped_topic", 10,
      std::bind(&PoseStampedBridge::ros_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PoseStamped DDS bridge initialized.");
  }

private:
  void ros_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_msgs::PoseStamped dds_msg;

    // Header
    auto &hdr = dds_msg.header();
    hdr.stamp().sec(msg->header.stamp.sec);
    hdr.stamp().nanosec(msg->header.stamp.nanosec);
    hdr.frame_id(msg->header.frame_id);

    // Pose
    auto &pose = dds_msg.pose();
    pose.position().x(msg->pose.position.x);
    pose.position().y(msg->pose.position.y);
    pose.position().z(msg->pose.position.z);

    pose.orientation().x(msg->pose.orientation.x);
    pose.orientation().y(msg->pose.orientation.y);
    pose.orientation().z(msg->pose.orientation.z);
    pose.orientation().w(msg->pose.orientation.w);

    dds_writer_.write(dds_msg);
    // RCLCPP_INFO(this->get_logger(), "Bridged PoseStamped for frame: %s", hdr.frame_id().c_str());
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  dds::domain::DomainParticipant dds_participant_;
  dds::topic::Topic<pose_msgs::PoseStamped> dds_topic_;
  dds::pub::Publisher dds_publisher_;
  dds::pub::DataWriter<pose_msgs::PoseStamped> dds_writer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<PoseStampedBridge>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}