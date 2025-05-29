#include <memory>
#include <chrono>
#include <ddscxx/dds/dds.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dds_comm/mypoint.hpp>

#include "pose.hpp"

#include <geometry_msgs/msg/pose.hpp>

class PoseBridge : public rclcpp::Node {
public:
  PoseBridge()
  : Node("pose_bridge"),
    dds_participant_(0),
    dds_topic_(dds_participant_, "PoseTopic"),
    dds_publisher_(dds_participant_),
    dds_writer_(dds_publisher_, dds_topic_)
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "pose_topic", 10, std::bind(&PoseBridge::ros_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DDS Pose bridge initialized.");
  }

private:
  void ros_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    pose_msgs::Pose dds_pose;

    // Set position
    auto &pos = dds_pose.position();
    pos.x(msg->position.x);
    pos.y(msg->position.y);
    pos.z(msg->position.z);

    // Set orientation
    auto &orient = dds_pose.orientation();
    orient.x(msg->orientation.x);
    orient.y(msg->orientation.y);
    orient.z(msg->orientation.z);
    orient.w(msg->orientation.w);

    dds_writer_.write(dds_pose);

    RCLCPP_INFO(this->get_logger(), "Bridged Pose:\n"
                                    "  Position: (%.2f, %.2f, %.2f)\n"
                                    "  Orientation: (%.2f, %.2f, %.2f, %.2f)",
                                    pos.x(), pos.y(), pos.z(),
                                    orient.x(), orient.y(), orient.z(), orient.w());
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;

  dds::domain::DomainParticipant dds_participant_;
  dds::topic::Topic<pose_msgs::Pose> dds_topic_;
  dds::pub::Publisher dds_publisher_;
  dds::pub::DataWriter<pose_msgs::Pose> dds_writer_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<PoseBridge>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}