#include <memory>
#include <chrono>
#include <ddscxx/dds/dds.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <dds_comm/mypoint.hpp>

#include "point.hpp"

class PointBridge : public rclcpp::Node {

private:
  void ros_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    pose_msgs::Point dds_point;
    dds_point.x(msg->x);
    dds_point.y(msg->y);
    dds_point.z(msg->z);

    dds_writer_->write(dds_point);

    // RCLCPP_INFO(this->get_logger(), "Received Point: x=%f, y=%f, z=%f", 
    //             msg->x, msg->y, msg->z);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_;

  // DDS components
  dds::domain::DomainParticipant dds_participant_;
  dds::topic::Topic<pose_msgs::Point> dds_topic_;
  dds::pub::Publisher dds_publisher_;
  dds::pub::DataWriter<pose_msgs::Point> dds_writer_;

public:
  PointBridge() : 
  Node("point_bridge"),
  dds_participant_(0),
  dds_topic_(dds_participant_, "PointTopic"),
  dds_publisher_(dds_participant_),
  dds_writer_(dds_publisher_, dds_topic_) {
    point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "point_topic", 10, std::bind(&PointBridge::ros_callback, this, std::placeholders::_1));

    // // Initialize DDS components
    // dds_participant_ = std::make_unique<dds::domain::DomainParticipant>(0);
    // dds_topic_ = std::make_unique<dds::topic::Topic<pose_msgs::Point>>(
    //   *dds_participant_.get(), "PointTopic");
    // dds_publisher_ = std::make_unique<dds::pub::Publisher>(*dds_participant_.get());
    // dds_writer_ = std::make_unique<dds::pub::DataWriter<pose_msgs::Point>>(
    //   *dds_publisher_.get(), *dds_topic_.get());

    RCLCPP_INFO(this->get_logger(), "PointBridge initialized and ready to receive messages.");
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<PointBridge>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}