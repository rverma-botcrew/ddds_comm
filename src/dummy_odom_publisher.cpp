#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class DummyOdomPublisher : public rclcpp::Node {
public:
  DummyOdomPublisher()
  : Node("dummy_odom_publisher") {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&DummyOdomPublisher::publish_odom, this));
    RCLCPP_INFO(this->get_logger(), "🚀 Dummy odometry publisher started");
  }

private:
  void publish_odom() {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    // Position
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 2.0;
    msg.pose.pose.position.z = 0.0;

    // Orientation (unit quaternion)
    msg.pose.pose.orientation.w = 1.0;

    // Twist
    msg.twist.twist.linear.x = 0.5;
    msg.twist.twist.angular.z = 0.1;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "📤 Published dummy odometry @ %u.%u",
      msg.header.stamp.sec, msg.header.stamp.nanosec);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
