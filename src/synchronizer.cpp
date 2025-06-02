#include <memory>
#include <deque>
#include <string>
#include <chrono>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

struct TimedOdom {
  rclcpp::Time Timestamp;
  nav_msgs::msg::Odometry::ConstSharedPtr OdomMsg;
};

class Sychronizer : public rclcpp::Node {
public:
  Sychronizer()
  : Node("synchronizer") {
    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      std::bind(&Sychronizer::odom_callback, this, std::placeholders::_1));

    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::QoS(10),
      std::bind(&Sychronizer::pcl_callback, this, std::placeholders::_1));

    // Publishers
    synced_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/synced_odom", 10);
    synced_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/synced_points", 10);
    debug_dt = this->create_publisher<std_msgs::msg::Float64>("/debug/dt", 10);
  }

private:
  TimedOdom odom_buffer;
  const rclcpp::Duration max_buffer_duration_ = rclcpp::Duration::from_seconds(0.2);  // Keep 200ms
  const rclcpp::Duration max_dt_ = rclcpp::Duration::from_seconds(0.025);  // Match threshold 25ms

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    rclcpp::Time now = this->get_clock()->now();
    odom_buffer = TimedOdom{msg->header.stamp, msg};

    // Clean old messages
    //while (!odom_buffer_.empty() && (now - odom_buffer_.front().Timestamp) > max_buffer_duration_) {
    //  odom_buffer.pop_front();
    //}
  }

  void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!odom_buffer.OdomMsg) {
      RCLCPP_WARN(this->get_logger(), "No odometry data available for synchronization.");
      return;
    }
    // TimedOdom* best_match = nullptr;

    // synced_odom_pub_->publish(*(best_match->OdomMsg));
    // synced_pcl_pub_->publish(*msg);

    last_odom_time = odom_buffer.Timestamp;
    last_pcl_time = msg->header.stamp;

    rclcpp::Time t_odom(last_odom_time);
    rclcpp::Time t_pcl(last_pcl_time);

    auto dt = std::abs((t_pcl - t_odom).seconds());

    std_msgs::msg::Float64 dt_msg;
    dt_msg.data = dt;
    debug_dt->publish(dt_msg);

    /*rclcpp::Time pcl_time = msg->header.stamp;
    const TimedOdom* best_match = nullptr;
    rclcpp::Duration best_dt = rclcpp::Duration::from_seconds(999);

    for (const auto& odom : odom_buffer_) {
      rclcpp::Duration dt = (odom.Timestamp > pcl_time) ? 
                             (odom.Timestamp - pcl_time) : 
                             (pcl_time - odom.Timestamp);
      if (dt < best_dt) {
        best_dt = dt;
        best_match = &odom;
      }
    }

    if (best_match && best_dt < max_dt_) {
      synced_odom_pub_->publish(*(best_match->OdomMsg));
      synced_pcl_pub_->publish(*msg);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No matching odometry found for the pointcloud at time %.4f", pcl_time.seconds());
    }*/


    synced_odom_pub->publish(*(odom_buffer.OdomMsg));
    synced_pcl_pub->publish(*msg);
    // Publish debug_dt message

    


  }


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr synced_odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr synced_pcl_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_dt;
  rclcpp::Time last_pcl_time;
  rclcpp::Time last_odom_time;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sychronizer>());
  rclcpp::shutdown();
  return 0;
}
