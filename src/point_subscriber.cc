#include <iostream>
#include <thread>
#include <ddscxx/dds/dds.hpp>
#include "odom_pcl.hpp"  // Generated from OdometryPointCloud.idl

int main() {
  dds::domain::DomainParticipant participant(0);
  dds::topic::Topic<odom_msgs::OdometryPointCloud> topic(participant, "OdometryPointCloudTopic");
  dds::sub::Subscriber subscriber(participant);
  dds::sub::DataReader<odom_msgs::OdometryPointCloud> reader(subscriber, topic);

  std::cout << "📡 Listening on 'OdometryPointCloudTopic'...\n";

  while (true) {
    auto samples = reader.take();

    for (const auto &sample : samples) {
      if (sample.info().valid()) {
        const auto &msg = sample.data();

        // === Odometry ===
        const auto &odom = msg.odom();
        const auto &hdr = odom.header();
        const auto &pose = odom.pose().pose();
        const auto &twist = odom.twist().twist();

        std::cout << "\n📦 Received OdometryPointCloud:\n";
        std::cout << "  Frame ID: " << hdr.frame_id() << "\n";
        std::cout << "  Timestamp: " << hdr.stamp().sec() << "." << hdr.stamp().nanosec() << "\n";
        std::cout << "  Child Frame ID: " << odom.child_frame_id() << "\n";

        std::cout << "  Position: (" << pose.position().x() << ", "
                  << pose.position().y() << ", " << pose.position().z() << ")\n";

        std::cout << "  Orientation: (" << pose.orientation().x() << ", "
                  << pose.orientation().y() << ", " << pose.orientation().z() << ", "
                  << pose.orientation().w() << ")\n";

        std::cout << "  Linear Velocity: (" << twist.linear().x() << ", "
                  << twist.linear().y() << ", " << twist.linear().z() << ")\n";

        std::cout << "  Angular Velocity: (" << twist.angular().x() << ", "
                  << twist.angular().y() << ", " << twist.angular().z() << ")\n";

        // === PointCloud2 ===
        const auto &pc = msg.pcl();
        std::cout << "  PointCloud Info:\n";
        std::cout << "    Frame ID: " << pc.header().frame_id() << "\n";
        std::cout << "    Timestamp: " << pc.header().stamp().sec()
                  << "." << pc.header().stamp().nanosec() << "\n";
        std::cout << "    Dimensions: " << pc.width() << " x " << pc.height() << "\n";
        std::cout << "    Fields: " << pc.fields().size() << "\n";

        for (const auto &field : pc.fields()) {
          std::cout << "      Field: " << field.name()
                    << " (offset: " << field.offset()
                    << ", datatype: " << unsigned(field.datatype())
                    << ", count: " << field.count() << ")\n";
        }

        std::cout << "    is_dense: " << std::boolalpha << pc.is_dense() << "\n";
        std::cout << "    Data size: " << pc.data().size() << " bytes\n";
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
