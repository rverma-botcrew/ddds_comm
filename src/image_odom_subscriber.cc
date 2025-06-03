#include <iostream>
#include <thread>
#include <chrono>
#include <ddscxx/dds/dds.hpp>
#include "odom_pcl.hpp"  // Make sure this includes OdometryImage

int main() {
  // Create domain participant
  dds::domain::DomainParticipant participant(0);

  // Create topic for OdometryImage
  dds::topic::Topic<odom_msgs::OdometryImage> topic(participant, "OdometryImage");

  // Create subscriber
  dds::sub::Subscriber subscriber(participant);

  // Configure QoS: reliable, keep last 10 samples
  auto reader_qos = subscriber.default_datareader_qos()
    << dds::core::policy::Reliability::Reliable()
    << dds::core::policy::History::KeepLast(10)
    << dds::core::policy::ResourceLimits(100, 100, 100)
    << dds::core::policy::Durability::Volatile();

  // Create data reader with QoS
  dds::sub::DataReader<odom_msgs::OdometryImage> reader(subscriber, topic, reader_qos);

  std::cout << "📡 Listening on 'OdometryImage'...\n";

  auto last_time = std::chrono::steady_clock::now();
  double average_frequency = 0.0;
  int sample_count = 0;
  int throttle_count = 0;

  while (true) {
    try {
      dds::sub::LoanedSamples<odom_msgs::OdometryImage> samples = reader.take();

      for (const auto &sample : samples) {
        if (!sample.info().valid()) continue;

        const auto &msg = sample.data();

        // Frequency measurement
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last_time;
        last_time = now;

        double frequency = 1.0 / dt.count();
        average_frequency = (average_frequency * sample_count + frequency) / (sample_count + 1);
        sample_count++;

        std::cout << "\n📊 Average Frequency: " << average_frequency << " Hz\n";

        // === Odometry ===
        const auto &odom = msg.odom();
        const auto &hdr = odom.header();
        const auto &pose = odom.pose().pose();
        const auto &twist = odom.twist().twist();

        std::cout << "📦 Received OdometryImage:\n";
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

        // === Image Metadata (throttled to once every 10 messages) ===
        const auto &img = msg.image();
        const auto &img_hdr = img.header();

        if (++throttle_count % 10 == 0) {
          std::cout << "🖼️ Image Info:\n";
          std::cout << "  Frame ID: " << img_hdr.frame_id() << "\n";
          std::cout << "  Timestamp: " << img_hdr.stamp().sec()
                    << "." << img_hdr.stamp().nanosec() << "\n";
          std::cout << "  Dimensions: " << img.width() << " x " << img.height() << "\n";
          std::cout << "  Encoding: " << img.encoding() << "\n";
          std::cout << "  Step: " << img.step() << " bytes\n";
          std::cout << "  is_bigendian: " << std::boolalpha << static_cast<bool>(img.is_bigendian()) << "\n";
          std::cout << "  Data size: " << img.data().size() << " bytes\n";
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "❌ DDS Read Error: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Prevent CPU spin
  }

  return 0;
}
