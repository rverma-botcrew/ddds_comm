#include <iostream>
#include <ddscxx/dds/dds.hpp>
#include "posestamped.hpp"
#include <thread>


int main() {
  dds::domain::DomainParticipant participant(0);
  dds::topic::Topic<pose_msgs::PoseStamped> topic(participant, "PoseStampedTopic");
  dds::sub::Subscriber subscriber(participant);
  dds::sub::DataReader<pose_msgs::PoseStamped> reader(subscriber, topic);

  std::cout << "Listening on 'PoseStampedTopic'...\n";

  while (true) {
    auto samples = reader.take();
    for (const auto &sample : samples) {
      if (sample.info().valid()) {
        const auto &msg = sample.data();
        const auto &hdr = msg.header();
        const auto &pose = msg.pose();

        std::cout << "Received PoseStamped\n"
                  << "  Frame ID: " << hdr.frame_id() << "\n"
                  << "  Time: " << hdr.stamp().sec() << "." << hdr.stamp().nanosec() << "\n"
                  << "  Position: (" << pose.position().x() << ", " << pose.position().y() << ", " << pose.position().z() << ")\n"
                  << "  Orientation: (" << pose.orientation().x() << ", " << pose.orientation().y() << ", "
                  << pose.orientation().z() << ", " << pose.orientation().w() << ")\n";
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}
