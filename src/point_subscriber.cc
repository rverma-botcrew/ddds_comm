#include <iostream>
#include <chrono>
#include <thread>

#include <ddscxx/dds/dds.hpp>
#include "pose.hpp"  // From generated/

int main() {
  dds::domain::DomainParticipant participant(0);
  dds::topic::Topic<pose_msgs::Pose> topic(participant, "PoseTopic");

  dds::sub::Subscriber subscriber(participant);
  dds::sub::DataReader<pose_msgs::Pose> reader(subscriber, topic);

  std::cout << "[Subscriber] Listening for poses on 'PoseTopic'...\n";

  while (true) {
    auto samples = reader.take();

    for (const auto &sample : samples) {
      if (sample.info().valid()) {
        const auto &pose = sample.data();

        const auto &pos = pose.position();
        const auto &orient = pose.orientation();

        std::cout << "Received DDS Pose:\n"
                  << "  Position: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n"
                  << "  Orientation: (" << orient.x() << ", " << orient.y() << ", "
                  << orient.z() << ", " << orient.w() << ")\n";
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;
}
