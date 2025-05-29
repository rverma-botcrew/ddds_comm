#include <iostream>
#include <chrono>
#include <thread>

#include <ddscxx/dds/dds.hpp>
#include "point.hpp"  // Your generated DDS type

int main() {
  // 1. Create a DomainParticipant
  dds::domain::DomainParticipant participant(0);

  // 2. Create a Topic (must match the one used by the bridge)
  dds::topic::Topic<pose_msgs::Point> topic(participant, "PointTopic");

  // 3. Create a Subscriber and DataReader
  dds::sub::Subscriber subscriber(participant);
  dds::sub::DataReader<pose_msgs::Point> reader(subscriber, topic);

  std::cout << "[Subscriber] Listening for points on DDS topic 'PointTopic'...\n";

  // 4. Loop forever, polling for samples
  while (true) {
    auto samples = reader.take();

    for (const auto &sample : samples) {
      if (sample.info().valid()) {
        const auto &msg = sample.data();
        std::cout << "Received DDS Point: ("
                  << msg.x() << ", "
                  << msg.y() << ", "
                  << msg.z() << ")\n";
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;
}
