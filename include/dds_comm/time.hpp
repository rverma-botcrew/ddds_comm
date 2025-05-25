#pragma once

#include <cstdint>

namespace dds_msgs
{

namespace msg
{

class Time
{
public:
  int32_t sec;
  uint32_t nanosec;

  Time() = default;

  Time(int32_t sec, uint32_t nanosec) : sec(sec), nanosec(nanosec) {}
};
  
} // namespace msg

  
} // namespace dds_msgs
