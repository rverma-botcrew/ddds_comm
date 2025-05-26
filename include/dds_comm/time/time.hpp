#pragma once

#include <cstdint>

namespace dds_msgs
{

namespace time
{

class Time
{
public:
  int32_t sec;
  uint32_t nanosec;

  Time() = default;

  Time(int32_t sec, uint32_t nanosec) : sec(sec), nanosec(nanosec) {}
};
  
} // namespace time
  
} // namespace dds_msgs
