#pragma once

#include <cstdint>

class MyTime
{
public:
  int32_t sec;
  uint32_t nanosec;

  MyTime() = default;

  MyTime(int32_t sec, uint32_t nanosec) : sec(sec), nanosec(nanosec) {}
};
