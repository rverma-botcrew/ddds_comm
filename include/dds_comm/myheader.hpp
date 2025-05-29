// header.hpp
#pragma once

#include <string>
#include "mytime.hpp"  

class MyHeader
{
public:
  MyTime stamp;
  std::string frame_id;

  MyHeader() = default;

  MyHeader(const MyTime stamp_, const std::string& frame_id_)
      : stamp(stamp_), frame_id(frame_id_) {}
};
