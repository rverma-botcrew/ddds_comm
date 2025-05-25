// header.hpp
#pragma once

#include <string>
#include "time.hpp"  

namespace dds_msgs
{
namespace msg
{

class Header
{
public:
  Time stamp;
  std::string frame_id;

  Header() = default;

  Header(const Time& stamp_, const std::string& frame_id_)
      : stamp(stamp_), frame_id(frame_id_) {}
};

}  // namespace msg
}  // namespace dds_msgs
