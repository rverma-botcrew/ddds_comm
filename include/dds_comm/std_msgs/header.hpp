// header.hpp
#pragma once

#include <string>
#include "time.hpp"  

namespace dds_msgs
{
namespace std_msgs
{

class Header
{
public:
  dds_msgs::time::Time stamp;
  std::string frame_id;

  Header() = default;

  Header(const dds_msgs::time::Time stamp_, const std::string& frame_id_)
      : stamp(stamp_), frame_id(frame_id_) {}
};

}  // namespace std_msgs
}  // namespace dds_msgs