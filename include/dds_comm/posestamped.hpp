#pragma once

#include "header.hpp"
#include "pose.hpp"

namespace dds_msgs
{
namespace msg
{
class PoseStamped
{
public:
  Header header;
  Pose pose;

  PoseStamped() = default;

  PoseStamped(const Header& header_, const Pose& pose_)
      : header(header_), pose(pose_) {}
};

} // namespace msg
  
} // namespace dds_msgs
