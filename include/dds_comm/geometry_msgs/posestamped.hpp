#pragma once

#include "header.hpp"
#include "pose.hpp"

namespace dds_msgs
{
namespace geometry_msgs
{
class PoseStamped
{
public:
  dds_msgs::std_msgs::Header header;
  dds_msgs::geometry_msgs::Pose pose;

  PoseStamped() = default;

  PoseStamped(const dds_msgs::std_msgs::Header& header_, const dds_msgs::geometry_msgs::Pose& pose_)
      : header(header_), pose(pose_) {}
};

} // namespace msg
  
} // namespace dds_msgs
