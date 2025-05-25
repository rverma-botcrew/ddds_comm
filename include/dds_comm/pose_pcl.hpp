#pragma once

#include "posestamped.hpp"
#include "pointcloud2.hpp"

namespace dds_msgs
{

namespace msg
{

class PosePCL
{
public:
  PoseStamped pose;
  PointCloud2 pcl;

  PosePCL() = default;

  PosePCL(const PoseStamped& pose_, const PointCloud2& pcl_)
      : pose(pose_), pcl(pcl_) {}
};

} // namespace msg

} // namespace dds_msgs