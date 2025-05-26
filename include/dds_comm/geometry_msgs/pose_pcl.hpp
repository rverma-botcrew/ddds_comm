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
  dds_msgs::geometry_msgs::PoseStamped pose;
  dds_msgs::sensor_msgs::PointCloud2 pcl;

  PosePCL() = default;

  PosePCL(const dds_msgs::geometry_msgs::PoseStamped& pose_, const dds_msgs::sensor_msgs::PointCloud2& pcl_)
      : pose(pose_), pcl(pcl_) {}
};

} // namespace msg

} // namespace dds_msgs