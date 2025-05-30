#pragma once

#include "myposestamped.hpp"
#include "mypointcloud2.hpp"

class PosePCL
{
public:
  MyPoseStamped pose;
  MyPointCloud2 pcl;

  PosePCL() = default;

  PosePCL(const MyPoseStamped& pose_, const MyPointCloud2& pcl_)
      : pose(pose_), pcl(pcl_) {}
};
