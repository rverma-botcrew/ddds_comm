#pragma once

#include "myheader.hpp"
#include "mypose.hpp"


class MyPoseStamped
{
public:
  MyHeader header;
  MyPose pose;

  MyPoseStamped() = default;

  MyPoseStamped(const MyHeader& header_, const MyPose& pose_)
      : header(header_), pose(pose_) {}
};

