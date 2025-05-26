#pragma once

#include "point.hpp"
#include "quaternion.hpp"

namespace dds_msgs
{
namespace geometry_msgs
{

class Pose
{
    dds_msgs::geometry_msgs::Point position;
    dds_msgs::geometry_msgs::Quaternion orientation;

    Pose() = default;

    Pose(const dds_msgs::geometry_msgs::Point& pos, const dds_msgs::geometry_msgs::Quaternion& orient)
        : position(pos), orientation(orient) {}
};

}  // namespace geometry_msgs
}  // namespace dds_msgs
