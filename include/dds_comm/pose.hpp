#pragma once

#include "point.hpp"
#include "quaternion.hpp"

namespace dds_msgs
{
namespace msg
{

class Pose
{
    Point position;
    Quaternion orientation;

    Pose() = default;

    Pose(const Point& pos, const Quaternion& orient)
        : position(pos), orientation(orient) {}
};

}  // namespace msg
}  // namespace dds_msgs
