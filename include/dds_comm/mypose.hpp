#pragma once

#include "mypoint.hpp"
#include "myquaternion.hpp"


class MyPose
{
    MyPoint position;
    MyQuaternion orientation;

    MyPose() = default;

    MyPose(const MyPoint& pos, const MyQuaternion& orient)
        : position(pos), orientation(orient) {}
};
