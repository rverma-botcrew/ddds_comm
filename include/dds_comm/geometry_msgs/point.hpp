#pragma once

namespace dds_msgs
{

namespace geometry_msgs
{

class Point
{
public:
  double x;
  double y;
  double z;

  Point() = default;
  Point(double x, double y, double z) : x(x), y(y), z(z) {}
};

} // namespace geometry_msgs

} // namespace dds_geometry_msg
