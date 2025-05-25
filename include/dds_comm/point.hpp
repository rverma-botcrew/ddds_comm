#pragma once

namespace dds_msgs
{

namespace msg
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

} // namespace msg

} // namespace dds_geometry_msg
