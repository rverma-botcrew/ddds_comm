#pragma once

namespace dds_msgs
{

namespace geometry_msgs
{

class Quaternion
{
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  Quaternion() = default;
  Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
};

} // namespace geometry_msgs

} // namespace dds_msgs
