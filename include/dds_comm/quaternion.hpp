#pragma once

namespace dds_msgs
{

namespace msg
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

} // namespace msg

} // namespace dds_msgs
