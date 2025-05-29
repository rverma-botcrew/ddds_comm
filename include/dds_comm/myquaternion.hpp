#pragma once

class MyQuaternion
{
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  MyQuaternion() = default;
  MyQuaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
};