#pragma once

class MyPoint
{
public:
  double x;
  double y;
  double z;

  MyPoint() = default;
  MyPoint(double x, double y, double z) : x(x), y(y), z(z) {}
};
