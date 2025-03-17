#ifndef NANOBOT_DIFFDRIVE__WHEEL_HPP_
#define NANOBOT_DIFFDRIVE__WHEEL_HPP_

#include <string>
#include <cmath>

class Wheel
{
public:
  std::string name = "";
  double cmd = 0;
  double pos = 0;
  double vel = 0;
  double rad_per_unit = 0;
  double rad_per_pulse = 0;

  Wheel() = default;
  Wheel(const std::string &wheel_name, float rpm_per_unit, float deg_per_pulse);

  void setup(const std::string &wheel_name, float rpm_per_unit, float deg_per_pulse);
};

#endif // NANOBOT_DIFFDRIVE__WHEEL_HPP_
