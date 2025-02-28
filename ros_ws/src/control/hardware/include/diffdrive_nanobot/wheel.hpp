#ifndef DIFFDRIVE_NANOBOT__WHEEL_HPP
#define DIFFDRIVE_NANOBOT__WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
public:
  std::string name = "";
  int enc = 0;
  double cmd = 0;
  double pos = 0;
  double vel = 0;
  double rad_per_unit = 0;

  Wheel() = default;

  Wheel(const std::string &wheel_name, float rpm_per_unit)
  {
    setup(wheel_name, rpm_per_unit);
  }

  void setup(const std::string &wheel_name, float rpm_per_unit)
  {
    name = wheel_name;
    rad_per_unit = rpm_per_unit * (M_PI / 30); // M_PI / 30 = (2 * M_PI) / 60 = 1 rpm
  }
};

#endif // DIFFDRIVE_NANOBOT__WHEEL_HPP