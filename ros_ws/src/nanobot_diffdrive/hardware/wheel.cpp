#include "nanobot_diffdrive/wheel.hpp"

Wheel::Wheel(const std::string &wheel_name, float rpm_per_unit)
{
  setup(wheel_name, rpm_per_unit);
}

void Wheel::setup(const std::string &wheel_name, float rpm_per_unit)
{
  name = wheel_name;
  rad_per_unit = rpm_per_unit * (M_PI / 30); // M_PI / 30 = (2 * M_PI) / 60 = 1 rpm
}