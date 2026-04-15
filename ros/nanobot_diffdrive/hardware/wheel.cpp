#include "nanobot_diffdrive/wheel.hpp"

Wheel::Wheel(const std::string &wheel_name, float rpm_per_unit, float deg_per_pulse)
{
  setup(wheel_name, rpm_per_unit, deg_per_pulse);
}

void Wheel::setup(const std::string &wheel_name, float rpm_per_unit, float deg_per_pulse)
{
  name = wheel_name;
  rad_per_unit = rpm_per_unit * (M_PI / 30); // M_PI / 30 = (2 * M_PI) / 60 = 1 rpm
  rad_per_pulse = deg_per_pulse * (M_PI / 180);
}