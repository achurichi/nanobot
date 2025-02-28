#ifndef DIFFDRIVE_NANOBOT__NANOBOT_SYSTEM_HPP
#define DIFFDRIVE_NANOBOT__NANOBOT_SYSTEM_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffdrive_nanobot/dynamixel_comms.hpp"
#include "diffdrive_nanobot/wheel.hpp"

namespace diffdrive_nanobot
{
  class DiffDriveNanobotHardware : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string left_wheel_name = "";
      std::string right_wheel_name = "";
      float wheel_radius = 0.0;
      float rpm_per_unit = 0.0;
      std::string device = "";
      float protocol_version = 0.0;
      int baud_rate = 0;
      int left_motor_id = 0;
      int right_motor_id = 0;
      int velocity_limit = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveNanobotHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    DynamixelComms comms_;
    Config cfg_;
    Wheel left_wheel_;
    Wheel right_wheel_;
  };

} // namespace diffdrive_nanobot

#endif // DIFFDRIVE_NANOBOT__NANOBOT_SYSTEM_HPP
