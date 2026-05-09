#include "nanobot_diffdrive/nanobot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nanobot_diffdrive
{
  // Helper function to parse comma-separated parameters
  std::vector<std::string> split_string(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
      tokens.push_back(token);
    }
    return tokens;
  }

  hardware_interface::CallbackReturn NanobotDiffDriveHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.velocity_limit = std::stoi(info_.hardware_parameters["velocity_limit"]);
    cfg_.rpm_per_unit = std::stof(info_.hardware_parameters["rpm_per_unit"]);
    cfg_.deg_per_pulse = std::stof(info_.hardware_parameters["deg_per_pulse"]);

    cfg_.device = info_.hardware_parameters["device"];
    cfg_.protocol_version = std::stof(info_.hardware_parameters["protocol_version"]);
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);

    auto left_names = split_string(info_.hardware_parameters["left_wheel_names"], ',');
    auto right_names = split_string(info_.hardware_parameters["right_wheel_names"], ',');
    auto left_ids = split_string(info_.hardware_parameters["left_motor_ids"], ',');
    auto right_ids = split_string(info_.hardware_parameters["right_motor_ids"], ',');
    auto left_signs = split_string(info_.hardware_parameters["left_wheel_signs"], ',');
    auto right_signs = split_string(info_.hardware_parameters["right_wheel_signs"], ',');

    // Populate Left Wheels
    for (size_t i = 0; i < left_names.size(); i++) {
      ConfiguredWheel cw;
      cw.wheel.setup(left_names[i], cfg_.rpm_per_unit, cfg_.deg_per_pulse);
      cw.motor_id = std::stoi(left_ids[i]);
      cw.sign = std::stod(left_signs[i]);
      wheels_.push_back(cw);
    }

    // Populate Right Wheels
    for (size_t i = 0; i < right_names.size(); i++) {
      ConfiguredWheel cw;
      cw.wheel.setup(right_names[i], cfg_.rpm_per_unit, cfg_.deg_per_pulse);
      cw.motor_id = std::stoi(right_ids[i]);
      cw.sign = std::stod(right_signs[i]);
      wheels_.push_back(cw);
    }

    // Verify joints match URDF definitions
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // Check Command Interfaces
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' has %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Check State Interfaces
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      bool has_position = false;
      bool has_velocity = false;

      for (const auto& state_if : joint.state_interfaces) {
        if (state_if.name == hardware_interface::HW_IF_POSITION) has_position = true;
        if (state_if.name == hardware_interface::HW_IF_VELOCITY) has_velocity = true;
      }

      if (!has_position || !has_velocity)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' is missing either 'position' or 'velocity' state interface.", 
            joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> NanobotDiffDriveHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &cw : wheels_) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          cw.wheel.name, hardware_interface::HW_IF_POSITION, &cw.wheel.pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          cw.wheel.name, hardware_interface::HW_IF_VELOCITY, &cw.wheel.vel));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> NanobotDiffDriveHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto &cw : wheels_) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          cw.wheel.name, hardware_interface::HW_IF_VELOCITY, &cw.wheel.cmd));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn NanobotDiffDriveHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Configuring ...please wait...");

    std::string error = comms_.connect(cfg_.device, cfg_.protocol_version, cfg_.baud_rate);

    if (!error.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Configuration error: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn NanobotDiffDriveHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");

    comms_.disconnect();

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn NanobotDiffDriveHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Activating ...please wait...");

    std::string error = comms_.setupMotors(cfg_.velocity_limit);

    if (!error.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Activation error: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn NanobotDiffDriveHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Deactivating ...please wait...");

    std::string error = comms_.shutdownMotors();

    if (!error.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Deactivation error: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("NanobotDiffDriveHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type NanobotDiffDriveHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::vector<MotorState> motor_states;
    motor_states.reserve(wheels_.size()); 
    for (const auto &cw : wheels_) { 
      motor_states.push_back({cw.motor_id, 0, 0}); // Initialize with ID, zero vel, zero pos
    }

    std::string error = comms_.read(motor_states);
    
    if (!error.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("NanobotDiffDriveHardware"), "Read error: %s", error.c_str());
    }

    for (size_t i = 0; i < wheels_.size(); i++) {
      wheels_[i].wheel.vel = (motor_states[i].velocity * wheels_[i].sign) * wheels_[i].wheel.rad_per_unit;
      wheels_[i].wheel.pos = (motor_states[i].position * wheels_[i].sign) * wheels_[i].wheel.rad_per_pulse;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type NanobotDiffDriveHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    std::vector<MotorState> motor_states;
    motor_states.reserve(wheels_.size());

    for (const auto &cw : wheels_) {
      int motor_cmd = (cw.wheel.cmd * cw.sign) / cw.wheel.rad_per_unit;
      motor_states.push_back({cw.motor_id, motor_cmd, 0}); // ID, Command Velocity, Position (ignored for write)
    }

    std::string error = comms_.write(motor_states);

    if (!error.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("NanobotDiffDriveHardware"), "Write error: %s", error.c_str());
    }

    return hardware_interface::return_type::OK;
  }

} // namespace nanobot_diffdrive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nanobot_diffdrive::NanobotDiffDriveHardware, hardware_interface::SystemInterface)
