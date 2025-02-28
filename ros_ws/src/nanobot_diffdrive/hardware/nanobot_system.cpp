#include "nanobot_diffdrive/nanobot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nanobot_diffdrive
{
  hardware_interface::CallbackReturn NanobotDiffDriveHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.left_motor_id = std::stoi(info_.hardware_parameters["left_motor_id"]);
    cfg_.right_motor_id = std::stoi(info_.hardware_parameters["right_motor_id"]);
    cfg_.velocity_limit = std::stoi(info_.hardware_parameters["velocity_limit"]);
    cfg_.rpm_per_unit = std::stof(info_.hardware_parameters["rpm_per_unit"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.protocol_version = std::stof(info_.hardware_parameters["protocol_version"]);
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);

    left_wheel_.setup(cfg_.left_wheel_name, cfg_.rpm_per_unit);
    right_wheel_.setup(cfg_.right_wheel_name, cfg_.rpm_per_unit);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
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
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("NanobotDiffDriveHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> NanobotDiffDriveHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> NanobotDiffDriveHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));

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

    std::string error = comms_.setupMotors(cfg_.left_motor_id, cfg_.right_motor_id, cfg_.velocity_limit);

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
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    int left_wheel_value;
    int right_wheel_value;

    comms_.read(left_wheel_value, right_wheel_value);

    left_wheel_.vel = left_wheel_value * left_wheel_.rad_per_unit;
    right_wheel_.vel = -right_wheel_value * right_wheel_.rad_per_unit;

    // RCLCPP_INFO(
    //     rclcpp::get_logger("NanobotDiffDriveHardware"),
    //     "read vel: %f %f",
    //     left_wheel_.vel,
    //     right_wheel_.vel);

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type nanobot_diffdrive::NanobotDiffDriveHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    int left_motor_value = left_wheel_.cmd / left_wheel_.rad_per_unit;
    int right_motor_value = -right_wheel_.cmd / right_wheel_.rad_per_unit;

    comms_.write(left_motor_value, right_motor_value);

    // RCLCPP_INFO(
    //     rclcpp::get_logger("NanobotDiffDriveHardware"),
    //     "write vel: %f %f",
    //     left_wheel_.cmd,
    //     right_wheel_.cmd);

    return hardware_interface::return_type::OK;
  }

} // namespace nanobot_diffdrive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nanobot_diffdrive::NanobotDiffDriveHardware, hardware_interface::SystemInterface)
