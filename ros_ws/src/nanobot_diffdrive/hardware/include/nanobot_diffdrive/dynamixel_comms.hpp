#ifndef NANOBOT_DIFFDRIVE__DYNAMIXEL_COMMS_HPP_
#define NANOBOT_DIFFDRIVE__DYNAMIXEL_COMMS_HPP_

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <string>
#include <memory>

class DynamixelComms
{
public:
  DynamixelComms();

  std::string connect(std::string device, float protocol_version, int baud_rate);
  void disconnect();

  std::string setupMotors(int left_motor_id, int right_motor_id, int velocity_limit);
  std::string shutdownMotors();

  std::string write(int left_motor_value, int right_motor_value);
  std::string read(int &left_motor_value, int &right_motor_value);

private:
  static const int ADDR_OPERATING_MODE = 11;
  static const int ADDR_VELOCITY_LIMIT = 44;
  static const int ADDR_TORQUE_ENABLE = 64;
  static const int ADDR_GOAL_VELOCITY = 104;
  static const int ADDR_PRESENT_VELOCITY = 128;

  std::unique_ptr<dynamixel::PortHandler> portHandler_;
  std::unique_ptr<dynamixel::PacketHandler> packetHandler_;
  int dxl_comm_result_;
  uint8_t dxl_error_;
  int left_motor_id_;
  int right_motor_id_;
  int velocity_limit_;
};

#endif // NANOBOT_DIFFDRIVE__DYNAMIXEL_COMMS_HPP_
