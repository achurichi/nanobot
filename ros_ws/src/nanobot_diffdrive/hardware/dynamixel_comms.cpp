#include "nanobot_diffdrive/dynamixel_comms.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

DynamixelComms::DynamixelComms() : dxl_comm_result_(COMM_TX_FAIL), dxl_error_(0) {}

std::string DynamixelComms::connect(
    std::string device, float protocol_version, int baud_rate)
{
  portHandler_ = std::unique_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(device.c_str()));
  packetHandler_ = std::unique_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(protocol_version));

  // Open Serial Port
  dxl_comm_result_ = portHandler_->openPort();
  if (dxl_comm_result_ == false)
  {
    return "Failed to open the port!";
  }

  // Set the baudrate of the serial port
  dxl_comm_result_ = portHandler_->setBaudRate(baud_rate);
  if (dxl_comm_result_ == false)
  {
    return "Failed to set the baudrate!";
  }

  return "";
}

void DynamixelComms::disconnect()
{
  portHandler_->closePort();
}

std::string DynamixelComms::setupMotors(
    int left_motor_id, int right_motor_id, int velocity_limit)
{
  left_motor_id_ = left_motor_id;
  right_motor_id_ = right_motor_id;
  velocity_limit_ = velocity_limit;

  // Use Velocity Control Mode
  dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_.get(),
      BROADCAST_ID,
      ADDR_OPERATING_MODE,
      1,
      &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return "Failed to set Position Velocity Control Mode.";
  }

  // Set velocity limit
  dxl_comm_result_ = packetHandler_->write4ByteTxRx(
      portHandler_.get(),
      BROADCAST_ID,
      ADDR_VELOCITY_LIMIT,
      velocity_limit_,
      &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return "Failed to set velocity limit.";
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_.get(),
      BROADCAST_ID,
      ADDR_TORQUE_ENABLE,
      1,
      &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return "Failed to enable torque.";
  }

  return "";
}

std::string DynamixelComms::shutdownMotors()
{
  // Disable Torque of DYNAMIXEL
  dxl_comm_result_ = packetHandler_->write1ByteTxRx(
      portHandler_.get(),
      BROADCAST_ID,
      ADDR_TORQUE_ENABLE,
      0,
      &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return "Failed to disable torque.";
  }

  return "";
}

std::string DynamixelComms::write(int left_motor_value, int right_motor_value)
{
  uint8_t dxl_error_ = 0;

  dxl_comm_result_ = packetHandler_->write4ByteTxRx(
      portHandler_.get(),
      left_motor_id_,
      ADDR_GOAL_VELOCITY,
      left_motor_value,
      &dxl_error_);

  dxl_comm_result_ = packetHandler_->write4ByteTxRx(
      portHandler_.get(),
      right_motor_id_,
      ADDR_GOAL_VELOCITY,
      right_motor_value,
      &dxl_error_);

  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return packetHandler_->getTxRxResult(dxl_comm_result_);
  }
  else if (dxl_error_ != 0)
  {
    return packetHandler_->getRxPacketError(dxl_error_);
  }
  return "";
}

std::string DynamixelComms::read(int &left_motor_value, int &right_motor_value)
{
  uint8_t dxl_error_ = 0;

  dxl_comm_result_ = packetHandler_->read4ByteTxRx(
      portHandler_.get(),
      left_motor_id_,
      ADDR_PRESENT_VELOCITY,
      reinterpret_cast<uint32_t *>(&left_motor_value),
      &dxl_error_);

  dxl_comm_result_ = packetHandler_->read4ByteTxRx(
      portHandler_.get(),
      right_motor_id_,
      ADDR_PRESENT_VELOCITY,
      reinterpret_cast<uint32_t *>(&right_motor_value),
      &dxl_error_);

  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return packetHandler_->getTxRxResult(dxl_comm_result_);
  }
  else if (dxl_error_ != 0)
  {
    return packetHandler_->getRxPacketError(dxl_error_);
  }
  return "";
}
