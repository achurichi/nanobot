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

std::string DynamixelComms::setupMotors(int velocity_limit)
{
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
      velocity_limit,
      &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return "Failed to set velocity limit.";
  }

  // Enable torque
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

  // Reset position
  dxl_comm_result_ = packetHandler_->clearMultiTurn(
      portHandler_.get(),
      BROADCAST_ID,
      &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    return "Failed to reset position.";
  }

  return "";
}

std::string DynamixelComms::shutdownMotors()
{
  // Disable torque
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

std::string DynamixelComms::write(const std::vector<MotorState>& motors)
{
  uint8_t dxl_error_ = 0;

  for (const auto& motor : motors)
  {
    dxl_comm_result_ = packetHandler_->write4ByteTxRx(
        portHandler_.get(),
        motor.id,
        ADDR_GOAL_VELOCITY,
        motor.velocity,
        &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS)
    {
      return packetHandler_->getTxRxResult(dxl_comm_result_);
    }
    else if (dxl_error_ != 0)
    {
      return packetHandler_->getRxPacketError(dxl_error_);
    }
  }

  return "";
}

std::string DynamixelComms::read(std::vector<MotorState>& motors)
{
  uint8_t dxl_error_ = 0;

  for (auto& motor : motors)
  {
    uint32_t current_vel = 0;
    uint32_t current_pos = 0;

    dxl_comm_result_ = packetHandler_->read4ByteTxRx(
        portHandler_.get(),
        motor.id,
        ADDR_PRESENT_VELOCITY,
        &current_vel,
        &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) return packetHandler_->getTxRxResult(dxl_comm_result_);
    if (dxl_error_ != 0) return packetHandler_->getRxPacketError(dxl_error_);

    dxl_comm_result_ = packetHandler_->read4ByteTxRx(
        portHandler_.get(),
        motor.id,
        ADDR_PRESENT_POSITION,
        &current_pos,
        &dxl_error_);

    if (dxl_comm_result_ != COMM_SUCCESS) return packetHandler_->getTxRxResult(dxl_comm_result_);
    if (dxl_error_ != 0) return packetHandler_->getRxPacketError(dxl_error_);

    motor.velocity = static_cast<int>(current_vel);
    motor.position = static_cast<int>(current_pos);
  }

  return "";
}
