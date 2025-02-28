#ifndef DIFFDRIVE_NANOBOT__DYNAMIXEL_COMMS_HPP
#define DIFFDRIVE_NANOBOT__DYNAMIXEL_COMMS_HPP

#include <string>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define ADDR_OPERATING_MODE 11

#define ADDR_VELOCITY_LIMIT 44
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128

class DynamixelComms
{

public:
  DynamixelComms() : dxl_comm_result_(COMM_TX_FAIL), dxl_error_(0) {}

  std::string connect(std::string device, float protocol_version, int baud_rate)
  {
    portHandler_ = dynamixel::PortHandler::getPortHandler(device.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

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

  std::string setupMotors(int left_motor_id, int right_motor_id, int velocity_limit)
  {
    left_motor_id_ = left_motor_id;
    right_motor_id_ = right_motor_id;
    velocity_limit_ = velocity_limit;

    // Use Velocity Control Mode
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
        portHandler_,
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
        portHandler_,
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
        portHandler_,
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

  std::string shutdownMotors()
  {
    // Disable Torque of DYNAMIXEL
    dxl_comm_result_ = packetHandler_->write1ByteTxRx(
        portHandler_,
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

  std::string write(int left_motor_value, int right_motor_value)
  {
    uint8_t dxl_error_ = 0;

    dxl_comm_result_ = packetHandler_->write4ByteTxRx(
        portHandler_,
        left_motor_id_,
        ADDR_GOAL_VELOCITY,
        left_motor_value,
        &dxl_error_);

    dxl_comm_result_ = packetHandler_->write4ByteTxRx(
        portHandler_,
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

  std::string read(int &left_motor_value, int &right_motor_value)
  {
    uint8_t dxl_error_ = 0;

    dxl_comm_result_ = packetHandler_->read4ByteTxRx(
        portHandler_,
        left_motor_id_,
        ADDR_PRESENT_VELOCITY,
        reinterpret_cast<uint32_t *>(&left_motor_value),
        &dxl_error_);

    dxl_comm_result_ = packetHandler_->read4ByteTxRx(
        portHandler_,
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

private:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  int dxl_comm_result_;
  uint8_t dxl_error_;
  int left_motor_id_;
  int right_motor_id_;
  int velocity_limit_;
};

#endif // DIFFDRIVE_NANOBOT__DYNAMIXEL_COMMS_HPP
