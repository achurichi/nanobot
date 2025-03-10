#ifndef NANOBOT_IMU__IMU_NODE_HPP_
#define NANOBOT_IMU__IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <nanobot_imu/MPU9250Driver.hpp>

namespace nanobot_imu
{
  class ImuNode : public rclcpp::Node
  {
  public:
    ImuNode();
    ~ImuNode();

  private:
    void publish_data();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    MPU9250Driver mpu9250_driver_;
  };
} // namespace nanobot_imu

#endif // NANOBOT_IMU__IMU_NODE_HPP_