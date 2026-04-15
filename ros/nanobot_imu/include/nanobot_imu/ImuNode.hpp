#ifndef NANOBOT_IMU__IMU_NODE_HPP_
#define NANOBOT_IMU__IMU_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <nanobot_imu/BNO085.hpp>

namespace nanobot_imu
{
  class ImuNode : public rclcpp::Node
  {
  public:
    ImuNode();
    ~ImuNode();

  private:
    void setup_reports(double read_freq);

    void read_data();
    void publish_data();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr _mag_publisher;
    rclcpp::TimerBase::SharedPtr _publish_timer;
    rclcpp::TimerBase::SharedPtr _read_timer;

    std::unique_ptr<BNO085> _bno085;

    sensor_msgs::msg::Imu _imu_msg;
    sensor_msgs::msg::MagneticField _mag_msg;
  };
} // namespace nanobot_imu

#endif // NANOBOT_IMU__IMU_NODE_HPP_