#include "nanobot_imu/ImuNode.hpp"

#include <chrono>
#include <array>
#include <cmath>

namespace nanobot_imu
{
  using namespace std::chrono_literals;

  ImuNode::ImuNode()
      : Node("nanobot_imu")
  {
    mpu9250_driver_.init(1);

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    timer_ = create_wall_timer(100ms, std::bind(&ImuNode::publish_data, this));
  }

  ImuNode::~ImuNode()
  {
    mpu9250_driver_.close();
  }

  void ImuNode::publish_data()
  {
    // Create the IMU and magnetic field messages
    sensor_msgs::msg::Imu imu_msg;
    sensor_msgs::msg::MagneticField mag_msg;

    // Read sensor data
    std::array<float, 3> gyro_data_dps = mpu9250_driver_.read_gyro();
    std::array<float, 3> accel_data_g = mpu9250_driver_.read_accel();
    std::array<float, 3> mag_data_uT = mpu9250_driver_.read_mag();

    auto stamp = this->get_clock()->now();

    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "imu_link";

    // Set angular velocity (gyro data)
    imu_msg.angular_velocity.x = gyro_data_dps[0] * (M_PI / 180.0);
    imu_msg.angular_velocity.y = gyro_data_dps[1] * (M_PI / 180.0);
    imu_msg.angular_velocity.z = gyro_data_dps[2] * (M_PI / 180.0);

    // Set linear acceleration (accelerometer data)
    imu_msg.linear_acceleration.x = accel_data_g[0] * 9.80665;
    imu_msg.linear_acceleration.y = accel_data_g[1] * 9.80665;
    imu_msg.linear_acceleration.z = accel_data_g[2] * 9.80665;

    mag_msg.header.stamp = stamp;
    mag_msg.header.frame_id = "imu_link";

    mag_msg.magnetic_field.x = mag_data_uT[0] * 1e-6;
    mag_msg.magnetic_field.y = mag_data_uT[1] * 1e-6;
    mag_msg.magnetic_field.z = mag_data_uT[2] * 1e-6;

    // Publish the IMU and magnetic field messages
    imu_publisher_->publish(imu_msg);
    mag_publisher_->publish(mag_msg);
  }

}
