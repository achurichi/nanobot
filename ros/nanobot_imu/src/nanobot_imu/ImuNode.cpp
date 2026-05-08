#include "nanobot_imu/ImuNode.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

namespace nanobot_imu
{
  using namespace std::chrono_literals;

  ImuNode::ImuNode()
      : Node("nanobot_imu"), _bno085(std::make_unique<BNO085>())
  {
    // Parameters
    declare_parameter<std::string>("device");
    declare_parameter<int>("address");
    declare_parameter<double>("read_freq");
    declare_parameter<double>("publish_freq");
    declare_parameter<std::string>("frame_id");
    declare_parameter<bool>("use_mag", false);
    declare_parameter<bool>("enable_dynamic_cal", true); 
    declare_parameter<bool>("debug", false);

    std::string device = get_parameter("device").as_string();
    int address = get_parameter("address").as_int();
    double read_freq = get_parameter("read_freq").as_double();
    double publish_freq = get_parameter("publish_freq").as_double();
    std::string frame_id = get_parameter("frame_id").as_string();
    bool enable_dynamic_cal = get_parameter("enable_dynamic_cal").as_bool();
    _use_mag = get_parameter("use_mag").as_bool();
    _debug = get_parameter("debug").as_bool();

    // Enable I2C communication
    if (!_bno085->begin_i2c(device, address))
    {
      RCLCPP_ERROR(get_logger(), "Failed to find BNO085 chip");
      return;
    }

    // Read the product IDs
    RCLCPP_INFO(get_logger(), "BNO085 found. Product details:");
    for (int n = 0; n < _bno085->prod_ids.numEntries; n++)
    {
      RCLCPP_INFO(get_logger(),
                  "Part: %d - Version: %d.%d.%d - Build %d",
                  _bno085->prod_ids.entry[n].swPartNumber,
                  _bno085->prod_ids.entry[n].swVersionMajor,
                  _bno085->prod_ids.entry[n].swVersionMinor,
                  _bno085->prod_ids.entry[n].swVersionPatch,
                  _bno085->prod_ids.entry[n].swBuildNumber);
    }

    // Conditionally start dynamic calibration
    if (enable_dynamic_cal)
    {
      _bno085->start_dynamic_calibration(_use_mag);
    }
    RCLCPP_INFO(get_logger(), "Dynamic calibration %s", enable_dynamic_cal ? "enabled" : "disabled");

    // Setup reports
    setup_reports(read_freq);

    // Define messages frame_id
    _imu_msg.header.frame_id = frame_id;
    _mag_msg.header.frame_id = frame_id;

    // Create publishers
    _imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    if (_use_mag)
    {
      _mag_publisher = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    }

    // Start timers
    auto publish_interval_ms = std::chrono::milliseconds(static_cast<int>(1000 / publish_freq));
    _publish_timer = create_wall_timer(publish_interval_ms, std::bind(&ImuNode::publish_data, this));

    auto read_interval_ms = std::chrono::milliseconds(static_cast<int>(1000 / read_freq));
    _read_timer = create_wall_timer(read_interval_ms, std::bind(&ImuNode::read_data, this));
  }

  ImuNode::~ImuNode()
  {
  }

  void ImuNode::setup_reports(double read_freq)
  {
    auto const handle_linear_acceleration = [this](const sh2_SensorValue_t &sensor_value)
    {
      this->_imu_msg.linear_acceleration.x = sensor_value.un.linearAcceleration.x;
      this->_imu_msg.linear_acceleration.y = sensor_value.un.linearAcceleration.y;
      this->_imu_msg.linear_acceleration.z = sensor_value.un.linearAcceleration.z;
      this->_imu_msg.header.stamp = this->get_clock()->now();
      if (this->_debug)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Accuracy lin acc: %d", sensor_value.status);
      }
    };

    auto const handle_gyroscope_calibrated = [this](const sh2_SensorValue_t &sensor_value)
    {
      this->_imu_msg.angular_velocity.x = sensor_value.un.gyroscope.x;
      this->_imu_msg.angular_velocity.y = sensor_value.un.gyroscope.y;
      this->_imu_msg.angular_velocity.z = sensor_value.un.gyroscope.z;
      this->_imu_msg.header.stamp = this->get_clock()->now();
      if (this->_debug)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Accuracy gyro: %d", sensor_value.status);
      }
    };

    auto const handle_magnetic_field_calibrated = [this](const sh2_SensorValue_t &sensor_value)
    {
      this->_mag_msg.magnetic_field.x = sensor_value.un.magneticField.x * 1e-6;
      this->_mag_msg.magnetic_field.y = sensor_value.un.magneticField.y * 1e-6;
      this->_mag_msg.magnetic_field.z = sensor_value.un.magneticField.z * 1e-6;
      this->_mag_msg.header.stamp = this->get_clock()->now();
      if (this->_debug)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Accuracy magnetometer: %d", sensor_value.status);
      }
    };

    auto const handle_ARVR_stabilized_RV = [this](const sh2_SensorValue_t &sensor_value)
    {
      this->_imu_msg.orientation.x = sensor_value.un.arvrStabilizedRV.i;
      this->_imu_msg.orientation.y = sensor_value.un.arvrStabilizedRV.j;
      this->_imu_msg.orientation.z = sensor_value.un.arvrStabilizedRV.k;
      this->_imu_msg.orientation.w = sensor_value.un.arvrStabilizedRV.real;
      this->_imu_msg.header.stamp = this->get_clock()->now();
      if (this->_debug)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Accuracy RV: %d", sensor_value.status);
      }
    };

    auto const handle_game_rotation_vector = [this](const sh2_SensorValue_t &sensor_value)
    {
      this->_imu_msg.orientation.x = sensor_value.un.gameRotationVector.i;
      this->_imu_msg.orientation.y = sensor_value.un.gameRotationVector.j;
      this->_imu_msg.orientation.z = sensor_value.un.gameRotationVector.k;
      this->_imu_msg.orientation.w = sensor_value.un.gameRotationVector.real;
      this->_imu_msg.header.stamp = this->get_clock()->now();
      if (this->_debug)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Accuracy Game RV: %d", sensor_value.status);
      }
    };

    // Base reports that are always enabled
    ReportCallbacksMap reports = {
      {SH2_LINEAR_ACCELERATION, handle_linear_acceleration},
      {SH2_GYROSCOPE_CALIBRATED, handle_gyroscope_calibrated}
    };

    // Conditionally add the correct orientation and magnetometer reports
    if (_use_mag) {
      reports[SH2_MAGNETIC_FIELD_CALIBRATED] = handle_magnetic_field_calibrated;
      reports[SH2_ARVR_STABILIZED_RV] = handle_ARVR_stabilized_RV;
    } else {
      reports[SH2_GAME_ROTATION_VECTOR] = handle_game_rotation_vector;
    }

    auto report_interval_us = static_cast<uint32_t>(1e6 / read_freq);

    for (const auto &[report, callback] : reports)
    {
      if (!_bno085->enable_report(report, report_interval_us, callback))
      {
        RCLCPP_ERROR(get_logger(), "Could not enable report with ID: %d", report);
      }
    }
  }

  void ImuNode::read_data()
  {
    _bno085->spin_once();
  }

  void ImuNode::publish_data()
  {
    _imu_publisher->publish(_imu_msg);
    if (_mag_publisher)
    {
      _mag_publisher->publish(_mag_msg);
    }
  }
} // namespace nanobot_imu
