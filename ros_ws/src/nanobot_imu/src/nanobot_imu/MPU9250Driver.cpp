#include "nanobot_imu/MPU9250Driver.hpp"

#include <unistd.h>
#include <jetgpio.h>
#include <stdio.h>
#include <limits>

namespace nanobot_imu
{
  MPU9250Driver::MPU9250Driver()
  {
  }

  MPU9250Driver::~MPU9250Driver()
  {
  }

  void MPU9250Driver::init(unsigned i2cBus)
  {
    mpu9250_ = i2cOpen(i2cBus, 0);

    /* Wake up the MPU-9250 with slave address 0x68 since it starts in sleep mode */
    i2cWriteByteData(mpu9250_, MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    usleep(100000);

    /* Set up the accelerator range to 4G */
    i2cWriteByteData(mpu9250_, MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_RANGE_4G);
    usleep(100000);

    /* Set up the gyroscope range to 250 deg/second */
    i2cWriteByteData(mpu9250_, MPU9250_ADDRESS, GYRO_CONFIG, GYRO_RANGE_250DEG);
    usleep(100000);

    // Enable the AK8963 magnetometer module
    i2cWriteByteData(mpu9250_, MPU9250_ADDRESS, I2C_BYPASS_CONFIG, 0x02);
    usleep(100000);

    // Set the magnetometer resolution to 16bit and 100Hz speed
    i2cWriteByteData(mpu9250_, AK8963_ADDRESS, MAG_CONFIG, MAG_16BIT_100HZ);
    usleep(100000);
  }

  void MPU9250Driver::close()
  {
    // Power down the AK8963.
    i2cWriteByteData(mpu9250_, AK8963_ADDRESS, MAG_CONFIG, 0x00);
    usleep(100000);

    // Disable the AK8963
    i2cWriteByteData(mpu9250_, MPU9250_ADDRESS, I2C_BYPASS_CONFIG, 0x00);
    usleep(100000);

    // Power down the MPU9250.
    i2cWriteByteData(mpu9250_, MPU9250_ADDRESS, PWR_MGMT_1, 0x40);
    usleep(100000);

    // Close the I2C port
    i2cClose(mpu9250_);

    gpioTerminate();
  }

  std::array<float, 3> MPU9250Driver::read_gyro()
  {
    const int gyro_registers[3] = {GYRO_X_H, GYRO_Y_H, GYRO_Z_H};
    std::array<float, 3> gyro_values;

    for (int i = 0; i < 3; ++i)
    {
      int high = i2cReadByteData(mpu9250_, MPU9250_ADDRESS, gyro_registers[i]);
      int low = i2cReadByteData(mpu9250_, MPU9250_ADDRESS, gyro_registers[i] + 1);

      int16_t gyro_raw = static_cast<int16_t>((high << 8) | (low & 0xFF));

      gyro_values[i] = gyro_raw / GYRO_SCALE_MODIFIER_250DEG;
    }

    usleep(10000);

    return gyro_values;
  }

  std::array<float, 3> MPU9250Driver::read_accel()
  {
    const int accel_registers[3] = {ACCEL_X_H, ACCEL_Y_H, ACCEL_Z_H};
    std::array<float, 3> accel_values;

    for (int i = 0; i < 3; ++i)
    {
      int high = i2cReadByteData(mpu9250_, MPU9250_ADDRESS, accel_registers[i]);
      int low = i2cReadByteData(mpu9250_, MPU9250_ADDRESS, accel_registers[i] + 1);

      int16_t accel_raw = static_cast<int16_t>((high << 8) | (low & 0xFF));

      accel_values[i] = accel_raw / ACCEL_SCALE_MODIFIER_4G;
    }

    usleep(10000);

    return accel_values;
  }

  std::array<float, 3> MPU9250Driver::read_mag()
  {
    const int mag_registers[3] = {MAG_X_L, MAG_Y_L, MAG_Z_L};
    std::array<float, 3> mag_values;

    for (int i = 0; i < 3; ++i)
    {
      int high = i2cReadByteData(mpu9250_, AK8963_ADDRESS, mag_registers[i] + 1);
      int low = i2cReadByteData(mpu9250_, AK8963_ADDRESS, mag_registers[i]);

      int16_t mag_raw = static_cast<int16_t>((high << 8) | (low & 0xFF));

      mag_values[i] = mag_raw * MAG_SCALE_MODIFIER;
    }

    usleep(10000);

    // Check if there was a magnetic overflow.
    int overflow = i2cReadByteData(mpu9250_, AK8963_ADDRESS, 0x09);
    if (overflow & 0x08)
    {
      mag_values[0] = std::numeric_limits<float>::quiet_NaN();
      mag_values[1] = std::numeric_limits<float>::quiet_NaN();
      mag_values[2] = std::numeric_limits<float>::quiet_NaN();
    }

    usleep(10000);

    return mag_values;
  }
} // namespace nanobot_imu