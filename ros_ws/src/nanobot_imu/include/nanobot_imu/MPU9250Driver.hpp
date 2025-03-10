#ifndef NANOBOT_IMU__MPU9250_DRIVER_HPP_
#define NANOBOT_IMU__MPU9250_DRIVER_HPP_

#include <array>

namespace nanobot_imu
{
  class MPU9250Driver
  {
  public:
    MPU9250Driver();
    ~MPU9250Driver();

    void init(unsigned i2cBus);
    void close();
    std::array<float, 3> read_gyro();
    std::array<float, 3> read_accel();
    std::array<float, 3> read_mag();

  private:
    static const int MPU9250_ADDRESS = 0x68;
    static const int AK8963_ADDRESS = 0x0C;
    static const int PWR_MGMT_1 = 0x6B;
    static const int I2C_BYPASS_CONFIG = 0x37;

    static const int GYRO_CONFIG = 0x1B;
    static const int GYRO_RANGE_250DEG = 0x00;
    static const int GYRO_X_H = 0x43;
    static const int GYRO_Y_H = 0x45;
    static const int GYRO_Z_H = 0x47;
    static constexpr float GYRO_SCALE_MODIFIER_250DEG = 131.0;

    static const int ACCEL_CONFIG = 0x1C;
    static const int ACCEL_RANGE_4G = 0x08;
    static const int ACCEL_X_H = 0x3B;
    static const int ACCEL_Y_H = 0x3D;
    static const int ACCEL_Z_H = 0x3F;
    static constexpr float ACCEL_SCALE_MODIFIER_4G = 8192.0;

    static const int MAG_CONFIG = 0x0A;
    static const int MAG_16BIT_100HZ = 0x16;
    static const int MAG_X_L = 0x03;
    static const int MAG_Y_L = 0x05;
    static const int MAG_Z_L = 0x07;
    static constexpr float MAG_SCALE_MODIFIER = 0.15;

    int mpu9250_;
  };
} // namespace nanobot_imu

#endif // NANOBOT_IMU__MPU9250_DRIVER_HPP_