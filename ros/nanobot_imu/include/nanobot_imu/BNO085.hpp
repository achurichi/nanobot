#ifndef NANOBOT_IMU__BNO085_HPP_
#define NANOBOT_IMU__BNO085_HPP_

#include <functional>
#include <string>
#include <unordered_map>

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

namespace nanobot_imu
{
  using ReportCallback = std::function<void(const sh2_SensorValue_t &)>;
  using ReportCallbacksMap = std::unordered_map<int, ReportCallback>;

  class BNO085
  {
  public:
    BNO085();
    ~BNO085();

    bool begin_i2c(std::string device, int address);
    void finish_i2c();
    void spin_once();
    bool enable_report(sh2_SensorId_t sensorId, uint32_t interval_us, ReportCallback callback);
    bool start_dynamic_calibration();
    bool tare();

    sh2_ProductIds_t prod_ids;
    ReportCallbacksMap report_callbacks;

  private:
    sh2_Hal_t _hal;
  };
} // namespace nanobot_imu

#endif // NANOBOT_IMU__BNO085_HPP_