#include "nanobot_imu/BNO085.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>

#define MAX_BUFFER_SIZE 512U

namespace nanobot_imu
{
  static int _file = -1;
  static std::chrono::steady_clock::time_point _start;

  static int i2c_hal_open(sh2_Hal_t *self);
  static void i2c_hal_close(sh2_Hal_t *self);
  static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
  static int i2c_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
  static uint32_t hal_getTimeUs(sh2_Hal_t *self);

  static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
  static void sensor_callback(void *cookie, sh2_SensorEvent_t *pEvent);

  BNO085::BNO085()
  {
  }

  BNO085::~BNO085()
  {
    finish_i2c();
  }

  bool BNO085::begin_i2c(std::string device, int address)
  {
    if (_file != -1)
    {
      std::cerr << "I2C device already open" << std::endl;
      return false;
    }

    _file = open(device.c_str(), O_RDWR);
    if (_file < 0)
    {
      std::cerr << "Failed to open I2C device" << std::endl;
      return false;
    }

    if (ioctl(_file, I2C_SLAVE, address) < 0)
    {
      std::cerr << "Failed to set I2C address" << std::endl;
      return false;
    }

    _start = std::chrono::steady_clock::now();

    _hal.open = i2c_hal_open;
    _hal.close = i2c_hal_close;
    _hal.read = i2c_hal_read;
    _hal.write = i2c_hal_write;
    _hal.getTimeUs = hal_getTimeUs;

    int status = sh2_open(&_hal, hal_callback, NULL);
    if (status != SH2_OK)
    {
      std::cerr << "Call to sh2_open failed" << std::endl;
      return false;
    }

    memset(&prod_ids, 0, sizeof(prod_ids));
    status = sh2_getProdIds(&prod_ids);
    if (status != SH2_OK)
    {
      std::cerr << "Call to sh2_getProdIds failed" << std::endl;
      return false;
    }

    sh2_setSensorCallback(sensor_callback, reinterpret_cast<void *>(this));

    return true;
  }

  void BNO085::finish_i2c()
  {
    if (_file == -1)
    {
      return;
    }

    close(_file);
    _file = -1;
  }

  void BNO085::spin_once()
  {
    sh2_service();
  }

  bool BNO085::enable_report(sh2_SensorId_t sensorId, uint32_t interval_us, ReportCallback callback)
  {
    static sh2_SensorConfig_t config;

    // These sensor options are disabled or not used in most cases
    config.alwaysOnEnabled = false;
    config.batchInterval_us = 0;
    config.changeSensitivity = 0;
    config.changeSensitivityEnabled = false;
    config.changeSensitivityRelative = false;
    config.reportInterval_us = interval_us;
    config.sensorSpecific = 0;
    config.wakeupEnabled = false;

    int status = sh2_setSensorConfig(sensorId, &config);

    if (status != SH2_OK)
    {
      return false;
    }

    report_callbacks[sensorId] = callback;

    return true;
  }

  bool BNO085::start_dynamic_calibration()
  {
    int status = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
    if (status != SH2_OK)
    {
      std::cerr << "Error setting calibration config" << std::endl;
      return false;
    }

    uint8_t sensors = 0;
    status = sh2_getCalConfig(&sensors);
    if (status != SH2_OK)
    {
      std::cerr << "Error getting calibration config" << std::endl;
      return false;
    }

    if (sensors & SH2_CAL_ACCEL)
    {
      std::cerr << "Accelerometer calibration enabled" << std::endl;
    }
    if (sensors & SH2_CAL_GYRO)
    {
      std::cerr << "Gyroscope calibration enabled" << std::endl;
    }
    if (sensors & SH2_CAL_MAG)
    {
      std::cerr << "Magnetometer calibration enabled" << std::endl;
    }
    if (sensors & SH2_CAL_PLANAR)
    {
      std::cerr << "Planar calibration enabled" << std::endl;
    }

    status = sh2_setDcdAutoSave(true);
    if (status != SH2_OK)
    {
      std::cerr << "Error configuring auto save" << std::endl;
      return false;
    }

    return true;
  }

  bool BNO085::tare()
  {
    int status = sh2_setTareNow(SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z, SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR);
    if (status != SH2_OK)
    {
      std::cerr << "Error when taring" << std::endl;
      return false;
    }

    status = sh2_persistTare();
    if (status != SH2_OK)
    {
      std::cerr << "Error trying to save tare values" << std::endl;
      return false;
    }

    return true;
  }

  // HAL functions

  static int i2c_hal_open(sh2_Hal_t * /* self */)
  {
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};

    for (uint8_t attempts = 0; attempts < 10; ++attempts)
    {
      if (write(_file, softreset_pkt, 5) == 5)
      {
        usleep(300000);
        return 0;
      }
      usleep(30000);
    }

    return -1;
  }

  void i2c_hal_close(sh2_Hal_t * /* self */)
  {
  }

  static int i2c_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
  {
    uint8_t header[4];
    if (read(_file, header, 4) != 4)
    {
      return 0;
    }

    // Determine amount to read
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    // Unset the "continue" bit
    packet_size &= ~0x8000;

    if (packet_size > len)
    {
      // Packet wouldn't fit in our buffer
      return 0;
    }

    // The number of non-header bytes to read
    uint16_t cargo_remaining = packet_size;
    uint8_t i2c_buffer[MAX_BUFFER_SIZE];
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;

    while (cargo_remaining > 0)
    {
      // Read the next chunk of data. If it isn't the first read, add 4 bytes for
      // the header
      read_size = std::min(MAX_BUFFER_SIZE, (unsigned)(cargo_remaining + (first_read ? 0 : 4)));

      if (read(_file, i2c_buffer, read_size) != read_size)
      {
        return 0;
      }

      if (first_read)
      {
        // The first time we're saving the "original" header, so include it in the
        // cargo count
        cargo_read_amount = read_size;
        memcpy(pBuffer, i2c_buffer, cargo_read_amount);
        first_read = false;
      }
      else
      {
        // This is not the first read, so copy from 4 bytes after the beginning of
        // the i2c buffer to skip the header included with every new i2c read and
        // don't include the header in the amount of cargo read
        cargo_read_amount = read_size - 4;
        memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
      }
      // Advance our pointer by the amount of cargo read
      pBuffer += cargo_read_amount;
      // Mark the cargo as received
      cargo_remaining -= cargo_read_amount;
    }

    *t_us = hal_getTimeUs(self);

    return packet_size;
  }

  int i2c_hal_write(sh2_Hal_t * /* self */, uint8_t *pBuffer, unsigned len)
  {
    uint16_t write_size = std::min(MAX_BUFFER_SIZE, len);
    if (write(_file, pBuffer, write_size) != write_size)
    {
      return 0;
    }

    return write_size;
  }

  uint32_t hal_getTimeUs(sh2_Hal_t * /* self */)
  {
    auto const now = std::chrono::steady_clock::now();
    auto const uptime = (now - _start);
    return std::chrono::duration_cast<std::chrono::microseconds>(uptime).count();
  }

  void hal_callback(void * /* cookie */, sh2_AsyncEvent_t *event)
  {
    auto const SHTPEventToErrStr = [](sh2_ShtpEvent_t const evt)
    {
      switch (evt)
      {
      case SH2_SHTP_TX_DISCARD:
        return std::string_view("SH2_SHTP_TX_DISCARD");
        break;
      case SH2_SHTP_SHORT_FRAGMENT:
        return std::string_view("SH2_SHTP_SHORT_FRAGMENT");
        break;
      case SH2_SHTP_TOO_LARGE_PAYLOADS:
        return std::string_view("SH2_SHTP_TOO_LARGE_PAYLOADS");
        break;
      case SH2_SHTP_BAD_RX_CHAN:
        return std::string_view("SH2_SHTP_BAD_RX_CHAN");
        break;
      case SH2_SHTP_BAD_TX_CHAN:
        return std::string_view("SH2_SHTP_BAD_TX_CHAN");
        break;
      default:
        __builtin_unreachable();
        break;
      }
    };

    if (event->eventId == SH2_RESET)
    {
      std::cout << "hal_callback(...) reset has occurred" << std::endl;
    }
    else if (event->eventId == SH2_SHTP_EVENT)
    {
      std::cerr << "hal_callback(...) SHTP error has occurred: \"" << SHTPEventToErrStr(event->shtpEvent) << "\"" << std::endl;
    }
    else if (event->eventId == SH2_GET_FEATURE_RESP)
    {
      std::cerr << "hal_callback(...) unhandled get feature response received" << std::endl;
    }
    else
    {
      std::ostringstream buf;
      buf << "hal_callback(...) unhandled hal event occurred, eventId = " << event->eventId;
      throw std::runtime_error(buf.str());
    }
  }

  void sensor_callback(void *cookie, sh2_SensorEvent_t *event)
  {
    sh2_SensorValue_t sensor_value;

    int status = sh2_decodeSensorEvent(&sensor_value, event);
    if (status != SH2_OK)
    {
      std::cerr << "sensor_callback(...) error decoding sensor event" << std::endl;
      return;
    }

    BNO085 *this_ptr = reinterpret_cast<BNO085 *>(cookie);
    this_ptr->report_callbacks[sensor_value.sensorId](sensor_value);
  }
} // namespace nanobot_imu