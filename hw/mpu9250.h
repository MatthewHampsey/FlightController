#pragma once

#include <vector>
#include <cstdint>
#include "vector3f.h"

namespace MPU9250_REG
{

  static const uint8_t XG_OFFSET_H = 0x13;
  static const uint8_t XG_OFFSET_L = 0x14;
  static const uint8_t YG_OFFSET_H = 0x15;
  static const uint8_t YG_OFFSET_L = 0x16;
  static const uint8_t ZG_OFFSET_H = 0x17;
  static const uint8_t ZG_OFFSET_L = 0x18;

  static const uint8_t GYRO_CONFIG = 0x1B;
  static const uint8_t ACCEL_CONFIG = 0x1C;
  static const uint8_t ACCEL_CONFIG2 = 0x1D;
  static const uint8_t SMPLRT_DIV = 0x19;
  static const uint8_t CONFIG     = 0x1A;
  static const uint8_t I2C_MST_CTRL = 0x24;
  static const uint8_t FIFO_EN    = 0x23;
  static const uint8_t INT_PIN_CFG = 0x37;
  static const uint8_t INT_ENABLE = 0x38;
  static const uint8_t INT_STATUS = 0x3A;
  static const uint8_t USER_CTRL = 0x6A;

  static const uint8_t PWR_MGMT_1 = 0x6B;
  static const uint8_t PWR_MGMT_2 = 0X6C;

  static const uint8_t FIFO_COUNTH = 0x72;
  static const uint8_t FIFO_COUNTL = 0x73;
  static const uint8_t FIFO_R_W = 0x74;
}

namespace AK8963_REG
{
  static const uint8_t WIA = 0x00;
  static const uint8_t ST1 = 0x02;
  static const uint8_t HXL = 0x03;
  static const uint8_t HXH = 0x04;
  static const uint8_t HYL = 0x05;
  static const uint8_t HYH = 0x06;
  static const uint8_t HZL = 0x07;
  static const uint8_t HZH = 0x08;
  static const uint8_t ST2 = 0x09;
  static const uint8_t CNTL1 = 0x0A;
  static const uint8_t CNTL2 = 0x0B;
  static const uint8_t ASAX = 0x10;
  static const uint8_t ASAY = 0x11;
  static const uint8_t ASAZ = 0x12;
  
}

namespace FrameDrag{
class MPU9250
{
  public:
  enum class GyroScale
  {
    two_hundred_and_fifty = 0b00,
    five_hundred = 0b01,
    one_thousand = 0b10,
    two_thousand = 0b11
  };

  enum class AccelerometerScale
  {
    two_g,
    four_g,
    eight_g,
    sixteen_g
  };

  MPU9250(const char * dev_path,
	  GyroScale gyro_scale,
	  AccelerometerScale acc_scale);

  ~MPU9250();

  char readReg(const char addr);
  std::vector<char> readRegBytes(const char addr, size_t length);
  void writeReg(const char addr, const std::vector<uint8_t>& data);
  int16_t readHighLowReg(const char addr, char maskH, char maskL);
  int16_t readLowHighReg(const char addr, char maskL, char maskH);

  Vector3f readGyro();
  Vector3f readAcc();
  Vector3f readMag();

  void calibrate();
  void waitFor(const char addr, uint8_t mask, uint8_t value);
  private:
  int fd = -1;
  GyroScale _gyro_scale;
  AccelerometerScale _acc_scale;
  Vector3f _gyro_bias;
  Vector3f _accel_bias;
  Vector3f _H_scale;
  Vector3f _prev_H;
};
}
