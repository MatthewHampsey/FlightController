#pragma once

#include <vector>
#include <cstdint>
#include "vector3f.h"

#define SPI

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
  static const uint8_t FIFO_EN    = 0x23;
  static const uint8_t I2C_MST_CTRL    = 0x24;
  static const uint8_t I2C_SLV0_ADDR   = 0x25;
  static const uint8_t I2C_SLV0_REG    = 0x26;
  static const uint8_t I2C_SLV0_CTRL   = 0x27;
  static const uint8_t I2C_SLV1_ADDR   = 0x28;
  static const uint8_t I2C_SLV1_REG    = 0x29;
  static const uint8_t I2C_SLV1_CTRL   = 0x2A;

  static const uint8_t I2C_SLV4_ADDR   = 0x31;
  static const uint8_t I2C_SLV4_REG    = 0x32;
  static const uint8_t I2C_SLV4_DO     = 0x33;
  static const uint8_t I2C_SLV4_CTRL   = 0x34;
  static const uint8_t I2C_SLV4_DI     = 0x35;
  static const uint8_t I2C_MST_STATUS  = 0x36;

  static const uint8_t INT_PIN_CFG = 0x37;
  static const uint8_t INT_ENABLE = 0x38;
  static const uint8_t INT_STATUS = 0x3A;
  static const uint8_t EXT_SENS_DATA_00 = 0x49;  
  static const uint8_t EXT_SENS_DATA_01 = 0x4A;  
  static const uint8_t I2C_SLV0_DO = 0x63;
  static const uint8_t USER_CTRL = 0x6A;

  static const uint8_t PWR_MGMT_1 = 0x6B;
  static const uint8_t PWR_MGMT_2 = 0x6C;
  static const uint8_t I2C_MST_DELAY_CTRL = 0x67;

  static const uint8_t FIFO_COUNTH = 0x72;
  static const uint8_t FIFO_COUNTL = 0x73;
  static const uint8_t FIFO_R_W = 0x74;
}

namespace AK8963_REG
{
  static const uint8_t WIA   = 0x00;
  static const uint8_t INFO  = 0x01;
  static const uint8_t ST1   = 0x02;
  static const uint8_t HXL   = 0x03;
  static const uint8_t HXH   = 0x04;
  static const uint8_t HYL   = 0x05;
  static const uint8_t HYH   = 0x06;
  static const uint8_t HZL   = 0x07;
  static const uint8_t HZH   = 0x08;
  static const uint8_t ST2   = 0x09;
  static const uint8_t CNTL1 = 0x0A;
  static const uint8_t CNTL2 = 0x0B;
  static const uint8_t ASAX  = 0x10;
  static const uint8_t ASAY  = 0x11;
  static const uint8_t ASAZ  = 0x12;
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

  char readReg(const uint8_t addr);
  char readMagReg(const uint8_t addr);
  std::vector<char> readRegBytes(const char addr, size_t length);
  void readRegBytes(const uint8_t addr, size_t length, std::vector<char>& bytes);
  void writeReg(const uint8_t addr, const std::vector<uint8_t>& data);
  int16_t readHighLowReg(const char addr, char maskH, char maskL);
  int16_t readLowHighReg(const char addr, char maskL, char maskH);
#ifdef SPI
  void writeMagReg(const uint8_t addr, const std::vector<uint8_t>& data);
  char readMagStatusReg();
  void enableMagI2CSlave();
  void readMagMeasurementRegs(std::vector<char>& bytes);
  bool readMagSensorValues(Vector3f& mag);
  void updateGyroAccAndMag(bool& updated_gyro_acc, bool& updated_mag);
	

  void resetMag();
#endif
  Vector3f& getGyro();
  Vector3f& getAcc();
  Vector3f& getMag();
  Vector3f readCalibratedMag();
  void setMagBiases(const Vector3f soft_iron_bias, const Vector3f hard_iron_bias);
  void updateGyroAndAcc();
  void calibrate();
  void calibrateAK8963();
  void waitFor(const char addr, uint8_t mask, uint8_t value);
  void waitForMagMeasurement();
  private:
  int fd = -1;
  GyroScale _gyro_scale_bits;
  AccelerometerScale _accel_scale_bits;
  float _gyro_scale;
  float _accel_scale;
  Vector3f _gyro_bias;
  Vector3f _accel_bias;
  Vector3f _H_scale;
  Vector3f _prev_H;
  Vector3f _prev_gyro{0.0f, 0.0f, 0.0f};
  Vector3f _prev_acc{0.0f, 0.0f, 9.81};
  std::vector<char> _temp_buffer;
  Vector3f _hard_iron_bias;
  Vector3f _soft_iron_bias;
  Vector3f& readMag();
  Vector3f readMagBlocking();
  Vector3f& readMagSensorValues();
};
}
