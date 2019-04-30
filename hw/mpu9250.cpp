#include "mpu9250.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <exception>
#include <limits>

namespace FrameDrag{

MPU9250::MPU9250(const char * dev_path,
		 GyroScale gyro_scale,
		 AccelerometerScale acc_scale)
	: _gyro_scale(gyro_scale)
	, _acc_scale(acc_scale)
{
  if((fd = open(dev_path, O_RDWR)) < 0)
  {
    std::string err = "Couldn't open " + std::string(dev_path);
    throw std::runtime_error(err);
  }

  if(ioctl(fd, I2C_SLAVE, 0x68) < 0)
  {       
    std::string err = "Couldn't set " + std::string(dev_path) + " bus address 0x68 to I2C_SLAVE";
    throw std::runtime_error(err);
  }

  char whoami = readReg(0x75);
  if(whoami != 0x71){
    throw std::runtime_error("whoami check failed");
  }
  //calibrate();
  //clear sleep bit, set to use 20 MHz internal oscillator
  //writeReg(MPU9250_REG::PWR_MGMT_1, {0x08});
  calibrate();
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x00});
  waitFor(MPU9250_REG::INT_STATUS, 0x01, 0x01);

  //set to use PLL clock
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x01});
  
  //set DLPF_CFG to 3: gyro BW is 41 Hz, gyro delay is 5.9ms, Fs is 1kHz
  //                   temp sensor BW is 42 Hz, delay is 4.8ms
  writeReg(MPU9250_REG::CONFIG, {0x03});
  
  // scale sample rate by 1/(1 + SMPLRT_DIV value) 
  writeReg(MPU9250_REG::SMPLRT_DIV, {0x04}); //divide sample rate by (4 + 1)


  uint8_t c = readReg(MPU9250_REG::GYRO_CONFIG);
  //clear fchoice_b [1:0]
  c = c & ~0x03;
  //clear scale selection bits [4:3]
  c = c & ~0x18;
  std::cout << "gyro scale: " << static_cast<int32_t>(_gyro_scale) << '\n';
  c = c | (static_cast<char>(_gyro_scale) << 3);
     // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeReg(MPU9250_REG::GYRO_CONFIG, {c});
  
  c = readReg(MPU9250_REG::ACCEL_CONFIG);
  //clear scale selection bits
  c = c & ~0x18;
  c = c | (static_cast<uint8_t>(_acc_scale) << 3);
  writeReg(MPU9250_REG::ACCEL_CONFIG, {c});
  
  c = readReg(MPU9250_REG::ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeReg(MPU9250_REG::ACCEL_CONFIG2, {c});

  //set bit 5 (LATCH_INT_EN) to true, interrupt pin status is held until interrupt status is cleared
  //also set bit 1 (BYPASS_EN) to true so we can access the other i2c devices (eg. the magnetometer) on the chip
  writeReg(MPU9250_REG::INT_PIN_CFG, {0b00100010});

  //enable RAW_RDY_EN, allows the raw sensor data interrupt value to propagate to the interrupt pin
  writeReg(MPU9250_REG::INT_ENABLE, {0x01}); 
  
  //configure magnetometer as slave
  if(ioctl(fd, I2C_SLAVE, 0x0C) < 0)
  {       
    std::string err = "Couldn't set " + std::string(dev_path) + " bus address 0x0C to I2C_SLAVE";
    throw std::runtime_error(err);
  }

  auto mag_whoami = readReg(AK8963_REG::WIA);
  std::cout << "mag whoami: " << (uint32_t)mag_whoami << '\n';
  if(mag_whoami != 0x48)
  {
    throw std::runtime_error("mag whoami check failed"); }

  char stat1 = readReg(AK8963_REG::ST1);
  std::cout << "data ready? " << (stat1 & 0x01) << '\n';

  //power down mag
  writeReg(AK8963_REG::CNTL1, {0x00});
  //enter Fuse ROM access mode
  writeReg(AK8963_REG::CNTL1, {0x0F});
  
  auto asax = readReg(AK8963_REG::ASAX);
  auto asay = readReg(AK8963_REG::ASAY);
  auto asaz = readReg(AK8963_REG::ASAZ);
  
  //16-bit twos complement measurement so measurements are scaled by 4912/32760 microTeslas (check datasheet)
  float scale_factor = 4912.0f/32760.0f;

  _H_scale[0] = scale_factor*(float)(asax-128)/256.0f + 1.0f;
  _H_scale[1] = scale_factor*(float)(asay-128)/256.0f + 1.0f;
  _H_scale[2] = scale_factor*(float)(asaz-128)/256.0f + 1.0f;
  
  //have to return to power down before switching to continuous read mode, check data sheet.
  writeReg(AK8963_REG::CNTL1, {0x00});

  //continuous mode 1 0010 8Hz measurement
  //continuous mode 2 0110 100Hz measurement
  //bit 4: 0 for 14-bit output, 1 for 16-bit output
  writeReg(AK8963_REG::CNTL1, {0b00010110}); //set to 16-bit output, mode 2
  calibrateAK8963();

  if(ioctl(fd, I2C_SLAVE, 0x68) < 0)
  {       
    std::string err = "Couldn't set " + std::string(dev_path) + " bus address 0x68 to I2C_SLAVE";
    throw std::runtime_error(err);
  }

}

MPU9250::~MPU9250()
{
  if(fd != -1)
  {
    close(fd);
  }
}

char MPU9250::readReg(const char addr){

  while(write(fd, &addr, 1) < 0){
    std::cerr << "failed to write" << '\n';
    //throw std::runtime_error("failed write");
  }
  char recv;
  if(read(fd, &recv, 1) < 0){
    std::cerr << "failed to read" << '\n';
    //throw std::runtime_error("failed read");
  }
  return recv;
}

std::vector<char> MPU9250::readRegBytes(const char addr, size_t length){

  while(write(fd, &addr, 1) < 0){
    std::cerr << "failed to write" << '\n';
    //throw std::runtime_error("failed write");
  }
  std::vector<char> recv(length, 0);
  if(read(fd, recv.data(), length) < 0){
    std::cerr << "failed to read" << '\n';
    //throw std::runtime_error("failed read");
  }
  return recv;
}

void MPU9250::writeReg(const char addr, const std::vector<uint8_t>& data)
{
  char full_data[data.size() + 1] = {addr};
  std::copy(data.begin(), data.end(), full_data + 1);
  //std::cout << "writing: " << '\n';
  for(unsigned int i = 0; i < data.size() + 1; i++){
    std::cout << std::hex << (uint32_t)full_data[i] << '\n';
  }  
  if(write(fd, full_data, data.size() + 1) < 0){
    std::cerr << "failed to write" << '\n';
    throw std::runtime_error("failed write");
  }
}

int16_t MPU9250::readHighLowReg(const char addr, char maskH, char maskL)
{
  char high = readReg(addr) & maskH;
  char low = readReg(addr + 1) & maskL;
   //std::cout << std::dec << "later high low raw: " << (int32_t)high << "  " << (int32_t)low << '\n'; 
  return (int16_t)(((int16_t)high << 8) | low);
}

int16_t MPU9250::readLowHighReg(const char addr, char maskL, char maskH)
{
  char low = readReg(addr) & maskL;
  char high = readReg(addr + 1) & maskH;
  return (int16_t)(((int16_t)high << 8) | low);
}

Vector3f MPU9250::readGyro()
{
  if(ioctl(fd, I2C_SLAVE, 0x68) < 0)
  {       
    std::string err = "Couldn't set i2c bus address 0x68 to I2C_SLAVE";
    throw std::runtime_error(err);
  }
  auto x = readHighLowReg(0x43, 0xFF, 0xFF);
  auto y = readHighLowReg(0x45, 0xFF, 0xFF);
  auto z = readHighLowReg(0x47, 0xFF, 0xFF);
  //std::cout << "raw gyro later vals: " << std::dec << x << "  " << y << "  " << z << '\n';
  float scale;
  switch(_gyro_scale)
  {
    case (GyroScale::two_hundred_and_fifty):
    scale = 250.0f;
    break;
    case(GyroScale::five_hundred):
    scale = 500.0f;
    break;
    case(GyroScale::one_thousand):
    scale = 1000.0f;
    break;
    default:
    scale = 2000.0f;
  }
  scale /= 32768.0f;
  auto vec = Vector3f{static_cast<float>(x)*scale, 
	  	      static_cast<float>(y)*scale, 
		      static_cast<float>(z)*scale};
  //std::cout << "gyro pre bias: " << std::dec << vec << '\n';
  vec -= _gyro_bias;
  //std::cout << "gyro: " << std::dec << vec << '\n';
  return vec;
}

Vector3f MPU9250::readAcc()
{
  if(ioctl(fd, I2C_SLAVE, 0x68) < 0)
  {       
    std::string err = "Couldn't set i2c bus address 0x68 to I2C_SLAVE";
    throw std::runtime_error(err);
  }
  auto x = readHighLowReg(0x3B, 0xFF, 0xFF);
  auto y = readHighLowReg(0x3D, 0xFF, 0xFF);
  auto z = readHighLowReg(0x3F, 0xFF, 0xFF);
  float scale;
  switch(_acc_scale)
  {
    case (AccelerometerScale::two_g):
    scale = 2.0f;
    break;
    case(AccelerometerScale::four_g):
    scale = 4.0f;
    break;
    case(AccelerometerScale::eight_g):
    scale = 8.0f;
    break;
    default:
    scale = 16.0f;
  }
  scale /= 32768.0f;
  auto vec = Vector3f{(float)x*scale, (float)y*scale, (float)z*scale};
  vec -= _accel_bias;
 // std::cout << "accel: " << vec << '\n';
  return vec;
}

Vector3f MPU9250::readMagSensorValues()
{
  auto x = readLowHighReg(AK8963_REG::HXL, 0xFF, 0xFF);
  auto y = readLowHighReg(AK8963_REG::HYL, 0xFF, 0xFF);
  auto z = readLowHighReg(AK8963_REG::HZL, 0xFF, 0xFF);
  
  char overflow = readReg(AK8963_REG::ST2);
  if((overflow & 0b00001000) != 0x00)
  {
    std::cout << "overflowed!" << '\n';
    return _prev_H;
  }
  auto vec = Vector3f{(float)x*_H_scale[0], (float)y*_H_scale[1], (float)z*_H_scale[0]};
  _prev_H = vec;
  return vec;
}

Vector3f MPU9250::readCalibratedMag()
{
  auto mag = readMag() - _hard_iron_bias;
  return Vector3f{mag[0]*_soft_iron_bias[0],
                  mag[1]*_soft_iron_bias[1],
                  mag[2]*_soft_iron_bias[2]};
}

Vector3f MPU9250::readMag()
{
  if(ioctl(fd, I2C_SLAVE, 0x0C) < 0)
  {       
    std::string err = "Couldn't set i2c bus address 0x0C to I2C_SLAVE";
    throw std::runtime_error(err);
  }
  
  if((readReg(AK8963_REG::ST1) & 0x01) != 0x01)
  {
    return _prev_H;
  }
  return readMagSensorValues();
}

Vector3f MPU9250::readMagBlocking()
{
  if(ioctl(fd, I2C_SLAVE, 0x0C) < 0)
  {       
    std::string err = "Couldn't set i2c bus address 0x0C to I2C_SLAVE";
    throw std::runtime_error(err);
  }
  
  while((readReg(AK8963_REG::ST1) & 0x01) != 0x01){}

  return readMagSensorValues();
}

void MPU9250::calibrate()
{

  //reset IMU by setting bit 7
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x80});  
  waitFor(MPU9250_REG::INT_STATUS, 0x01, 0x01);

  // select PLL clock
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x01});  

  //enable gyro and accel
  writeReg(MPU9250_REG::PWR_MGMT_2, {0x00}); 
  waitFor(MPU9250_REG::INT_STATUS, 0x01, 0x01);

  writeReg(MPU9250_REG::INT_ENABLE, {0x00});   // Disable all interrupts
  writeReg(MPU9250_REG::FIFO_EN, {0x00});      // Disable FIFO
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x00});   // Turn on internal clock source
  writeReg(MPU9250_REG::I2C_MST_CTRL, {0x00}); // Disable I2C master
  writeReg(MPU9250_REG::USER_CTRL, {0x00});    // Disable FIFO and I2C master modes
  writeReg(MPU9250_REG::USER_CTRL, {0x0C});    // Reset FIFO and DMP
  waitFor(MPU9250_REG::INT_STATUS, 0x01, 0x01);
	       
  // Configure MPU9250 gyro and accelerometer for bias calculation
  //set no FIFO rewrite. Set DLPF_CONFIG to 1 -> 184Hz gyro BW
  writeReg(MPU9250_REG::CONFIG, {0x41});      // Set low-pass filter to 188 Hz
  writeReg(MPU9250_REG::SMPLRT_DIV, {0x00});  // Set sample rate to 1 kHz

  //clear fchoice on gyro	
  writeReg(MPU9250_REG::GYRO_CONFIG, {0x00});  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeReg(MPU9250_REG::ACCEL_CONFIG, {0x00}); // Set accelerometer full-scale to 2 g, maximum sensitivity

  int32_t gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  int32_t accelsensitivity = 16384; // = 16384 LSB/g

  uint16_t fifo_count = readHighLowReg(MPU9250_REG::FIFO_COUNTH, 0b0001111, 0xFF);
  std::cout << "fifo count begin: " << std::dec << fifo_count << '\n';

  writeReg(MPU9250_REG::USER_CTRL, {0b01000000});   // Enable FIFO  
  writeReg(MPU9250_REG::FIFO_EN, {0b01111000});     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  usleep(40000); // accumulate 40 samples in 80 milliseconds = 480 bytes

  fifo_count = readHighLowReg(MPU9250_REG::FIFO_COUNTH, 0b0001111, 0xFF);
  std::cout << "fifo count end: " << std::dec << fifo_count << '\n';
  int16_t packet_count = fifo_count/12;
  std::vector<int16_t> gyro_meas{0, 0, 0};
  std::vector<int16_t> accel_meas{0, 0, 0};
  //std::vector<int32_t> gyro_bias{0, 0, 0};
  //std::vector<int32_t> accel_bias{0, 0, 0};
  for (uint16_t i = 0; i < packet_count; i++) {
	  
    auto data = readRegBytes(MPU9250_REG::FIFO_R_W, 12); // read data for averaging
    accel_meas[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_meas[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_meas[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_meas[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_meas[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_meas[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    //std::cout << std::dec << "high low raw: " << (int32_t)data[6] << "  " << (int32_t)data[7] << '\n'; 
    //std::cout << std::dec << "raw gyro vals: " << gyro_meas[0] << "  " << gyro_meas[1] << "  " << gyro_meas[2] << '\n';					      
//  std::cout << "gyro messes: " << std::dec << (float)gyro_meas[0]/(float)gyrosensitivity << "   " << (float)gyro_meas[1]/(float)gyrosensitivity << "   " << (float)gyro_meas[2]/(float)gyrosensitivity << '\n'; 
//  std::cout << "accel messes: " << std::dec << (float)accel_meas[0]/(float)gyrosensitivity << "   " << (float)accel_meas[1]/(float)gyrosensitivity << "   " << (float)accel_meas[2]/(float)gyrosensitivity << '\n'; 
    _accel_bias[0] += accel_meas[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    _accel_bias[1] += accel_meas[1];
    _accel_bias[2] += accel_meas[2];
    _gyro_bias[0]  += gyro_meas[0];
    _gyro_bias[1]  += gyro_meas[1];
    _gyro_bias[2]  += gyro_meas[2];								              
  }
  float accel_scale = packet_count*accelsensitivity;
  float gyro_scale = packet_count*gyrosensitivity;
  _accel_bias[0] /= accel_scale; // Normalize sums to get average count biases
  _accel_bias[1] /= accel_scale;
  _accel_bias[2] /= accel_scale;
  _gyro_bias[0]  /= gyro_scale;
  _gyro_bias[1]  /= gyro_scale;
  _gyro_bias[2]  /= gyro_scale;

  //std::cout << std::dec << "bs: " << gyro_bias[0] << "  " << gyro_bias[1] << "  " << gyro_bias[2] << '\n';					      

  std::cout << "gyro biases: " << std::dec << _gyro_bias[0] << "   " << _gyro_bias[1] << "   " << _gyro_bias[2] << '\n'; 
  std::cout << "acc biases: " << std::dec << _accel_bias[0] << "   " << _accel_bias[1] << "   " << _accel_bias[2] << '\n'; 
    
  if(_accel_bias[2] > 0L) {_accel_bias[2] += 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
  else {_accel_bias[2] -= 1.0f;}
  //std::vector<char> data(6);  
  //data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  //data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  //data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  //data[3] = (-gyro_bias[1]/4)       & 0xFF;
  //data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  //data[5] = (-gyro_bias[2]/4) & 0xFF;

  //writeReg(MPU9250_REG::XG_OFFSET_H, {data[0]});
  //writeReg(MPU9250_REG::XG_OFFSET_L, {data[1]});
  //writeReg(MPU9250_REG::XG_OFFSET_H, {data[2]});
  //writeReg(MPU9250_REG::YG_OFFSET_L, {data[3]});
  //writeReg(MPU9250_REG::ZG_OFFSET_H, {data[4]});
  //writeReg(MPU9250_REG::ZG_OFFSET_L, {data[5]});
}

void MPU9250::calibrateAK8963()
{
  std::cout << "beginning mag calibration" << '\n';
  //std::vector<Vector3f> mag_values(1000);
  Vector3f max_mag = {-std::numeric_limits<float>::max(),
	              -std::numeric_limits<float>::max(),
		      -std::numeric_limits<float>::max()};
  Vector3f min_mag = {std::numeric_limits<float>::max(),
	              std::numeric_limits<float>::max(),
		      std::numeric_limits<float>::max()};
  for(unsigned int i = 0; i < 1000; i++)
  {
    auto val = readMagBlocking();
    max_mag[0] = std::max(max_mag[0], val[0]);
    max_mag[1] = std::max(max_mag[1], val[1]);
    max_mag[2] = std::max(max_mag[2], val[2]);
    min_mag[0] = std::min(min_mag[0], val[0]);
    min_mag[1] = std::min(min_mag[1], val[1]);
    min_mag[2] = std::min(min_mag[2], val[2]);
  }

  _hard_iron_bias = (min_mag + max_mag)/2.0f;
  
  auto length_diff = (max_mag - min_mag)/2.0f;
  float average_elliptical_length = (_soft_iron_bias[0] + _soft_iron_bias[1] + _soft_iron_bias[2])/2.0f;
  _soft_iron_bias[0] = average_elliptical_length/length_diff[0];
  _soft_iron_bias[1] = average_elliptical_length/length_diff[1];
  _soft_iron_bias[2] = average_elliptical_length/length_diff[2];
}

void MPU9250::waitFor(const char addr, uint8_t mask, uint8_t value)
{
  //char val;
	std::cout << "doing wait" << '\n';
  while((readReg(addr) & mask) != value)
  {
    std::cout << "not ready" << '\n';
  }
}

}
