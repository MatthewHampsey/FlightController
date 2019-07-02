#include "mpu9250.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <exception>
#include <limits>
#include <bitset>
#include <chrono>

namespace FrameDrag{

MPU9250::MPU9250(const char * dev_path,
		 GyroScale gyro_scale,
		 AccelerometerScale acc_scale)
	: _gyro_scale_bits(gyro_scale)
	, _accel_scale_bits(acc_scale)
{
  _temp_buffer.resize(8);
  switch(gyro_scale)
  {
    case (GyroScale::two_hundred_and_fifty):
    _gyro_scale = 250.0f/32768.0f;
    break;
    case(GyroScale::five_hundred):
    _gyro_scale = 500.0f/32768.0f;
    break;
    case(GyroScale::one_thousand):
    _gyro_scale = 1000.0f/32768.0f;
    break;
    default:
    _gyro_scale = 2000.0f/32768.0f;
  }
  switch(acc_scale)
  {
    case (AccelerometerScale::two_g):
    _accel_scale = 2.0f/32768.0f;
    break;
    case(AccelerometerScale::four_g):
    _accel_scale = 4.0f/32768.0f;
    break;
    case(AccelerometerScale::eight_g):
    _accel_scale = 8.0f/32768.0f;
    break;
    default:
    _accel_scale = 16.0f/32768.0f;
  }
  
  if((fd = open(dev_path, O_RDWR)) < 0)
  {
    std::string err = "Couldn't open " + std::string(dev_path);
    throw std::runtime_error(err);
  }
  //wait for 100 ms for startup according to datasheet
  usleep(100000);
  
  //SPI mode should 0, so that data out changes on falling edge and data in latches on rising edge
  uint8_t mode = 0; 

  if(ioctl(fd, SPI_IOC_WR_MODE, &mode))
  {
    std::cerr << "Failed to set spi mode" << '\n';
  }

  if(ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0)
  {
    std::cerr << "Failed to read spi mode" << '\n';
  }

  writeReg(MPU9250_REG::USER_CTRL, {0b00110000});

  char whoami = readReg(0x75);
  if(whoami != 0x71){
    throw std::runtime_error("whoami check failed");
  }
  auto whoami_res = readRegBytes(0x75, 1);
  if(whoami_res[0] != 0x71){
    throw std::runtime_error("whoami check 2 failed");
  }

  calibrate();
  //clear sleep bit, set to use 20 MHz internal oscillator
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x00});
  waitFor(MPU9250_REG::INT_STATUS, 0x01, 0x01);

  //set to use PLL clock
  writeReg(MPU9250_REG::PWR_MGMT_1, {0x01});
  
  //set DLPF_CFG to 3: gyro BW is 41 Hz, gyro delay is 5.9ms, Fs is 1kHz
  //                   temp sensor BW is 42 Hz, delay is 4.8ms
  //writeReg(MPU9250_REG::CONFIG, {0x00});
  writeReg(MPU9250_REG::CONFIG, {0x03});
  
  // scale sample rate by 1/(1 + SMPLRT_DIV value) 
  writeReg(MPU9250_REG::SMPLRT_DIV, {0x04}); //divide sample rate by (4 + 1)


  uint8_t c = readReg(MPU9250_REG::GYRO_CONFIG);
  //clear fchoice_b [1:0]
  c = c & ~0x03;
  //clear scale selection bits [4:3]
  c = c & ~0x18;
  c = c | (static_cast<char>(_gyro_scale_bits) << 3);
     // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeReg(MPU9250_REG::GYRO_CONFIG, {c});
  
  c = readReg(MPU9250_REG::ACCEL_CONFIG);
  //clear scale selection bits
  c = c & ~0x18;
  c = c | (static_cast<uint8_t>(_accel_scale_bits) << 3);
  writeReg(MPU9250_REG::ACCEL_CONFIG, {c});
  
  c = readReg(MPU9250_REG::ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeReg(MPU9250_REG::ACCEL_CONFIG2, {c});
  
  //set bit 4 (INT_ANYRD_2CLEAR) to true, interrupt pin status is reset on a read
  //also set bit 1 (BYPASS_EN) to true so we can access the other i2c devices (eg. the magnetometer) on the chip
  writeReg(MPU9250_REG::INT_PIN_CFG, {0b00010010});

  //enable RAW_RDY_EN, allows the raw sensor data interrupt value to propagate to the interrupt pin
  writeReg(MPU9250_REG::INT_ENABLE, {0x01}); 
  
  //configure magnetometer as slave
  writeReg(MPU9250_REG::USER_CTRL, {0b00110000});
  writeReg(MPU9250_REG::I2C_MST_CTRL, {0x19});

  resetMag();

  auto mag_whoami = readMagReg(AK8963_REG::WIA);
  if(mag_whoami != 0x48)
  {
    throw std::runtime_error("mag whoami check failed");
  }

  //power down mag
  writeMagReg(AK8963_REG::CNTL1, {0x00});
  usleep(1000);
  //enter Fuse ROM access mode
  writeMagReg(AK8963_REG::CNTL1, {0x0F});
  
  auto asax = readMagReg(AK8963_REG::ASAX);
  auto asay = readMagReg(AK8963_REG::ASAY);
  auto asaz = readMagReg(AK8963_REG::ASAZ);
  
  //16-bit twos complement measurement so measurements are scaled by 4912/32760 microTeslas (check datasheet)
  float scale_factor = 4912.0f/32760.0f;

  _H_scale[0] = scale_factor*(float)(asax-128)/256.0f + 1.0f;
  _H_scale[1] = scale_factor*(float)(asay-128)/256.0f + 1.0f;
  _H_scale[2] = scale_factor*(float)(asaz-128)/256.0f + 1.0f;
  
  //have to return to power down before switching to continuous read mode, check data sheet.
  writeMagReg(AK8963_REG::CNTL1, {0x00});
  usleep(1000);
  //continuous mode 1 0010 8Hz measurement
  //continuous mode 2 0110 100Hz measurement
  //bit 4: 0 for 14-bit output, 1 for 16-bit output
  writeMagReg(AK8963_REG::CNTL1, {0b00010110}); //set to 16-bit output, mode 2
  enableMagI2CSlave();
//  calibrateAK8963();
}

MPU9250::~MPU9250()
{
  if(fd != -1)
  {
    close(fd);
  }
}


void MPU9250::enableMagI2CSlave()
{
  writeReg(MPU9250_REG::I2C_SLV0_CTRL, {0x00}); //disable slave
  writeReg(MPU9250_REG::I2C_SLV0_ADDR, {128 | 0x0C});
  writeReg(MPU9250_REG::I2C_SLV0_REG, {AK8963_REG::INFO});
  //set continuous read?
  writeReg(MPU9250_REG::I2C_SLV0_CTRL, {0x80 | 10}); //enable reading from slave | read 10 bytes
}

char MPU9250::readMagReg(const uint8_t addr){

  //mag address is 0x0C

  writeReg(MPU9250_REG::I2C_SLV4_ADDR, {0x80 | 0x0C});
  writeReg(MPU9250_REG::I2C_SLV4_REG, {addr});
  writeReg(MPU9250_REG::I2C_SLV4_CTRL, {0x80 | 0x01}); //enable reading from slave | read 1 byte

  char master_status = 0x00;
  auto begin = std::chrono::high_resolution_clock::now();
  auto timeout = std::chrono::milliseconds(5);
  while(((master_status = readReg(MPU9250_REG::I2C_MST_STATUS)) & 0x40) == 0)
  {
    auto duration = std::chrono::high_resolution_clock::now() - begin;
    if(duration >= timeout)
    {
      throw std::runtime_error("ReadMagReg - transfer ack timed out");
    }

  }
  if(master_status & 0x10)
  {
    throw std::runtime_error("SLV4 NACK asserted");
  }
  
  return readReg(MPU9250_REG::I2C_SLV4_DI);
}

void MPU9250::writeMagReg(const uint8_t addr, const std::vector<uint8_t>& data)
{
  //mag address is 0x0C

  //might need to move this line to later
  writeReg(MPU9250_REG::I2C_SLV4_ADDR, {0x0C});
  writeReg(MPU9250_REG::I2C_SLV4_REG, {addr});
  writeReg(MPU9250_REG::I2C_SLV4_DO, data);
  writeReg(MPU9250_REG::I2C_SLV4_CTRL, {0x80 | 0x01}); //enable reading from slave | read 1 byte

  //add timeout here 
  char master_status = 0x00;
  auto begin = std::chrono::high_resolution_clock::now();
  auto timeout = std::chrono::milliseconds(5);
  while(((master_status = readReg(MPU9250_REG::I2C_MST_STATUS)) & 0x40) == 0)
  {
    auto duration = std::chrono::high_resolution_clock::now() - begin;
    if(duration >= timeout)
    {
      throw std::runtime_error("WriteMagReg - transfer ack timed out");
    }
  }
  if(master_status & 0x10)
  {
    throw std::runtime_error("WriteMagReg - SLV4 NACK asserted");
  }
  
  return;
}

void MPU9250::resetMag()
{
  writeMagReg(AK8963_REG::CNTL2, {0x01});
}

char MPU9250::readReg(const uint8_t addr){
  uint8_t tx[2] = {static_cast<uint8_t>(addr | 128), 129};
  uint8_t rx[2] = {0, 0};
  
  struct spi_ioc_transfer tr = {0};
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = 2;
  tr.delay_usecs = 0;
  tr.speed_hz = 1000000;
  tr.bits_per_word = 8;
  auto ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

  if(ret < 1)
  {
    perror("err:");
    throw std::runtime_error("SPI message transfer failed");
  }

  return rx[1];
}

std::vector<char> MPU9250::readRegBytes(const char addr, size_t length){

  size_t read_length = length + 1;
  std::vector<uint8_t> tx(read_length, 129);
  tx[0] = addr | 128;

  std::vector<uint8_t> rx(read_length, 0);
  
  struct spi_ioc_transfer tr = {0};
  tr.tx_buf = (unsigned long)tx.data();
  tr.rx_buf = (unsigned long)rx.data();
  tr.len = length + 1;
  tr.delay_usecs = 0;
  tr.speed_hz = 1000000;
  tr.bits_per_word = 8;
  auto ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

  if(ret < 1)
  {
    perror("err:");
    throw std::runtime_error("SPI message transfer failed");
  }

  return std::vector<char>(rx.begin()+1, rx.end());
}


void MPU9250::readRegBytes(const uint8_t addr, size_t length, std::vector<char>& bytes){
  size_t read_length = length + 1;
  std::vector<uint8_t> tx(read_length, 129);
  tx[0] = addr | 128;

  std::vector<uint8_t> rx(read_length, 0);

  struct spi_ioc_transfer tr = {0};
  tr.tx_buf = (unsigned long)tx.data();
  tr.rx_buf = (unsigned long)rx.data();
  tr.len = length + 1;
  tr.delay_usecs = 0;
  tr.speed_hz = 1000000;
  tr.bits_per_word = 8;
  auto ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

  if(ret < 1)
  {
    perror("err:");
    throw std::runtime_error("SPI message transfer failed");
  }
  std::copy(rx.begin() + 1, rx.end(), bytes.begin());
}


void MPU9250::writeReg(const uint8_t addr, const std::vector<uint8_t>& data)
{
  std::vector<uint8_t> tx{addr};
  tx.insert(tx.end(), data.begin(), data.end());
  uint8_t rx[10] = {0};
  struct spi_ioc_transfer tr = {};
  tr.tx_buf = (unsigned long)tx.data();
  tr.rx_buf = (unsigned long)rx;
  tr.len = tx.size();
  tr.delay_usecs = 0;
  tr.speed_hz = 1000000;
  tr.bits_per_word = 8;
  auto ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
  if(ret < 1)
  {
    throw std::runtime_error("SPI message transfer failed");
  }
}

int16_t MPU9250::readHighLowReg(const char addr, char maskH, char maskL)
{
  char high = readReg(addr) & maskH;
  char low = readReg(addr + 1) & maskL;
  return (int16_t)(((int16_t)high << 8) | low);
}

int16_t MPU9250::readLowHighReg(const char addr, char maskL, char maskH)
{
  char low = readReg(addr) & maskL;
  char high = readReg(addr + 1) & maskH;
  return (int16_t)(((int16_t)high << 8) | low);
}

void MPU9250::setMagBiases(const Vector3f soft_iron_bias, const Vector3f hard_iron_bias)
{
  _soft_iron_bias = soft_iron_bias;
  _hard_iron_bias = hard_iron_bias;
}

int16_t concat(char high, char low)
{
  return (int16_t)((high << 8) | low);
}

Vector3f& MPU9250::getGyro()
{
  return _prev_gyro;
}
Vector3f& MPU9250::getAcc()
{
  return _prev_acc;
}
Vector3f& MPU9250::getMag()
{
  return _prev_H;
}

bool MPU9250::readMagSensorValues(Vector3f& mag)
{
  auto data = readRegBytes(MPU9250_REG::EXT_SENS_DATA_00, 10);
  //acc at 1-6
  //gyro at 9-14
  //mag data starts at 15, from INFO
  //check mag st1 and overflow
  if((data[1] & 0x01) && !(data[8] & 0b00001000))
  {
    //mag is low - high
    auto x = concat(data[3], data[2]);
    auto y = concat(data[5], data[4]);
    auto z = concat(data[7], data[6]);
    mag = Vector3f{(float)x*_H_scale[0], (float)y*_H_scale[1], (float)z*_H_scale[2]};
    return true;
  }
  return false;
}

void MPU9250::updateGyroAccAndMag(bool& updated_gyro_acc, bool& updated_mag)
{
  std::vector<char> data(24, 0);
  readRegBytes(MPU9250_REG::INT_STATUS, 24, data);
  //acc at 1-6
  //gyro at 9-14
  //mag data starts at 15, from INFO

  //check gyro and mag data is ready
  updated_gyro_acc = false;
  updated_mag = false;
  if(data[0] & 0x01)
  {
    auto a_x = concat(data[1], data[2]);
    auto a_y = concat(data[3], data[4]);
    auto a_z = concat(data[5], data[6]);
    auto g_x = concat(data[9], data[10]);
    auto g_y = concat(data[11], data[12]);
    auto g_z = concat(data[13], data[14]);
    _prev_gyro = Vector3f{static_cast<float>(g_x)*_gyro_scale, 
	  	      static_cast<float>(g_y)*_gyro_scale, 
		      static_cast<float>(g_z)*_gyro_scale};
    _prev_acc = Vector3f{(float)a_x*_accel_scale, (float)a_y*_accel_scale, (float)a_z*_accel_scale};

    _prev_acc -= _accel_bias;
    updated_gyro_acc = true;
  }
  //check mag st1 and overflow
  if((data[16] & 0x01) && !(data[23] & 0b00001000))
  {
    //mag is low - high
    auto x = concat(data[18], data[17]);
    auto y = concat(data[20], data[19]);
    auto z = concat(data[22], data[21]);
    auto H_vec = Vector3f{(float)x*_H_scale[0], (float)y*_H_scale[1], (float)z*_H_scale[2]};
    

    auto mag = H_vec - _hard_iron_bias;
    _prev_H = Vector3f{mag[0]*_soft_iron_bias[0],
                       mag[1]*_soft_iron_bias[1],
                       mag[2]*_soft_iron_bias[2]};
    updated_mag = true;
  }
}

Vector3f MPU9250::readMagBlocking()
{
  Vector3f result;
  while(!(readMagSensorValues(result))){}
  
  return result;
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

  writeReg(MPU9250_REG::USER_CTRL, {0x00 | 0b00010000});    // Disable FIFO and I2C master modes
  writeReg(MPU9250_REG::USER_CTRL, {0x0C | 0b00010000});    // Reset FIFO and DMP
  waitFor(MPU9250_REG::INT_STATUS, 0x01, 0x01);
	       
  // Configure MPU9250 gyro and accelerometer for bias calculation
  //set no FIFO rewrite. Set DLPF_CONFIG to 1 -> 184Hz gyro BW
  writeReg(MPU9250_REG::CONFIG, {0x41});      // Set low-pass filter to 188 Hz
  writeReg(MPU9250_REG::SMPLRT_DIV, {0x00});  // Set sample rate to 1 kHz

  //clear fchoice on gyro	
  writeReg(MPU9250_REG::GYRO_CONFIG, {0x00});  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeReg(MPU9250_REG::ACCEL_CONFIG, {0x00}); // Set accelerometer full-scale to 2 g, maximum sensitivity

  int32_t accelsensitivity = 16384; // = 16384 LSB/g

  uint16_t fifo_count = readHighLowReg(MPU9250_REG::FIFO_COUNTH, 0b0001111, 0xFF);

  writeReg(MPU9250_REG::USER_CTRL, {0b01010000});   // Enable FIFO  
  writeReg(MPU9250_REG::FIFO_EN, {0b01111000});     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  usleep(40000); // accumulate 40 samples in 80 milliseconds = 480 bytes

  fifo_count = readHighLowReg(MPU9250_REG::FIFO_COUNTH, 0b0001111, 0xFF);
  int16_t packet_count = fifo_count/12;
  std::vector<int16_t> gyro_meas{0, 0, 0};
  std::vector<int16_t> accel_meas{0, 0, 0};
  std::vector<int32_t> gyro_bias{0, 0, 0};
  for (uint16_t i = 0; i < packet_count; i++) {
	  
    auto data = readRegBytes(MPU9250_REG::FIFO_R_W, 12); // read data for averaging
    accel_meas[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_meas[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_meas[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_meas[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_meas[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_meas[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    _accel_bias[0] += accel_meas[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    _accel_bias[1] += accel_meas[1];
    _accel_bias[2] += accel_meas[2];
    gyro_bias[0]  += gyro_meas[0];
    gyro_bias[1]  += gyro_meas[1];
    gyro_bias[2]  += gyro_meas[2];								              
  }
  float accel_scale = packet_count*accelsensitivity;
  _accel_bias[0] /= accel_scale; // Normalize sums to get average count biases
  _accel_bias[1] /= accel_scale;
  _accel_bias[2] /= accel_scale;
  gyro_bias[0] /= (int32_t) packet_count;
  gyro_bias[1] /= (int32_t) packet_count;
  gyro_bias[2] /= (int32_t) packet_count;
   

  if(_accel_bias[2] > 0L) {_accel_bias[2] -= 1.0f;} // Remove gravity from the z-axis accelerometer bias calculation
  else {_accel_bias[2] += 1.0f;}
  std::vector<uint8_t> data(6);  
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4) & 0xFF;

  writeReg(MPU9250_REG::XG_OFFSET_H, {data[0]});
  writeReg(MPU9250_REG::XG_OFFSET_L, {data[1]});
  writeReg(MPU9250_REG::YG_OFFSET_H, {data[2]});
  writeReg(MPU9250_REG::YG_OFFSET_L, {data[3]});
  writeReg(MPU9250_REG::ZG_OFFSET_H, {data[4]});
  writeReg(MPU9250_REG::ZG_OFFSET_L, {data[5]});
}

void MPU9250::calibrateAK8963()
{
  Vector3f max_mag = {-std::numeric_limits<float>::max(),
	              -std::numeric_limits<float>::max(),
		      -std::numeric_limits<float>::max()};
  Vector3f min_mag = {std::numeric_limits<float>::max(),
	              std::numeric_limits<float>::max(),
		      std::numeric_limits<float>::max()};
  for(unsigned int i = 0; i < 10000; i++)
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
  float average_elliptical_length = (length_diff[0] + length_diff[1] + length_diff[2])/3.0f;
  _soft_iron_bias[0] = average_elliptical_length/length_diff[0];
  _soft_iron_bias[1] = average_elliptical_length/length_diff[1];
  _soft_iron_bias[2] = average_elliptical_length/length_diff[2];
}

void MPU9250::waitFor(const char addr, uint8_t mask, uint8_t value)
{
  while((readReg(addr) & mask) != value)
  {
  }
}

}
