#include <stdio.h>
#include <fcntl.h>
#include <cmath>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include "mpu9250.h"
#include "mahony_complimentary_filter.h"
#include "madgwick_marg_filter.h"
#include "quat_control.h"
#include "type_conversion.h"
#include "prop_controller_x.h"
#include <chrono>
#include <boost/property_tree/json_parser.hpp>
#include <termios.h>
#include <sys/mman.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct{
  uint32_t A;
  uint32_t B;
  uint32_t C;
  uint32_t D;
  uint32_t period;
} DutyCycles;


struct RxMessage
{
  uint8_t header;
  uint8_t length;
  uint8_t type;
  int16_t data[16];
  uint8_t flags;
  uint8_t RSSI;
  uint8_t crc;
  uint8_t footer;
};

template <typename Iterator, typename OutputIterator>
void parseSBUSArray(Iterator begin, Iterator end, OutputIterator result)
{
  assert(std::distance(begin, end) > 7);
  *result = *begin;
  ++begin;
  *result |= ((*begin & 0b00000111) << 8);
  ++result;
  *result  = ((*begin & 0b11111000) >> 3);
  ++begin;
  *result |= ((*begin & 0b00111111) << 5); 
  ++result;
  *result  = ((*begin & 0b11000000) >> 6);
  ++begin;
  *result |= (*begin << 2);
  ++begin;
  *result |= ((*begin & 0b00000001) << 10); 
  ++result;
  *result  = ((*begin & 0b11111110) >> 1);
  ++begin;
  *result |= ((*begin & 0b00001111) << 7); 
  ++result;
  *result  = ((*begin & 0b11110000) >> 4);
  begin++;
  *result |= ((*begin & 0b01111111) << 4); 
  ++result;
  *result  = ((*begin & 0b10000000) >> 7);
  begin++;
  *result |= (*begin << 1);
  begin++;
  *result |= ((*begin & 0b00000011) << 9);
  ++result;
  *result  = ((*begin & 0b11111100) >> 2);
  begin++;
  *result |= ((*begin & 0b00011111) << 6);
  ++result;
  *result  = ((*begin & 0b11100000) >> 5);
  begin++;
  *result |= (*begin << 3);
}

RxMessage parseMessage(unsigned char * buf)
{
  RxMessage msg{};
  msg.header = buf[0];
  msg.length = buf[1];
  msg.type   = buf[2];
  parseSBUSArray(&buf[3], &buf[3] + 11, msg.data);
  parseSBUSArray(&buf[14], &buf[14] + 11, msg.data + 8);
  //get flag bs from bytes 25, 26
  msg.crc = buf[27];
  msg.footer = buf[28];
  return msg;
}


int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* enable parity bit */
    //tty.c_cflag &= ~PARODD;    /* even parity */
    tty.c_cflag &= ~CSTOPB;     /* need 2 stop bits */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

int main(){
  uint8_t *pru;
  int dev_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if(dev_fd < 0){
    printf("can't open dev mem\n");
    return -1;
  }

  pru = (uint8_t*)mmap(0, 20, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, 0x4A300000);
  boost::property_tree::ptree pt;
  boost::property_tree::read_json("hw/config.json", pt);  
  bool debug = pt.get_child("debug").get_value<bool>();	
  FrameDrag::MPU9250 mpu9250{"/dev/spidev1.0", 
//  FrameDrag::MPU9250 mpu9250{"/dev/i2c-2", 
  	FrameDrag::MPU9250::GyroScale::two_thousand,
  	FrameDrag::MPU9250::AccelerometerScale::sixteen_g};
  FrameDrag::Vector3f soft_iron_bias;
  unsigned int index = 0;
  for( auto &x : pt.get_child("soft_iron_bias"))
  {
    soft_iron_bias[index++] = x.second.get_value<float>();
  }
  FrameDrag::Vector3f hard_iron_bias;
  index = 0;
  for( auto &x : pt.get_child("hard_iron_bias"))
  {
    hard_iron_bias[index++] = x.second.get_value<float>();
  }
  mpu9250.setMagBiases(soft_iron_bias, hard_iron_bias);

  FrameDrag::Vector3f init_mag_m{};
  for(int i = 1; i < 250;)
  {
    bool read_acc = false;
    bool read_mag = false;
    mpu9250.updateGyroAccAndMag(read_acc, read_mag);
    if(read_mag){
      std::cout << "init mag: " << mpu9250.getMag() << '\n';
      ++i;	    
      auto prev = init_mag_m;
      init_mag_m = prev + (mpu9250.getMag() - prev)/i;
    }
  }
  std::cout << "avg init: " << init_mag_m << '\n';
  float temp_mag = init_mag_m[0];
  init_mag_m[0] = init_mag_m[1];
  init_mag_m[1] = temp_mag;
  init_mag_m[2] = -1.0f*init_mag_m[2];
  init_mag_m[0] = std::sqrt(init_mag_m[0]*init_mag_m[0] + init_mag_m[1]*init_mag_m[1]);
  init_mag_m[1] = 0.0f;
  std::cout << "k_g: " << pt.get_child("k_g").get_value<float>() << '\n';
  std::cout << "k_h: " << pt.get_child("k_h").get_value<float>() << '\n';
  std::cout << "k_i: " << pt.get_child("k_i").get_value<float>() << '\n';

  FrameDrag::MahonyFilter filter{
  	  {1.0f, 0.0f, 0.0f, 0.0f},
  	  {std::make_pair(pt.get_child("k_g").get_value<float>(), FrameDrag::Vector3f{0.0, 0.0, -1.0f}),
  	   std::make_pair(pt.get_child("k_h").get_value<float>(), init_mag_m)}, 
          1.0f, pt.get_child("k_i").get_value<float>()
//  	  1.0f, 0.002f
  };

  auto I = FrameDrag::Matrix3f{ 2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 4.0f };
  FrameDrag::QuaternionController controller(I);
  controller.setParameters(0.85f, 1.0f);

  //make sure to measure wall clock time, not proc time!
  auto start = std::chrono::high_resolution_clock::now();
  FrameDrag::Quaternion prev_quat = FrameDrag::Quaternion();
  auto target_euler_angles = FrameDrag::Vector3f{0.0f, 0.0f, 0.0f};
  FrameDrag::PropellerControllerX prop_controller(1.0f, 1.0f, 1.0f, 1.0f);//length, k_f, k_t)
  
  const char *portname = "/dev/ttyS4";
  int fd = open(portname, O_RDWR | O_NONBLOCK | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("Error opening %s: %s\n", portname, strerror(errno));
    return -1;
  }
  /*baudrate 110, 8 bits, no parity, 1 stop bit */
  set_interface_attribs(fd, B115200);
  unsigned char buf[50];

  while(true)
  {

    int read_length = read(fd, buf, sizeof(buf));
    if(read_length > 0)
    {
      auto msg = parseMessage(buf);
      std::cout << std::hex << "header: " << (uint32_t)msg.header << '\n';
      std::cout << std::hex << "footer: " << (uint32_t)msg.footer << '\n';
      //static volatile int x = 0;
      //++x;
      //std::cout << "read some data!" << '\n\n\n\n\n';
    }
    bool read_acc = false;
    bool read_mag = false;
    mpu9250.updateGyroAccAndMag(read_acc, read_mag);
    auto finish = std::chrono::high_resolution_clock::now();
    
    auto gyro_m = mpu9250.getGyro()*3.14159/180.0f;
    auto grav_m = -1.0f*mpu9250.getAcc();
    auto mag_m = mpu9250.getMag();
    std::chrono::duration<double> delta_t = finish-start;
    float temp_mag = mag_m[0];
    //swap x and y of mag to align with imu axes
    mag_m[0] = mag_m[1];
    mag_m[1] = temp_mag;
    mag_m[2] = -1.0f*mag_m[2];
    
    filter.update((gyro_m),
  		  {(grav_m/grav_m.norm()),
		   (mag_m/mag_m.norm())},
		   delta_t.count());

    //filter quaternion holds body orientation relative to world frame
    auto quat_deriv = (filter.estimate() - prev_quat)/(delta_t.count());
    FrameDrag::Vector3f torque = controller.getControlVector(
		            filter.estimate(), quat_deriv, ZYXEulerToQuaternion(target_euler_angles),
			    quat_deriv);
    //prop_controller.applyControlTargets(9.81, torque);
    prop_controller.applyControlTargets(0.0, torque);
    auto prop_speeds = prop_controller.getCurrentPropSpeed();
    std::cout << "prop speeds: " << prop_speeds << '\n';    
    for(int i = 0; i < 4; i++)
    {
      prop_speeds[i] = std::max(0.0f, std::min(prop_speeds[i], 1.0f));
    }

    DutyCycles duty_cycles = {
      (int)(200000/0.5*prop_speeds[0] + 200000),
      (int)(200000/0.5*prop_speeds[1] + 200000),
      (int)(200000/0.5*prop_speeds[2] + 200000),
      (int)(200000/0.5*prop_speeds[3] + 200000),
      4000000
    };
    std::cout << "duty cycles: " << '\n';
    std::cout << duty_cycles.A << '\n';
    std::cout << duty_cycles.B << '\n';
    std::cout << duty_cycles.C << '\n';
    std::cout << duty_cycles.D << '\n';
    *(DutyCycles*)pru = duty_cycles;

    auto now2 = std::chrono::high_resolution_clock::now();
    auto time_diff = now2 - start;
    auto ms = std::chrono::duration_cast<std::chrono::microseconds>(time_diff);
    
    static float avg = 0.0f;
    static int i = 0;

    auto prev_avg = avg;
    avg = prev_avg + (1.0f/(i+1))*(ms.count() - prev_avg);
    start = finish;
    prev_quat = filter.estimate();
    i++;
    if(debug && (i % 1000 == 0)){
     // std::cout << "avg time: " << avg << '\n';
     //   std::cout << "time: " << ms.count() << " " << read_length << '\n';
        //std::cout << "read time: " << (read_length > 0) << '\n';
//      std::cout << "torque: " << torque << '\n';
//      std::cout << "prop speeds: " << prop_speeds << '\n';
    }
  }
  close(dev_fd);
  close(fd);
  munmap(pru, 20);
  return 0;
}


