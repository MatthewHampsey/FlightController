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

char readReg(const int fd, const char addr){

  if(write(fd, &addr, 1) < 0){
    std::cout << "failed to write" << '\n';
    throw std::runtime_error("failed write");
  }
  char recv;
  if(read(fd, &recv, 1) < 0){
    std::cout << "failed to read" << '\n';
    throw std::runtime_error("failed read");
  }
  return recv;
}

float getFloatFromReg(const int fd, const char addr)
{
  char highReg = readReg(fd, addr);
  char lowReg = readReg(fd, addr+1);
  uint16_t combined = (highReg << 8) + lowReg;
  return (float)combined; 
}

int main(){
  
	FrameDrag::MPU9250 mpu9250{"/dev/i2c-2", 
	           FrameDrag::MPU9250::GyroScale::one_thousand,
                   FrameDrag::MPU9250::AccelerometerScale::eight_g};
  //mpu9250.calibrate();
  /*//std::cout << "starting... " << '\n';
  //}
  //std::cout << "whoami reg: " << (uint32_t)readReg(fd, 117) << '\n';

  //while(0){
  //  auto x = getFloatFromReg(fd, 0x3B); 
  //  auto y = getFloatFromReg(fd, 0x3D); 
  //  auto z = getFloatFromReg(fd, 0x3F); 
  //}

  
  //close(fd);
*/
  auto init_mag_m = mpu9250.readCalibratedMag();
  while((init_mag_m[0] + init_mag_m[1] + init_mag_m[2]) == 0.0f)
  {
    init_mag_m = mpu9250.readCalibratedMag(); 
  }
  std::cout << "init mag: " << init_mag_m << '\n';
  init_mag_m[0] = std::sqrt(init_mag_m[0]*init_mag_m[0] + init_mag_m[1]*init_mag_m[1]);
  init_mag_m[1] = 0.0f;
  //FrameDrag::MahonyFilter filter{
//	  {1.0f, 0.0f, 0.0f, 0.0f},
//	  {std::make_pair(0.5f, FrameDrag::Vector3f{0.0, 0.0, -1.0f}),
//	   std::make_pair(0.4f, init_mag_m)}, 
//	  0.2f, 0.002f
  //};
  FrameDrag::Quaternion initial_quat{1.0f, 1.0f, 1.0f, 1.0f};
  initial_quat /= initial_quat.norm();

  FrameDrag::MadgwickMARGFilter filter{
		        initial_quat,
			1.2f,
			0.2f
			};



  //know sample rate, can just check sample number??
  clock_t prev_time = clock();
  for(unsigned int i = 0; i < 10000000; i++)
  {
    //usleep(30000);
//    std::cout << mpu9250.readGyro() << '\n';
    //std::cout << mpu9250.readAcc() << '\n';
    //const clock_t current_time = clock();
    auto gyro_m = mpu9250.readGyro();
    auto grav_m = mpu9250.readAcc();
    //grav_m[2] *= -1.0f; 
    auto mag_m = mpu9250.readCalibratedMag();
    clock_t current_time = clock();
    float delta_t = float(current_time - prev_time) / CLOCKS_PER_SEC;// + 0.00000001;
    //filter.update(gyro_m,
	//	  {grav_m/grav_m.norm(),
	//	   mag_m/mag_m.norm()},
	//	  delta_t);
   filter.update(gyro_m,
		 9.8*grav_m,
		 mag_m,
		 delta_t);

    auto est = filter.estimate();
    auto vec = est.im();
    float roll = atan2(2*(est.re()*vec[0] + vec[1]*vec[2]), 1 - 2*(vec[0]*vec[0] + vec[1]*vec[1]));
    float pitch = asin(2*(est.re()*vec[1] - vec[2]*vec[0]));
    float yaw = atan2(2*(est.re()*vec[2] + vec[0]*vec[1]), 1 - 2*(vec[1]*vec[1] + vec[2]*vec[2]));
    float angle = 2.0f*acos(est.re());
    auto axis = est.im()/est.im().norm();
    //std::cout << "time diff: " << delta_t << '\n';
    if(i % 100 == 0){
      std::cout << roll << "   " << pitch << "    " << yaw << '\n';// " axis: " << axis << '\n';
    }
    //std::cout << "angle: " << angle << "      " << axis << '\n';
    prev_time = current_time;
  }
  return 0;
}


