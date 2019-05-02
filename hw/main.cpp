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
#include <chrono>
#include <boost/property_tree/json_parser.hpp>

int main(){
  
  boost::property_tree::ptree pt;
  boost::property_tree::read_json("hw/config.json", pt);  
  bool debug = pt.get_child("debug").get_value<bool>();	
  FrameDrag::MPU9250 mpu9250{"/dev/i2c-2", 
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

  mpu9250.waitForMagMeasurement();
  auto init_mag_m = mpu9250.readCalibratedMag();
  init_mag_m[0] = std::sqrt(init_mag_m[0]*init_mag_m[0] + init_mag_m[1]*init_mag_m[1]);
  init_mag_m[1] = 0.0f;
  FrameDrag::MahonyFilter filter{
  	  {1.0f, 0.0f, 0.0f, 0.0f},
  	  {std::make_pair(0.5f, FrameDrag::Vector3f{0.0, 0.0, -1.0f}),
  	   std::make_pair(0.4f, init_mag_m)}, 
  	  1.0f, 0.002f
  };
  //FrameDrag::Quaternion initial_quat{1.0f, 0.0f, 0.0f, 0.0f};
  //float GyroMeasError = 3.14159265f * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
  //float GyroMeasDrift = 3.14159265f * (0.0f / 180.0f);
  //float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
  //float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; 
  //FrameDrag::MadgwickMARGFilter filter{
  //		        initial_quat,
  //			beta,
  //			zeta,
  //			};


  //make sure to measure wall clock time, not proc time!
  auto start = std::chrono::high_resolution_clock::now();
  for(unsigned int i = 0; i < 10000000; i++)
  {
    auto ga_pair = mpu9250.readGyroAndAcc();
    auto finish = std::chrono::high_resolution_clock::now();
    
    auto gyro_m = ga_pair.first*3.14159/180.0f;

    auto grav_m = -1.0f*ga_pair.second;
    auto mag_m = mpu9250.readCalibratedMag();
    std::chrono::duration<double> delta_t2 = finish-start;

    float temp_mag = mag_m[0];
    //swap x and y of mag
    mag_m[0] = mag_m[1];
    mag_m[1] = temp_mag;
    mag_m[2] = -1.0f*mag_m[2];

    filter.update(gyro_m,
		  {grav_m/grav_m.norm(),
		   mag_m/mag_m.norm()},
		   delta_t2.count());
    
    //filter.update(gyro_m,
	//	 9.8*grav_m,
	//	 mag_m,
	//	 //0.005f);
	//	 delta_t);

    start = finish;
    auto est = filter.estimate();
    auto vec = est.im();
    
    float angle = 2.0f*acos(est.re());
    auto axis = vec/vec.norm();
    if(debug){
      std::cout << "angle: " << angle << "      " << axis << '\n';
    }
  }
  
  return 0;
}


