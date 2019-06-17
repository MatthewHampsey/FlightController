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
#include "dynamic_compensation_qc.h"
#include "type_conversion.h"
#include "prop_controller_x.h"
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

  auto I = FrameDrag::Matrix3f{ 2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 4.0f };
  FrameDrag::PDDynamic controller(I);
  controller.setParameters(0.25f, 7.0f);



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
  FrameDrag::Vector3f prev_euler = FrameDrag::Vector3f();
  auto target_euler_angles = FrameDrag::Vector3f{};
  auto target_euler_derivatives = FrameDrag::Vector3f{};
  FrameDrag::PropellerControllerX prop_controller(1.0f, 1.0f, 1.0f, 1.0f);//length, k_f, k_t)
  for(unsigned int i = 0; i < 10000000; i++)
  {
    auto ga_pair = mpu9250.readGyroAndAcc();
    auto finish = std::chrono::high_resolution_clock::now();
    
    auto gyro_m = ga_pair.first*3.14159/180.0f;

    auto grav_m = -1.0f*ga_pair.second;
    auto mag_m = mpu9250.readCalibratedMag();
    std::chrono::duration<double> delta_t = finish-start;

    float temp_mag = mag_m[0];
    //swap x and y of mag
    mag_m[0] = mag_m[1];
    mag_m[1] = temp_mag;
    mag_m[2] = -1.0f*mag_m[2];

    filter.update(gyro_m,
		  {grav_m/grav_m.norm(),
		   mag_m/mag_m.norm()},
		   delta_t.count());
    
    //filter.update(gyro_m,
	//	 9.8*grav_m,
	//	 mag_m,
	//	 //0.005f);
	//	 delta_t);

    
    auto euler = QuaternionToZYXEuler(filter.estimate());
    auto euler_deriv = (euler - prev_euler)/delta_t.count();
    
    FrameDrag::Vector3f torque = controller.getControlVector(
		            euler, euler_deriv, target_euler_angles,
			    target_euler_derivatives);
    prop_controller.applyControlTargets(9.81, torque);
    auto prop_speeds = prop_controller.getCurrentPropSpeed();
    start = finish;
    prev_euler = euler;

    //auto est = filter.estimate();
    //auto vec = est.im();
    
    //float angle = 2.0f*acos(est.re());
    //auto axis = vec/vec.norm();
    if(debug){
      std::cout << "euler angles: " << euler << '\n';
    }
  }
  
  return 0;
}


