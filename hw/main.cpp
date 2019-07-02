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
  FrameDrag::PDDynamic controller(I);
  controller.setParameters(0.85f, 1.0f);

  //make sure to measure wall clock time, not proc time!
  auto start = std::chrono::high_resolution_clock::now();
  FrameDrag::Vector3f prev_euler = FrameDrag::Vector3f();
  auto target_euler_angles = FrameDrag::Vector3f{0.0f, 0.0f, 0.0f};
  auto target_euler_derivatives = FrameDrag::Vector3f{0.0f, 0.0f, 0.0f};
  FrameDrag::PropellerControllerX prop_controller(1.0f, 1.0f, 1.0f, 1.0f);//length, k_f, k_t)
  while(true)
  {
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
    auto euler = QuaternionToZYXEuler(filter.estimate());
    auto euler_deriv = (euler - prev_euler)/(delta_t.count());
    target_euler_angles[2] = euler[2];    
    target_euler_derivatives[2] = euler_deriv[2];    
    FrameDrag::Vector3f torque = controller.getControlVector(
		            euler, euler_deriv, target_euler_angles,
			    target_euler_derivatives);
    prop_controller.applyControlTargets(9.81, torque);
    auto prop_speeds = prop_controller.getCurrentPropSpeed();
    auto now2 = std::chrono::high_resolution_clock::now();
    auto time_diff = now2 - start;
    auto ms = std::chrono::duration_cast<std::chrono::microseconds>(time_diff);
    start = finish;
    prev_euler = euler;

    if(debug){
      std::cout << "euler angles: " << euler << '\n';
      std::cout << "torque: " << torque << '\n';
      std::cout << "prop speeds: " << prop_speeds << '\n';
    }
  }
  
  return 0;
}


