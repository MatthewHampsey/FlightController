#include <iostream>
#include "mpu9250.h"
#include "mahony_complimentary_filter.h"
#include "quat_control.h"
#include "type_conversion.h"
#include "prop_controller_x.h"
#include "angular_velocity_conversion.h"
#include <chrono>
#include <boost/property_tree/json_parser.hpp>
#include "receiver.h"
#include "pru_comms.h"

int main(){

  PRUComms pru_comms;

  boost::property_tree::ptree pt;
  boost::property_tree::read_json("hw/config.json", pt);  
  bool debug = pt.get_child("debug").get_value<bool>();	
  FrameDrag::MPU9250 mpu9250{"/dev/spidev1.0", 
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
  };

  auto I = FrameDrag::Matrix3f{ 0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.2f };
  FrameDrag::QuaternionController controller(I);
  controller.setParameters(1.0f, 5.0f);

  //make sure to measure wall clock time, not proc time!
  auto start = std::chrono::high_resolution_clock::now();
  FrameDrag::Quaternion prev_quat = FrameDrag::Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
  auto target_euler_angles = FrameDrag::Vector3f{0.0f, 0.0f, 0.0f};
  auto target_angular_velocity = FrameDrag::Vector3f{0.0f, 0.0f, 0.0f};
  FrameDrag::PropellerControllerX prop_controller(1.0f, 1.0f, 1.0f, 1.0f);//length, k_f, k_t)
  
  Receiver receiver("/dev/ttyS4");
  RxMessage msg{};

  while(true)
  {

    auto maybe_msg = receiver.getMessage();
    if(maybe_msg)
    {
      msg = maybe_msg.value();
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

    auto new_ms = std::chrono::duration_cast<std::chrono::seconds>(delta_t);

    auto filtered_ang_vel = FrameDrag::QuaternionAndTimeToBodyFrameAngularVelocity(prev_quat, filter.estimate(), delta_t.count()); 

    FrameDrag::Vector3f torque = controller.getControlVector(
		            filter.estimate(), filtered_ang_vel, ZYXEulerToQuaternion(target_euler_angles),
			    target_angular_velocity);
    prop_controller.applyControlTargets(0.0, torque);
    auto prop_speeds = prop_controller.getCurrentPropSpeed();
    for(int i = 0; i < 4; i++)
    {
      prop_speeds[i] = std::max(0.0f, std::min(prop_speeds[i], 1.0f));
    }

    DutyCycles duty_cycles = {
      (uint32_t)(200000/0.5*prop_speeds[0] + 200000),
      (uint32_t)(200000/0.5*prop_speeds[1] + 200000),
      (uint32_t)(200000/0.5*prop_speeds[2] + 200000),
      (uint32_t)(200000/0.5*prop_speeds[3] + 200000),
      400000
    };

    pru_comms.setDutyCycles(duty_cycles);

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
      //std::cout << "torque: " << torque << '\n';
      //std::cout << "euler: " << QuaternionToZYXEuler(prev_quat) << '\n';
      //std::cout << "prop speeds: " << prop_speeds << '\n';    
     // std::cout << "avg time: " << avg << '\n';
     //   std::cout << "time: " << ms.count() << " " << read_length << '\n';
        //std::cout << "read time: " << (read_length > 0) << '\n';
//      std::cout << "torque: " << torque << '\n';
//      std::cout << "prop speeds: " << prop_speeds << '\n';
    }
  }
  return 0;
}


