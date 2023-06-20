#ifndef WHEEL_CONFIG_HPP_
#define WHEEL_CONFIG_HPP_

#include <string>
#include "dynamixel_wrapper/fake_dynamixel_handle.hpp"
#include "dynamixel_wrapper/dynamixel_handle.hpp"

using namespace std;

struct WheelConfig 
{
  shared_ptr<FakeDynamixelHandle> fake_motor;
  shared_ptr<DynamixelHandle>     motor;

  bool                            real_hardware = 0;
  
  string                          wheel_name = "";
  int16_t                         wheel_id = -1;
  bool                            mode = 1; // 1: POSITION_CONTROL; 0: VELOCITY_CONTROL
  
  double                          encoder_pos = 0;
  double                          encoder_vel = 0;

  double                          goal = 0;

  double                          radius = 0.05;
};

#endif // WHEEL_CONFIG_HPP_