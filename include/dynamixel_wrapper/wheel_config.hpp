#ifndef WHEEL_CONFIG_HPP_
#define WHEEL_CONFIG_HPP_

#include <string>
#include "dynamixel_wrapper/fake_dynamixel_handle.hpp"
#include "dynamixel_wrapper/dynamixel_handle.hpp"

using namespace std;

struct WheelConfig 
{
  shared_ptr<FakeDynamixelHandle> fake_motor;
  shared_ptr<DynamixelHandle>     real_motor;

  bool                            real_hardware = 0;    // 1: using real hardware; 0: fake hardware
  
  string                          wheel_name = "";
  int16_t                         wheel_id = -1;
  bool                            mode = 1;             // 1: POSITION_CONTROL; 0: VELOCITY_CONTROL
  
  double                          encoder_pos = 0;      // [rad]
  double                          encoder_vel = 0;      // [rpm]

  double                          pos_multiplier = 1;
  double                          vel_multiplier = 1;

  double                          goal = 0;             // [rpm]

  double                          radius = 0.05;        // [m]
};

#endif // WHEEL_CONFIG_HPP_