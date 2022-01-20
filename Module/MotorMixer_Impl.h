/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include "MotorMixer.h"
#include "Device/MotorDriver.h"

/// ********************************************************************************************************************

class MotorMixer_Impl
{
public:

  explicit MotorMixer_Impl();
  ~MotorMixer_Impl();

  void  Init(const std::string &pwm_device, const std::string &gpio_device);
  void  Term();
  void  Output(MotorMixer::Power &power);
  void  Brake();

private:

  struct Mix
  {
    float p_;
    float y_;
  };

  enum Wheel
  {
    L,
    R,
    N
  };

  Mix   mix_value_[Wheel::N];

  MotorDriver   motor_driver_;
};

/// EOF ****************************************************************************************************************
