/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "MotorMixer_Impl.h"
#include "Misc/MiscTools.hpp"

/// ********************************************************************************************************************

MotorMixer_Impl::MotorMixer_Impl()
{
  mix_value_[Wheel::L].p_ = -1.0f;
  mix_value_[Wheel::L].y_ = -1.0f;
  mix_value_[Wheel::R].p_ = -1.0f;
  mix_value_[Wheel::R].y_ =  1.0f;
}

MotorMixer_Impl::~MotorMixer_Impl()
{
  Term();
}

void MotorMixer_Impl::Init(const std::string &pwm_device, const std::string &gpio_device)
{
  motor_driver_.Init(pwm_device, gpio_device);
}

void MotorMixer_Impl::Term()
{
  motor_driver_.Term();
}

void MotorMixer_Impl::Output(MotorMixer::Power &power)
{
  float l_out = power.pitch_ * mix_value_[Wheel::L].p_ + power.yaw_ * mix_value_[Wheel::L].y_;
  float r_out = power.pitch_ * mix_value_[Wheel::R].p_ + power.yaw_ * mix_value_[Wheel::R].y_;

  motor_driver_.Move(l_out, r_out);
}

void MotorMixer_Impl::Brake()
{
  motor_driver_.Brake();
}

/// EOF ****************************************************************************************************************
