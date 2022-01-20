/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "MotorMixer.h"
#include "MotorMixer_Impl.h"

/// ********************************************************************************************************************

MotorMixer::MotorMixer()
{
  impl_ = std::make_unique<MotorMixer_Impl>();
}

MotorMixer::~MotorMixer()
{
  impl_.reset();
}

void MotorMixer::Init(const std::string &pwm_device, const std::string &gpio_device)
{
  impl_->Init(pwm_device, gpio_device);
}

void MotorMixer::Term()
{
  impl_->Term();
}

void MotorMixer::Output(Power &power)
{
  impl_->Output(power);
}

void MotorMixer::Brake()
{
  impl_->Brake();
}

/// EOF ****************************************************************************************************************
