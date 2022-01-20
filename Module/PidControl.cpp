/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "PidControl.h"
#include "Misc/MiscTools.hpp"

/// ********************************************************************************************************************

void PidControl::Init(float kp, float ki, float kd, float max_i)
{
  kp_     = kp;
  ki_     = ki;
  kd_     = kd;
  max_i_  = max_i;
  sum_iterm_  = 0.0f;
  prev_rate_  = 0.0f;
}

void PidControl::Reset()
{
  sum_iterm_  = 0.0f;
  prev_rate_  = 0.0f;
}

float PidControl::Update(float desired, float actual, float rate, float dt)
{
  float   desired_rate = (rate != 0.0f) ? desired - actual : desired;
  float   error = desired_rate - rate;
  float   pterm = kp_ * error;
  float   iterm = ki_ * error * dt + sum_iterm_;
  float   dterm = kd_ * ((prev_rate_ - rate) * dt);

  iterm = MiscTools::Constrain(iterm, -1.0f * max_i_, max_i_);

  prev_rate_ = rate;
  sum_iterm_ = iterm;

  return pterm + iterm + dterm;
}

/// EOF ****************************************************************************************************************
