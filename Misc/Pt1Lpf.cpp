/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include <cmath>
#include "Pt1Lpf.h"

/// ********************************************************************************************************************

void Pt1Lpf::Init(int32_t cutoff_freq, int32_t sample_freq)
{
  float   dt = 1.0f / static_cast<float>(sample_freq);
  float   rc = 1.0f / (2.0f * M_PI * static_cast<float>(cutoff_freq));

  a_ = dt / (rc + dt);
  y_ = 0;
}

float Pt1Lpf::Update(float x)
{
  y_ += a_ * (x - y_);

  return y_;
}

/// EOF ****************************************************************************************************************