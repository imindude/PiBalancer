/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>

/// ********************************************************************************************************************

class Pt1Lpf
{
public:

  explicit Pt1Lpf() = default;
  ~Pt1Lpf() = default;

  void  Init(int32_t cutoff_freq, int32_t sample_freq);
  float Update(float x);

private:

  float   a_;
  float   y_;
};

/// EOF ****************************************************************************************************************
