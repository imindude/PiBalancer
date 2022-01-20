/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "InertialMotion.h"
#include "InertialMotion_Impl.h"

/// ********************************************************************************************************************

InertialMotion::InertialMotion()
{
  impl_ = std::make_unique<InertialMotion_Impl>();
}

InertialMotion::~InertialMotion()
{
  impl_.reset();
}

bool InertialMotion::Init(const std::string &device, uint8_t address, Vector &gyro_bias, Vector &accel_bias)
{
  return impl_->Init(device, address, gyro_bias, accel_bias);
}

void InertialMotion::Term()
{
  impl_->Term();
}

InertialMotion::PoseConnect InertialMotion::RegSlotPoseUpdated(const PoseSlot &slot)
{
  return impl_->RegSlotPoseUpdated(slot);
}

/// EOF ****************************************************************************************************************
