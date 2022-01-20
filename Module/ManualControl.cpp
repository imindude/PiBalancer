/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "ManualControl.h"
#include "ManualControl_Impl.h"

/// ********************************************************************************************************************

ManualControl::ManualControl()
{
  impl_ = std::make_unique<ManualControl_Impl>();
}

ManualControl::~ManualControl()
{
  impl_.reset();
}

bool ManualControl::Init(const std::string &device)
{
  return impl_->Init(device);
}

void ManualControl::Term()
{
  impl_->Term();
}

ManualControl::RcInConnect ManualControl::RegSlotRcInUpdated(const ManualControl::RcInSlot &slot)
{
  return impl_->RegSlotRcInUpdated(slot);
}

/// EOF ****************************************************************************************************************
