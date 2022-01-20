/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include <boost/thread.hpp>
#include "Device/GamepadInput.h"
#include "ManualControl.h"
#include "ManualControl_Impl.h"

/// ********************************************************************************************************************

class ManualControl_Impl
{
public:

  explicit ManualControl_Impl();
  ~ManualControl_Impl();

  bool  Init(const std::string &device);
  void  Term();

  ManualControl::RcInConnect RegSlotRcInUpdated(const ManualControl::RcInSlot &slot);

private:

  void  run();

  std::unique_ptr<boost::thread>  manual_thread_;
  GamepadInput    gamepad_input_;
  ManualControl::RcIn         rcin_value_;
  ManualControl::RcInSignal   rcin_signal_;
};

/// EOF ****************************************************************************************************************
