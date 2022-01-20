/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "ManualControl_Impl.h"
#include "Misc/MiscTools.hpp"

/// ********************************************************************************************************************

ManualControl_Impl::ManualControl_Impl()
{
}

ManualControl_Impl::~ManualControl_Impl()
{
  Term();
}

bool ManualControl_Impl::Init(const std::string &device)
{
  do
  {
    if (gamepad_input_.Init(device) == false)
      break;

    manual_thread_ = std::make_unique<boost::thread>(boost::bind(&ManualControl_Impl::run, this));

    return true;
  }
  while (false);

  return false;
}

void ManualControl_Impl::Term()
{
  if (manual_thread_)
  {
    manual_thread_->interrupt();
    manual_thread_->join();
    manual_thread_.reset();
  }

  gamepad_input_.Term();
}

ManualControl::RcInConnect ManualControl_Impl::RegSlotRcInUpdated(const ManualControl::RcInSlot &slot)
{
  return rcin_signal_.connect(slot);
}

void ManualControl_Impl::run()
{
  while (!manual_thread_->interruption_requested())
  {
    std::pair<GamepadInput::Key, bool>  rcin = gamepad_input_.Read();

    switch (rcin.first)
    {
    case GamepadInput::Key::AXIS_UP:
      rcin_value_.main_[std::to_underlying(ManualControl::Main::E)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::AXIS_DOWN:
      rcin_value_.main_[std::to_underlying(ManualControl::Main::E)] = rcin.second ? -1.0f : 0.0f;
      break;
    case GamepadInput::Key::AXIS_LEFT:
      rcin_value_.main_[std::to_underlying(ManualControl::Main::R)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::AXIS_RIGHT:
      rcin_value_.main_[std::to_underlying(ManualControl::Main::R)] = rcin.second ? -1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_X:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_3)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_Y:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_4)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_A:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_5)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_B:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_6)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_L:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_7)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_R:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_8)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_SELECT:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_1)] = rcin.second ? 1.0f : 0.0f;
      break;
    case GamepadInput::Key::BUTTON_START:
      rcin_value_.aux_[std::to_underlying(ManualControl::Aux::_2)] = rcin.second ? 1.0f : 0.0f;
      break;
    default:
      break;
    }

    rcin_signal_(rcin_value_);
  }
}

/// EOF ****************************************************************************************************************
