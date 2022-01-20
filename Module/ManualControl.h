/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include <boost/signals2/signal.hpp>
#include "Misc/MiscTools.hpp"

/// ********************************************************************************************************************

class ManualControl_Impl;

/// ********************************************************************************************************************

class ManualControl
{
public:

  enum class Main
  {
    T,
    A,
    E,
    R,
    N
  };

  enum class Aux
  {
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    _8,
    N
  };

  struct RcIn
  {
    float main_[std::to_underlying(Main::N)];
    float aux_[std::to_underlying(Aux::N)];
  };

  typedef boost::signals2::signal <void(const RcIn&)> RcInSignal;
  typedef boost::signals2::slot   <void(const RcIn&)> RcInSlot;
  typedef boost::signals2::connection   RcInConnect;

  explicit ManualControl();
  ~ManualControl();

  bool  Init(const std::string &device);
  void  Term();

  RcInConnect RegSlotRcInUpdated(const RcInSlot &slot);

private:

  std::unique_ptr<ManualControl_Impl>   impl_;
};

/// EOF ****************************************************************************************************************
