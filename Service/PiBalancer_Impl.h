/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include <mutex>
#include <condition_variable>
#include <boost/thread.hpp>
#include "Module/InertialMotion.h"
#include "Module/ManualControl.h"
#include "Module/MotorMixer.h"
#include "Module/PidControl.h"

/// ********************************************************************************************************************

class PiBalancer_Impl
{
public:

  explicit PiBalancer_Impl();
  ~PiBalancer_Impl();

  void  Init();
  void  Term();

private:

  static constexpr auto kLoopDelay  = 500;

  void  run();
  void  initInertialMotion();
  void  initManualControl();
  void  initMotorMixer();
  void  initPidControl();
  void  stop();

  void  onSlotPoseUpdated(const InertialMotion::Pose &pose);
  void  onSlotRcInUpdated(const ManualControl::RcIn &rcin);

  InertialMotion  motion_;
  ManualControl   mc_;
  MotorMixer      motor_;
  PidControl      tilt_angle_pid_;
  PidControl      turn_rate_pid_;

  InertialMotion::Pose          motion_pose_;
  InertialMotion::PoseConnect   pose_connection_;
  ManualControl::RcIn           mc_rcin_;
  ManualControl::RcInConnect    rcin_connection_;

  mutable std::mutex  update_motion_mutex_  {};
  mutable std::mutex  update_mc_mutex_      {};
  std::mutex          motion_mutex_         {};
  std::unique_lock<std::mutex>    motion_lock_;
  std::condition_variable         motion_condition_;
  std::unique_ptr<boost::thread>  balance_thread_;
};

/// EOF ****************************************************************************************************************
