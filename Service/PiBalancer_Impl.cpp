/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include "PiBalancer_Impl.h"
#include "Misc/AppConfig.h"
#include "Misc/MiscTools.hpp"

/// ********************************************************************************************************************

PiBalancer_Impl::PiBalancer_Impl() : motion_lock_(motion_mutex_)
{
}

PiBalancer_Impl::~PiBalancer_Impl()
{
  Term();
}

void PiBalancer_Impl::Init()
{
  _AC->Init();

  initInertialMotion();
  initManualControl();
  initMotorMixer();
  initPidControl();

  balance_thread_ = std::make_unique<boost::thread>(boost::bind(&PiBalancer_Impl::run, this));

  pose_connection_ = motion_.RegSlotPoseUpdated(
      boost::bind(&PiBalancer_Impl::onSlotPoseUpdated, this, boost::placeholders::_1)
  );
  rcin_connection_ = mc_.RegSlotRcInUpdated(
      boost::bind(&PiBalancer_Impl::onSlotRcInUpdated, this, boost::placeholders::_1)
  );
}

void PiBalancer_Impl::Term()
{
  balance_thread_->join();
  balance_thread_.reset();

  motion_.Term();
  mc_.Term();
  motor_.Term();
}

void PiBalancer_Impl::run()
{
  int64_t   now_us;
  int64_t   exe_us = MiscTools::TimeSinceEpoch();
  float     dt;
  float     desired_tilt_angle  = 0.0f;
  float     desired_turn_rate   = 0.0f;
  float     actual_tilt_angle   = 0.0f;
  float     actual_tilt_rate    = 0.0f;
  float     actual_turn_rate    = 0.0f;
  bool      armed = false;
  bool      request_arm     = false;
  bool      request_disarm  = false;

  while (!balance_thread_->interruption_requested())
  {
    if (motion_condition_.wait_for(motion_lock_, std::chrono::milliseconds(kLoopDelay)) != std::cv_status::no_timeout)
    {
      std::cerr << "Motion Sensor is not work" << std::endl;

      stop();
      continue;
    }

    now_us = MiscTools::TimeSinceEpoch();
    dt = static_cast<float>(exe_us - now_us) / 1000000.0f;
    dt = MiscTools::LimitMax(dt, 1.0f);
    exe_us = now_us;

    {
      std::lock_guard<std::mutex> lock(update_mc_mutex_);

      desired_tilt_angle  = mc_rcin_.main_[std::to_underlying(ManualControl::Main::E)] * _AC->MaxTiltAngle();
      desired_turn_rate   = mc_rcin_.main_[std::to_underlying(ManualControl::Main::R)] * _AC->MaxTiltAngle();  // temporary

      request_arm =
          (mc_rcin_.aux_[std::to_underlying(ManualControl::Aux::_3)] != 0.0f) &&
          (mc_rcin_.aux_[std::to_underlying(ManualControl::Aux::_5)] != 0.0f);
      request_disarm =
          (mc_rcin_.aux_[std::to_underlying(ManualControl::Aux::_4)] != 0.0f) &&
          (mc_rcin_.aux_[std::to_underlying(ManualControl::Aux::_6)] != 0.0f);
    }

    if (request_disarm)   armed = false;
    else if (request_arm) armed = true;

    if (armed == false)
    {
      stop();
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(update_motion_mutex_);

      actual_tilt_angle = motion_pose_.pose_.p_;
      actual_tilt_rate  = motion_pose_.rate_.p_;
      actual_turn_rate  = motion_pose_.rate_.y_;
    }

    MotorMixer::Power mix_out;

    mix_out.pitch_  = tilt_angle_pid_.Update(desired_tilt_angle, actual_tilt_angle, actual_tilt_rate, dt);
    mix_out.yaw_    = turn_rate_pid_.Update(desired_turn_rate, actual_turn_rate, 0.0f, dt);

    motor_.Output(mix_out);
  }
}

void PiBalancer_Impl::initInertialMotion()
{
  std::vector<float>  stored_gyro_bias  = _AC->GyroBias();
  std::vector<float>  stored_accel_bias = _AC->AccelBias();
  InertialMotion::Vector  gyro_bias   { stored_gyro_bias[0], stored_gyro_bias[1], stored_gyro_bias[2] };
  InertialMotion::Vector  accel_bias  { stored_accel_bias[0], stored_accel_bias[1], stored_accel_bias[2] };

  motion_.Init(_AC->ImuDeviceName(), _AC->ImuDeviceAddress(), gyro_bias, accel_bias);
}

void PiBalancer_Impl::initManualControl()
{
  mc_.Init(_AC->InputDeviceName());
}

void PiBalancer_Impl::initMotorMixer()
{
  motor_.Init(_AC->PwmDeviceName(), _AC->GpioDeviceName());
}

void PiBalancer_Impl::initPidControl()
{
  std::vector<float>  tilt_a_pid = _AC->TiltAnglePid();
  std::vector<float>  turn_r_pid  = _AC->TurnRatePid();

  tilt_angle_pid_.Init(tilt_a_pid[0], tilt_a_pid[1], tilt_a_pid[2], tilt_a_pid[3]);
  turn_rate_pid_.Init(turn_r_pid[0], turn_r_pid[1], turn_r_pid[2], turn_r_pid[3]);
}

void PiBalancer_Impl::stop()
{
  motor_.Brake();
  tilt_angle_pid_.Reset();
  turn_rate_pid_.Reset();
}

void PiBalancer_Impl::onSlotPoseUpdated(const InertialMotion::Pose &pose)
{
  {
    std::lock_guard<std::mutex> lock(update_motion_mutex_);
    motion_pose_ = pose;
    // std::cout << pose.pose_.r_ << ":" << pose.pose_.p_ << ":" << pose.pose_.y_ << std::endl;
  }

  motion_condition_.notify_one();
}

void PiBalancer_Impl::onSlotRcInUpdated(const ManualControl::RcIn &rcin)
{
  std::lock_guard<std::mutex> lock(update_mc_mutex_);
  mc_rcin_ = rcin;
  // std::cout << "\e[0;36m"
  //           << "T[" << rcin.main_[std::to_underlying(ManualControl::Main::T)] << "] "
  //           << "A[" << rcin.main_[std::to_underlying(ManualControl::Main::A)] << "] "
  //           << "E[" << rcin.main_[std::to_underlying(ManualControl::Main::E)] << "] "
  //           << "R[" << rcin.main_[std::to_underlying(ManualControl::Main::R)] << "] "
  //           << "\e[0;32m"
  //           << "1[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_1)] << "] "
  //           << "2[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_2)] << "] "
  //           << "3[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_3)] << "] "
  //           << "4[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_4)] << "] "
  //           << "5[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_5)] << "] "
  //           << "6[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_6)] << "] "
  //           << "7[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_7)] << "] "
  //           << "8[" << rcin.aux_[std::to_underlying(ManualControl::Aux::_8)] << "]"
  //           << "\e[0m" << std::endl;
}

/// EOF ****************************************************************************************************************
