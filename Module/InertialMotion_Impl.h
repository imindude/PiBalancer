/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <memory>
#include <mutex>
#include <boost/thread.hpp>
#include <boost/signals2/signal.hpp>
#include "Device/MpuSensor.h"
#include "Fusion/Fusion.h"
#include "InertialMotion.h"

/// ********************************************************************************************************************

class InertialMotion_Impl
{
public:

  explicit InertialMotion_Impl();
  ~InertialMotion_Impl();

  bool  Init(
      const std::string       &device,
      uint8_t                 address,
      InertialMotion::Vector  &gyro_bias,
      InertialMotion::Vector  &accel_bias
  );
  void  Term();

  InertialMotion::PoseConnect RegSlotPoseUpdated(const InertialMotion::PoseSlot &slot);

private:

  void  run();
  bool  refresh();

  std::unique_ptr<boost::thread>  motion_thread_;
  MpuSensor     mpu_sensor_;
  float         sample_period_  { 0.0f };
  FusionAhrs    fusion_ahrs_;
  FusionBias    fusion_bias_;
  FusionVector3 bias_gyro_  { 0.0f, 0.0f, 0.0f };
  FusionVector3 bias_accel_ { 0.0f, 0.0f, 0.0f };

  // int   bias_sample_count_    { 0 };
  // CalibrationStep calibration_step_ { CalibrationStep::NONE };
  // InertialMotion::MotionVector  bias_gyro_mean_vector_;
  // InertialMotion::MotionVector  bias_accel_mean_vector_;

  InertialMotion::PoseSignal  pose_signal_;
};

/// EOF ****************************************************************************************************************
