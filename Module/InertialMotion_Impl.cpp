/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include <boost/chrono.hpp>
#include "InertialMotion.h"
#include "InertialMotion_Impl.h"
#include "Fusion/Fusion.h"
#include "Misc/MiscTools.hpp"

/// ********************************************************************************************************************

#define FUSION_VECTOR3_BASE ((FusionVector3){ .array = {1.0f, 1.0f, 1.0f} })

/// ********************************************************************************************************************

InertialMotion_Impl::InertialMotion_Impl()
{
}

InertialMotion_Impl::~InertialMotion_Impl()
{
  Term();
}

bool InertialMotion_Impl::Init(
    const std::string       &device,
    uint8_t                 address,
    InertialMotion::Vector  &gyro_bias,
    InertialMotion::Vector  &accel_bias
)
{
  do
  {
    if (mpu_sensor_.Init(device, address) == false)
      break;

    sample_period_ = 1.0f / static_cast<float>(mpu_sensor_.RefreshRate());

    FusionAhrsInitialise(&fusion_ahrs_, InertialMotion::kFusionGain);
    FusionBiasInitialise(&fusion_bias_, InertialMotion::kFusionBiasThreshold, sample_period_);

    for (int i = 0; i < 3; i++)
    {
      bias_gyro_.array[i]   = gyro_bias.axis_[i];
      bias_accel_.array[i]  = accel_bias.axis_[i];
    }

    motion_thread_ = std::make_unique<boost::thread>(boost::bind(&InertialMotion_Impl::run, this));

    return true;
  }
  while (false);

  return false;
}

void InertialMotion_Impl::Term()
{
  if (motion_thread_)
  {
    motion_thread_->interrupt();
    motion_thread_->join();
    motion_thread_.reset();
  }

  mpu_sensor_.Term();
}

InertialMotion::PoseConnect InertialMotion_Impl::RegSlotPoseUpdated(const InertialMotion::PoseSlot &slot)
{
  return pose_signal_.connect(slot);
}

void InertialMotion_Impl::run()
{
  int64_t   start_us;
  int64_t   delay_us;

  while (!motion_thread_->interruption_requested())
  {
    start_us = MiscTools::TimeSinceEpoch();

    if (refresh())
      delay_us = 1000000 / mpu_sensor_.RefreshRate() - (MiscTools::TimeSinceEpoch() - start_us);
    else
      delay_us = 1000;

    boost::this_thread::sleep_for(boost::chrono::microseconds(delay_us));
  }
}

bool InertialMotion_Impl::refresh()
{
  MpuSensor::Motions<float> motions;

  if (mpu_sensor_.Read(motions))
  {
    FusionVector3 uncalibrated_gyro   = { motions.gyro_x_, motions.gyro_y_, motions.gyro_z_ };
    FusionVector3 uncalibrated_accel  = { motions.accel_x_, motions.accel_y_, motions.accel_z_ };
    FusionVector3 calibrated_gyro   = FusionCalibrationInertial(
        uncalibrated_gyro, FUSION_ROTATION_MATRIX_IDENTITY, FUSION_VECTOR3_BASE, bias_gyro_
    );
    FusionVector3 calibrated_accel  = FusionCalibrationInertial(
        uncalibrated_accel, FUSION_ROTATION_MATRIX_IDENTITY, FUSION_VECTOR3_BASE, bias_accel_
    );

    calibrated_gyro = FusionBiasUpdate(&fusion_bias_, calibrated_gyro);
    FusionAhrsUpdateWithoutMagnetometer(&fusion_ahrs_, calibrated_gyro, calibrated_accel, sample_period_);

    FusionVector3     linear_acceleration = FusionAhrsGetLinearAcceleration(&fusion_ahrs_);
    FusionVector3     earth_acceleration  = FusionAhrsGetEarthAcceleration(&fusion_ahrs_);
    FusionQuaternion  quaternion  = FusionAhrsGetQuaternion(&fusion_ahrs_);
    FusionEulerAngles euler_angle = FusionQuaternionToEulerAngles(quaternion);

    InertialMotion::Pose    motion_pose {
      .pose_          { euler_angle.angle.roll, euler_angle.angle.pitch, euler_angle.angle.yaw },
      .q_             { quaternion.element.w, quaternion.element.x, quaternion.element.y, quaternion.element.z },
      .rate_          { calibrated_gyro.axis.x, calibrated_gyro.axis.y, calibrated_gyro.axis.z },
      .linear_accel_  { linear_acceleration.axis.x, linear_acceleration.axis.y, linear_acceleration.axis.z },
      .earth_accel_   { earth_acceleration.axis.x, earth_acceleration.axis.y, earth_acceleration.axis.z }
    };

    pose_signal_(motion_pose);

    return true;
  }

  return false;
}

/// EOF ****************************************************************************************************************
