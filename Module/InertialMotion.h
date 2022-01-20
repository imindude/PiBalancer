/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <memory>
#include <boost/signals2/signal.hpp>

/// ********************************************************************************************************************

class InertialMotion_Impl;

/// ********************************************************************************************************************

class InertialMotion
{
public:

  union Euler
  {
    float axis_[3];

    struct
    {
      float r_;
      float p_;
      float y_;
    };
  };

  union Vector
  {
    float axis_[3];

    struct
    {
      float x_;
      float y_;
      float z_;
    };
  };

  struct Pose
  {
    Euler   pose_;
    float   q_[4];
    Euler   rate_;
    Vector  linear_accel_;
    Vector  earth_accel_;
  };

  typedef boost::signals2::signal <void(const Pose&)>   PoseSignal;
  typedef boost::signals2::slot   <void(const Pose&)>   PoseSlot;
  typedef boost::signals2::connection   PoseConnect;

  static constexpr auto kFusionGain           = 0.5f;
  static constexpr auto kFusionBiasThreshold  = 1.1f;

  explicit InertialMotion();
  ~InertialMotion();

  bool  Init(const std::string &device, uint8_t address, Vector &gyro_bias, Vector &accel_bias);
  void  Term();

  PoseConnect RegSlotPoseUpdated(const PoseSlot &slot);

private:

  std::unique_ptr<InertialMotion_Impl>  impl_;
};

/// EOF ****************************************************************************************************************
