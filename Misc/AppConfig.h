/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include <vector>

/// ********************************************************************************************************************

class AppConfig
{
public:

  explicit AppConfig() {}
  ~AppConfig() = default;

  void        Init();
  std::string VersionString() const;
  std::string ImuDeviceName() const;
  uint8_t     ImuDeviceAddress() const;
  std::string PwmDeviceName() const;
  std::string GpioDeviceName() const;
  std::string InputDeviceName() const;
  std::string HostAddress() const;
  uint16_t    HostPort() const;
  float       MaxTiltAngle() const;
  std::vector<float>  GyroBias() const;
  std::vector<float>  AccelBias() const;
  std::vector<float>  TiltAnglePid() const;
  std::vector<float>  TurnRatePid() const;

  static AppConfig* GetInst();

private:

  static constexpr auto kImuDeviceName    = "/dev/i2c-1";
  static constexpr auto kImuSlaveAddress  = 0x68;   // 104
  static constexpr auto kPwmDeviceName    = "/sys/class/pwm/pwmchip0";
  static constexpr auto kGpioDeviceName   = "/sys/class/gpio";
  static constexpr auto kInputDeviceName  = "/dev/input/js0";
  static constexpr auto kHostAddress      = "192.168.5.10";
  static constexpr auto kHostPort         = 5300;
  static constexpr auto kMaxTiltAngle     = 10.0f;

  std::string imu_device_name_      { kImuDeviceName    };
  uint8_t     imu_device_address_   { kImuSlaveAddress  };
  std::string pwm_device_name_      { kPwmDeviceName    };
  std::string gpio_device_name_     { kGpioDeviceName   };
  std::string input_device_name_    { kInputDeviceName  };
  std::string host_address_         { kHostAddress  };
  uint16_t    host_port_            { kHostPort     };
  float       max_tilt_angle_       { kMaxTiltAngle };
  float       gyro_bias_x_          { 0.0f };
  float       gyro_bias_y_          { 0.0f };
  float       gyro_bias_z_          { 0.0f };
  float       accel_bias_x_         { 0.0f };
  float       accel_bias_y_         { 0.0f };
  float       accel_bias_z_         { 0.0f };
  float       tilt_angle_pid_kp_    { 0.0f };
  float       tilt_angle_pid_ki_    { 0.0f };
  float       tilt_angle_pid_kd_    { 0.0f };
  float       tilt_angle_pid_max_i_ { 0.0f };
  float       turn_rate_pid_kp_     { 0.0f };
  float       turn_rate_pid_ki_     { 0.0f };
  float       turn_rate_pid_kd_     { 0.0f };
  float       turn_rate_pid_max_i_  { 0.0f };
};

/// ********************************************************************************************************************

#define _AC   AppConfig::GetInst()

/// EOF ****************************************************************************************************************
