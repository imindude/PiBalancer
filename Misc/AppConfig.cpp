/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include <fstream>
#include <json/json.h>
#include "AppConfig.h"
#include "version.h"

/// ********************************************************************************************************************

#define JSON_KEY_IMU_DEVICE_NAME          "IMU Device Name"
#define JSON_KEY_IMU_SLAVE_ADDRESS        "IMU Slave Address"
#define JSON_KEY_PWM_DEVICE_NAME          "PWM Device Name"
#define JSON_KEY_GPIO_DEVICE_NAME         "GPIO Device Name"
#define JSON_KEY_INPUT_DEVICE_NAME        "Input Device Name"
#define JSON_KEY_HOST_ADDRESS             "Host Address"
#define JSON_KEY_HOST_PORT                "Host Port"
#define JSON_KEY_MAX_TILT_ANGLE           "Max Tilt Angle"
#define JSON_KEY_GYRO_BIAS_X              "Gyro Bias X"
#define JSON_KEY_GYRO_BIAS_Y              "Gyro Bias Y"
#define JSON_KEY_GYRO_BIAS_Z              "Gyro Bias Z"
#define JSON_KEY_ACCEL_BIAS_X             "Accel Bias X"
#define JSON_KEY_ACCEL_BIAS_Y             "Accel Bias Y"
#define JSON_KEY_ACCEL_BIAS_Z             "Accel Bias Z"
#define JSON_KEY_TILT_ANGLE_PID_KP        "Tilt Angle Pid Kp"
#define JSON_KEY_TILT_ANGLE_PID_KI        "Tilt Angle Pid Ki"
#define JSON_KEY_TILT_ANGLE_PID_KD        "Tilt Angle Pid Kd"
#define JSON_KEY_TILT_ANGLE_PID_MAX_I     "Tilt Angle Pid Max i"
#define JSON_KEY_TURN_RATE_PID_KP         "Turn Rate Pid Kp"
#define JSON_KEY_TURN_RATE_PID_KI         "Turn Rate Pid Ki"
#define JSON_KEY_TURN_RATE_PID_KD         "Turn Rate Pid Kd"
#define JSON_KEY_TURN_RATE_PID_MAX_I      "Turn Rate Pid Max i"

/// ********************************************************************************************************************

void AppConfig::Init()
{
#ifdef NDEBUG
  std::ifstream   json_file("/etc/dude/PiBalancerConfig.json", std::ifstream::binary);
#else
  std::ifstream   json_file("PiBalancerConfig.json", std::ifstream::binary);
#endif

  if (json_file.is_open())
  {
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string error_string;

    if (Json::parseFromStream(builder, json_file, &root, &error_string))
    {
      imu_device_name_      = root.get(JSON_KEY_IMU_DEVICE_NAME, kImuDeviceName).asString();
      imu_device_address_   = root.get(JSON_KEY_IMU_SLAVE_ADDRESS, kImuSlaveAddress).asUInt();
      pwm_device_name_      = root.get(JSON_KEY_PWM_DEVICE_NAME, kPwmDeviceName).asString();
      gpio_device_name_     = root.get(JSON_KEY_GPIO_DEVICE_NAME, kGpioDeviceName).asString();
      input_device_name_    = root.get(JSON_KEY_INPUT_DEVICE_NAME, kInputDeviceName).asString();
      host_address_         = root.get(JSON_KEY_HOST_ADDRESS, kHostAddress).asString();
      host_port_            = root.get(JSON_KEY_HOST_PORT, kHostPort).asUInt();
      max_tilt_angle_       = root.get(JSON_KEY_MAX_TILT_ANGLE, kMaxTiltAngle).asFloat();
      gyro_bias_x_          = root.get(JSON_KEY_GYRO_BIAS_X, 0.0f).asFloat();
      gyro_bias_y_          = root.get(JSON_KEY_GYRO_BIAS_Y, 0.0f).asFloat();
      gyro_bias_z_          = root.get(JSON_KEY_GYRO_BIAS_Z, 0.0f).asFloat();
      accel_bias_x_         = root.get(JSON_KEY_ACCEL_BIAS_X, 0.0f).asFloat();
      accel_bias_y_         = root.get(JSON_KEY_ACCEL_BIAS_Y, 0.0f).asFloat();
      accel_bias_z_         = root.get(JSON_KEY_ACCEL_BIAS_Z, 0.0f).asFloat();
      tilt_angle_pid_kp_    = root.get(JSON_KEY_TILT_ANGLE_PID_KP, 0.0f).asFloat();
      tilt_angle_pid_ki_    = root.get(JSON_KEY_TILT_ANGLE_PID_KI, 0.0f).asFloat();
      tilt_angle_pid_kd_    = root.get(JSON_KEY_TILT_ANGLE_PID_KD, 0.0f).asFloat();
      tilt_angle_pid_max_i_ = root.get(JSON_KEY_TILT_ANGLE_PID_MAX_I, 0.0f).asFloat();
      turn_rate_pid_kp_     = root.get(JSON_KEY_TURN_RATE_PID_KP, 0.0f).asFloat();
      turn_rate_pid_ki_     = root.get(JSON_KEY_TURN_RATE_PID_KI, 0.0f).asFloat();
      turn_rate_pid_kd_     = root.get(JSON_KEY_TURN_RATE_PID_KD, 0.0f).asFloat();
      turn_rate_pid_max_i_  = root.get(JSON_KEY_TURN_RATE_PID_MAX_I, 0.0f).asFloat();
    }
    else
    {
      std::cerr << "Json parsing error. Use default settings." << std::endl;
    }
  }
  else
  {
    std::cerr << "Configuration file does not exist." << std::endl;
  }
}

std::string AppConfig::VersionString() const
{
  return std::string { VERSION };
}

std::string AppConfig::ImuDeviceName() const
{
  return imu_device_name_;
}

uint8_t AppConfig::ImuDeviceAddress() const
{
  return imu_device_address_;
}

std::string AppConfig::PwmDeviceName() const
{
  return pwm_device_name_;
}

std::string AppConfig::GpioDeviceName() const
{
  return gpio_device_name_;
}

std::string AppConfig::InputDeviceName() const
{
  return input_device_name_;
}

std::string AppConfig::HostAddress() const
{
  return host_address_;
}

uint16_t AppConfig::HostPort() const
{
  return host_port_;
}

float AppConfig::MaxTiltAngle() const
{
  return max_tilt_angle_;
}

std::vector<float> AppConfig::GyroBias() const
{
  return std::vector<float> { gyro_bias_x_, gyro_bias_y_, gyro_bias_z_ };
}

std::vector<float> AppConfig::AccelBias() const
{
  return std::vector<float> { accel_bias_x_, accel_bias_y_, accel_bias_z_ };
}

std::vector<float> AppConfig::TiltAnglePid() const
{
  return std::vector<float> { tilt_angle_pid_kp_, tilt_angle_pid_ki_, tilt_angle_pid_kd_, tilt_angle_pid_max_i_ };
}

std::vector<float> AppConfig::TurnRatePid() const
{
  return std::vector<float> { turn_rate_pid_kp_, turn_rate_pid_ki_, turn_rate_pid_kd_, turn_rate_pid_max_i_ };
}

AppConfig* AppConfig::GetInst()
{
  static AppConfig  app_config;
  return &app_config;
}

/// EOF ****************************************************************************************************************
