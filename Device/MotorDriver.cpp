/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <sys/stat.h>
#include <iostream>
#include "MotorDriver.h"

/// ********************************************************************************************************************

MotorDriver::~MotorDriver()
{
  Term();
}

void MotorDriver::Init(const std::string &pwm_device, const std::string &gpio_device)
{
  std::ofstream   attr_file;

  /// PWM - ready

  attr_file.open(pwm_device + "/export", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file << "1" << std::endl;
  attr_file.close();

  /// PWM - period (nano-sec)

  attr_file.open(pwm_device + "/pwm0/period", std::ios::out);
  attr_file << std::to_string(kPwmPeriodNs) << std::endl;
  attr_file.close();
  attr_file.open(pwm_device + "/pwm1/period", std::ios::out);
  attr_file << std::to_string(kPwmPeriodNs) << std::endl;
  attr_file.close();

  /// PWM - duty to zero

  attr_file.open(pwm_device + "/pwm0/duty_cycle", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(pwm_device + "/pwm1/duty_cycle", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();

  /// PWM - enable

  attr_file.open(pwm_device + "/pwm0/enable", std::ios::out);
  attr_file << "1" << std::endl;
  attr_file.close();
  attr_file.open(pwm_device + "/pwm1/enable", std::ios::out);
  attr_file << "1" << std::endl;
  attr_file.close();

  /// GPIO - ready

  attr_file.open(gpio_device + "/export", std::ios::out);
  attr_file << std::to_string(kGpioMotorIn1) << std::endl;
  attr_file << std::to_string(kGpioMotorIn2) << std::endl;
  attr_file << std::to_string(kGpioMotorIn3) << std::endl;
  attr_file << std::to_string(kGpioMotorIn4) << std::endl;
  attr_file.close();

  /// GPIO - output

  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn1) + "/direction", std::ios::out);
  attr_file << "out" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn2) + "/direction", std::ios::out);
  attr_file << "out" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn3) + "/direction", std::ios::out);
  attr_file << "out" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn4) + "/direction", std::ios::out);
  attr_file << "out" << std::endl;
  attr_file.close();

  /// GPIO - active high

  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn1) + "/active_low", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn2) + "/active_low", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn3) + "/active_low", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn1) + "/active_low", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();

  /// GPIO - reset to zero

  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn1) + "/value", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn2) + "/value", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn3) + "/value", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();
  attr_file.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn4) + "/value", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file.close();

  /// open file stream

  pwm0_duty_file_.open(pwm_device + "/pwm0/duty_cycle", std::ios::out);
  pwm1_duty_file_.open(pwm_device + "/pwm1/duty_cycle", std::ios::out);

  gpio0_value_file_.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn1) + "/value", std::ios::out);
  gpio1_value_file_.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn2) + "/value", std::ios::out);
  gpio2_value_file_.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn3) + "/value", std::ios::out);
  gpio3_value_file_.open(gpio_device + "/gpio" + std::to_string(kGpioMotorIn4) + "/value", std::ios::out);

  /// device path

  pwm_file_path_  = pwm_device;
  gpio_file_path_ = gpio_device;
}

void MotorDriver::Term()
{
  Brake();

  std::ofstream   attr_file;

  /// PWM - term

  pwm0_duty_file_.close();
  pwm1_duty_file_.close();

  attr_file.open(pwm_file_path_ + "/unexport", std::ios::out);
  attr_file << "0" << std::endl;
  attr_file << "1" << std::endl;
  attr_file.close();

  // GPIO - term

  gpio0_value_file_.close();
  gpio1_value_file_.close();
  gpio2_value_file_.close();
  gpio3_value_file_.close();

  attr_file.open(gpio_file_path_ + "/unexport", std::ios::out);
  attr_file << std::to_string(kGpioMotorIn1) << std::endl;
  attr_file << std::to_string(kGpioMotorIn2) << std::endl;
  attr_file << std::to_string(kGpioMotorIn3) << std::endl;
  attr_file << std::to_string(kGpioMotorIn4) << std::endl;
  attr_file.close();
}

void MotorDriver::Move(float left_power, float right_power)
{
  if (left_power > 0.0f)
  {
    gpioForward(Id::_0);
  }
  else
  {
    gpioBackward(Id::_0);
    left_power = -left_power;
  }

  if (right_power > 0.0f)
  {
    gpioForward(Id::_1);
  }
  else
  {
    gpioBackward(Id::_1);
    right_power = -right_power;
  }

  if (left_power > 1.0f)
    left_power = 1.0f;
  if (right_power > 1.0f)
    right_power = 1.0f;

  int32_t   lp_ns = static_cast<int32_t>(left_power * static_cast<float>(kPwmPeriodNs));
  int32_t   rp_ns = static_cast<int32_t>(right_power * static_cast<float>(kPwmPeriodNs));

  pwm0_duty_file_ << std::to_string(lp_ns) << std::endl;
  pwm1_duty_file_ << std::to_string(rp_ns) << std::endl;
}

void MotorDriver::Brake()
{
  gpioStop();

  pwm0_duty_file_ << "0" << std::endl;
  pwm1_duty_file_ << "0" << std::endl;
}

void MotorDriver::gpioForward(Id id)
{
  if (id == Id::_0)
  {
    gpio0_value_file_ << "0" << std::endl;
    gpio1_value_file_ << "1" << std::endl;
  }
  else
  {
    gpio2_value_file_ << "0" << std::endl;
    gpio3_value_file_ << "1" << std::endl;
  }
}

void MotorDriver::gpioBackward(Id id)
{
  if (id == Id::_0)
  {
    gpio0_value_file_ << "1" << std::endl;
    gpio1_value_file_ << "0" << std::endl;
  }
  else
  {
    gpio2_value_file_ << "1" << std::endl;
    gpio3_value_file_ << "0" << std::endl;
  }
}

void MotorDriver::gpioStop()
{
  gpio0_value_file_ << "0" << std::endl;
  gpio1_value_file_ << "0" << std::endl;

  gpio2_value_file_ << "0" << std::endl;
  gpio3_value_file_ << "0" << std::endl;
}

/// EOF ****************************************************************************************************************
