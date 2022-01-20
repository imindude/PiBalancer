/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include <fstream>

/// ********************************************************************************************************************

class MotorDriver
{
public:

  /// PWM Configuration
  /// https://github.com/raspberrypi/linux/blob/04c8e47067d4873c584395e5cb260b4f170a99ea/arch/arm/boot/dts/overlays/README#L944
  /// GPIO12(Pin32): PWM0, GPIO13(Pin33): PWM1
  /// in /boot/config.txt, "dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4"
  ///
  /// How To Use
  /// root@rpi3:/sys/class/pwm/pwmchip0# echo 0 > export
  /// root@rpi3:/sys/class/pwm/pwmchip0# echo 10000000 > pwm0/period
  /// root@rpi3:/sys/class/pwm/pwmchip0# echo 8000000 > pwm0/duty_cycle
  /// root@rpi3:/sys/class/pwm/pwmchip0# echo 1 > pwm0/enable
  ///
  ///
  /// GPIO Configuration
  /// GPIO16(Pin36), GPIO20(Pin38) / GPIO19(Pin35), GPIO26(Pin37)
  ///
  /// How To Use
  /// # echo 16 > /sys/class/gpio/export
  /// # echo 20 > /sys/class/gpio/export
  /// # echo 19 > /sys/class/gpio/export
  /// # echo 26 > /sys/class/gpio/export
  /// # echo out > /sys/class/gpio/gpio16/direction
  /// ...
  /// # echo 0 > /sys/class/gpio/gpio16/active_low
  /// ...
  /// # echo 1 > /sys/class/gpio/gpio16/value
  /// ...

  explicit MotorDriver() = default;
  ~MotorDriver();

  void  Init(const std::string &pwm_device, const std::string &gpio_device);
  void  Term();
  void  Move(float left_power, float right_power);
  void  Brake();

private:

  enum class Id
  {
    _0,
    _1
  };

  static constexpr auto kPwmPeriodNs = 40000;   // 1s/25Khz = 1,000,000,000ns/25,000Hz = 40,000ns/Hz

  static constexpr auto kGpioMotorIn1 = 16;
  static constexpr auto kGpioMotorIn2 = 20;
  static constexpr auto kGpioMotorIn3 = 19;
  static constexpr auto kGpioMotorIn4 = 26;

  void  gpioForward(Id id);
  void  gpioBackward(Id id);
  void  gpioStop();

  std::string     pwm_file_path_;
  std::string     gpio_file_path_;

  std::ofstream   pwm0_duty_file_;
  std::ofstream   pwm1_duty_file_;

  std::ofstream   gpio0_value_file_;
  std::ofstream   gpio1_value_file_;
  std::ofstream   gpio2_value_file_;
  std::ofstream   gpio3_value_file_;
};

/// EOF ****************************************************************************************************************
