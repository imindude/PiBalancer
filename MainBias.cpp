/// ********************************************************************************************************************
/// @author   wuyong.yi@asoa.co.kr
/// ********************************************************************************************************************

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include "Misc/MiscTools.hpp"
#include "Misc/AppConfig.h"
#include "Device/MpuSensor.h"

/// ********************************************************************************************************************

#define RESET   "\e[0m"
#define RED     "\e[0;31m"
#define GREEN   "\e[0;32m"
#define CYAN    "\e[0;36m"

/// ********************************************************************************************************************

static void signal_handler(int signum)
{
  std::cerr << "Interrupt: " << signum << std::endl;
  exit(signum);
}

int main(int argc, char* argv[])
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGSEGV, signal_handler);
  std::signal(SIGTERM, signal_handler);

  _AC->Init();

  /// ------------------------------------------------------------------------------------------------------------------

  #define CALIBRATION_SAMPLES     2500

  float   gyro_mean[3]   = { 0.0f, 0.0f, 0.0f };
  float   accel_mean[3]  = { 0.0f, 0.0f, 0.0f };
  float   bias_scale = 1.0f / static_cast<float>(CALIBRATION_SAMPLES);
  MpuSensor motion_device;
  MpuSensor::Motions<float> motions;

  if (!motion_device.Init(_AC->ImuDeviceName(), _AC->ImuDeviceAddress()))
  {
    std::cerr << RED << "Motion sensor is not reaedy." << RESET << std::endl;
    exit(1);
  }

  std::cout << GREEN;
  for (int i = 3; i >= 0; i--)
  {
    std::cerr << "Calibration countdown ..... " << std::to_string(i) << "\r";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  std::cout << RESET << std::endl;

  /// ------------------------------------------------------------------------------------------------------------------

  std::cout << CYAN << "Calibration is starting. It takes 5 seconeds." << RESET << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000 / motion_device.RefreshRate()));

  for (int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    if (!motion_device.Read(motions))
    {
      std::cerr << RED << "Motion sensor is not correct, my guess." << std::to_string(i) << RESET << std::endl;
      exit(1);
    }

    gyro_mean[0]  += (motions.gyro_x_ * bias_scale);
    gyro_mean[1]  += (motions.gyro_y_ * bias_scale);
    gyro_mean[2]  += (motions.gyro_z_ * bias_scale);
    accel_mean[0] += (motions.accel_x_ * bias_scale);
    accel_mean[1] += (motions.accel_y_ * bias_scale);
    accel_mean[2] += (motions.accel_z_ * bias_scale);

    std::this_thread::sleep_for(std::chrono::microseconds(1900));
  }

  std::cout << ">> Result <<" << std::endl;
  std::cout.precision(10);
  std::cout << "Gyro X : " << gyro_mean[0] << std::endl;
  std::cout << "Gyro Y : " << gyro_mean[1] << std::endl;
  std::cout << "Gyro Z : " << gyro_mean[2] << std::endl;
  std::cout << "Accel X: " << accel_mean[0] << std::endl;
  std::cout << "Accel Y: " << accel_mean[1] << std::endl;
  std::cout << "Accel Z: " << accel_mean[2] << std::endl;

  /// ------------------------------------------------------------------------------------------------------------------

  return 0;
}

/// EOF ****************************************************************************************************************
