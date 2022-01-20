/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "MpuSensor.h"

/// ********************************************************************************************************************

MpuSensor::MpuSensor()
{
  gyro_scale_factor_  = kMaxGyroDps / 32768.0f;
  accel_scale_factor_ = kMaxAccelG / 32768.0f;
}

MpuSensor::~MpuSensor()
{
  Term();
}

bool MpuSensor::Init(const std::string &device, uint8_t address)
{
  do
  {
    if ((i2c_fd_ = open(device.c_str(), O_RDWR)) < 0)
      break;

    if (ioctl(i2c_fd_, I2C_SLAVE, address) < 0)
    {
      close(i2c_fd_);
      break;
    }

    return init();
  }
  while (false);

  i2c_fd_ = 0;

  return false;
}

void MpuSensor::Term()
{
  if (i2c_fd_ > 0)
    close(i2c_fd_);
  i2c_fd_ = 0;
}

int MpuSensor::RefreshRate() const
{
  return kUpdateRate;
}

bool MpuSensor::Read(MpuSensor::Motions<float> &motions)
{
  do
  {
    MpuSensor::Motions<int16_t> raw_motions;

    if (!ReadRaw(raw_motions))
      break;

    motions.accel_x_  = static_cast<float>(raw_motions.accel_x_) * accel_scale_factor_;
    motions.accel_y_  = static_cast<float>(raw_motions.accel_y_) * accel_scale_factor_;
    motions.accel_z_  = static_cast<float>(raw_motions.accel_z_) * accel_scale_factor_;
    motions.gyro_x_   = static_cast<float>(raw_motions.gyro_x_) * gyro_scale_factor_;
    motions.gyro_y_   = static_cast<float>(raw_motions.gyro_y_) * gyro_scale_factor_;
    motions.gyro_z_   = static_cast<float>(raw_motions.gyro_z_) * gyro_scale_factor_;

    return true;
  }
  while (false);

  return false;
}

bool MpuSensor::ReadRaw(MpuSensor::Motions<int16_t> &motions)
{
  do
  {
    uint8_t   buffer[BURST_READ_LENGTH] = { static_cast<uint8_t>(MPUREG_RD(Regs::BURST_READ_POS)) };

    if (write(i2c_fd_, buffer, 1) != 1)
      break;
    if (read(i2c_fd_, buffer, BURST_READ_LENGTH) != BURST_READ_LENGTH)
      break;

    if ((buffer[0] & INT_STATUS_RAW_DATA_RDY_INT) != INT_STATUS_RAW_DATA_RDY_INT)
      break;

    /// MPU9250 direction
    ///     +y      z_up +
    ///  -x    +x
    ///     -y      z_dn -
    ///
    /// Convert to NWU
    ///     +x       z_up +
    ///  +y    -y
    ///     -x       z_dn -
    ///

    motions.accel_y_  = static_cast<int16_t>(buffer[ 1] << 8) | static_cast<int16_t>(buffer[ 2]);
    motions.accel_x_  = static_cast<int16_t>(buffer[ 3] << 8) | static_cast<int16_t>(buffer[ 4]);
    motions.accel_z_  = static_cast<int16_t>(buffer[ 5] << 8) | static_cast<int16_t>(buffer[ 6]);
    motions.gyro_y_   = static_cast<int16_t>(buffer[ 9] << 8) | static_cast<int16_t>(buffer[10]);
    motions.gyro_x_   = static_cast<int16_t>(buffer[11] << 8) | static_cast<int16_t>(buffer[12]);
    motions.gyro_z_   = static_cast<int16_t>(buffer[13] << 8) | static_cast<int16_t>(buffer[14]);

    motions.accel_y_  *= -1;
    motions.gyro_y_   *= -1;

    return true;
  }
  while (false);

  return false;
}

bool MpuSensor::init()
{
  writeReg(Regs::PWR_MGMT_1, PWR_MGMT_1_H_RESET);

  do
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  while (readReg(Regs::PWR_MGMT_1) & PWR_MGMT_1_H_RESET);

  std::this_thread::sleep_for(std::chrono::milliseconds(3));

  uint8_t whoami = readReg(Regs::WHO_AM_I);

  if ((whoami != MPU9250_WHOAMI) && (whoami != MPU9255_WHOAMI))
    return false;

  writeReg(Regs::USER_CTRL, USER_CTRL_FIFO_RST | USER_CTRL_I2C_MST_RST | USER_CTRL_SIG_COND_RST);

  do
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  while (readReg(Regs::USER_CTRL) & (USER_CTRL_FIFO_RST | USER_CTRL_I2C_MST_RST | USER_CTRL_SIG_COND_RST));

  writeReg(Regs::PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::PWR_MGMT_2, PWR_MGMT_2_GYRO_ACCEL_EN);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_GYRO | SIGNAL_PATH_RESET_ACCEL | SIGNAL_PATH_RESET_TEMP);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  writeReg(Regs::SMPLRT_DIV, SMPLRT_DIV_2);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  writeReg(Regs::CONFIG, CONFIG_DLPF_184HZ_2900US);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::GYRO_CONFIG, GYRO_CONFIG_FS_SEL_1000DPS | GYRO_CONFIG_FCHOICE_B_1K);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::ACCEL_CONFIG, ACCEL_CONFIG_FS_SEL_2G);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::ACCEL_CONFIG2, ACCEL_CONFIG2_FCHOICE_B_1K | ACCEL_CONFIG2_DLPF_218HZ_1880US);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::INT_PIN_CFG, INT_PIN_CFG_LATCH_INT_EN | INT_PIN_CFG_ANYRD_2CLEAR);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  writeReg(Regs::INT_ENABLE, INT_ENABLE_RAW_DRY_EN);

  return true;
}

void MpuSensor::writeReg(Regs reg, uint8_t value)
{
  uint8_t   stream[2] = { static_cast<uint8_t>(MPUREG_WR(reg)), value };

  write(i2c_fd_, stream, 2);
}

uint8_t MpuSensor::readReg(Regs reg)
{
  uint8_t   stream[2] = { static_cast<uint8_t>(MPUREG_RD(reg)), 0 };

  write(i2c_fd_, stream, 1);
  read(i2c_fd_, stream + 1, 1);

  return stream[1];
}

/// EOF ****************************************************************************************************************
