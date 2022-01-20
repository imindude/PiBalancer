/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>

/// ********************************************************************************************************************

#define MPU925x_READ_LEN              14      // AXH|AXL|AYH|AYL|AZH|AZL|TH|TL|GXH|GXL|GYH|GYL|GZH|GZL
#define MPU9250_WHOAMI                0x71
#define MPU9255_WHOAMI                0x73
#define MPU925x_ZERO_TOLERANCE_DPS    15      // MPU9255's gyroscope ZERO tolerance in 25°C is ±5

/**
 * MPU9250 register command
 */

#define MPUREG_RD(r)        ((int)r | 0x80)
#define MPUREG_WR(r)        (r)

#define PWR_MGMT_1_H_RESET              (1 << 7)
#define PWR_MGMT_1_CLKSEL_AUTO          3

#define PWR_MGMT_2_GYRO_ACCEL_EN        0

#define SIGNAL_PATH_RESET_GYRO          (1 << 2)
#define SIGNAL_PATH_RESET_ACCEL         (1 << 1)
#define SIGNAL_PATH_RESET_TEMP          (1 << 0)

#define USER_CTRL_I2C_MST_EN            (1 << 5)
#define USER_CTRL_I2C_IF_DIS            (1 << 4)
#define USER_CTRL_FIFO_RST              (1 << 2)
#define USER_CTRL_I2C_MST_RST           (1 << 1)
#define USER_CTRL_SIG_COND_RST          (1 << 0)

#define SMPLRT_DIV_1                    0
#define SMPLRT_DIV_2                    1
#define SMPLRT_DIV_4                    3
#define SMPLRT_DIV_10                   9

#define CONFIG_DLPF_250HZ_970US         0   // 8K
#define CONFIG_DLPF_184HZ_2900US        1   // 1K
#define CONFIG_DLPF_3600HZ_170US        7   // 32K

#define GYRO_CONFIG_FS_SEL_1000DPS      (0b10 << 3)
#define GYRO_CONFIG_FS_SEL_2000DPS      (0b11 << 3)
#define GYRO_CONFIG_FCHOICE_B_1K        0b00
#define GYRO_CONFIG_FCHOICE_B_8K        0b00

#define ACCEL_CONFIG_FS_SEL_2G          (0b00 << 3)
#define ACCEL_CONFIG_FS_SEL_4G          (0b01 << 3)
#define ACCEL_CONFIG_FS_SEL_8G          (0b10 << 3)

#define ACCEL_CONFIG2_FCHOICE_B_1K      (0 << 3)
#define ACCEL_CONFIG2_FCHOICE_B_4K      (1 << 3)
#define ACCEL_CONFIG2_DLPF_1046HZ_503US 0   // 4K
#define ACCEL_CONFIG2_DLPF_218HZ_1880US 1   // 1K

#define INT_PIN_CFG_LATCH_INT_EN        (1 << 5)
#define INT_PIN_CFG_ANYRD_2CLEAR        (1 << 4)

#define INT_STATUS_RAW_DATA_RDY_INT     (1 << 0)

#define INT_ENABLE_RAW_DRY_EN           (1 << 0)

#define I2C_MST_CTRL_400KHZ             13
#define I2C_MST_CTRL_500KHZ             9

#define I2C_SLVx_EN                     (1 << 7)
#define I2C_SLVx_LEN(x)                 ((x) & 0x0F)

#define I2C_SLV0_DLY_EN                 (1 << 0)

#define BURST_READ_LENGTH               15

/// ********************************************************************************************************************

class MpuSensor
{
public:

  template<typename T>
  struct Motions
  {
    T gyro_x_;
    T gyro_y_;
    T gyro_z_;
    T accel_x_;
    T accel_y_;
    T accel_z_;
  };

  explicit MpuSensor();
  ~MpuSensor();

  bool  Init(const std::string &device, uint8_t address);
  void  Term();
  float GyroFactor() const;
  float AccelFactor() const;
  float Accel1G() const;
  int   RefreshRate() const;
  bool  Read(Motions<float> &motions);
  bool  ReadRaw(Motions<int16_t> &motions);

private:

  enum class Regs
  {
    SELF_TEST_X_GYRO   = 0x00,
    SELF_TEST_Y_GYRO   = 0x01,
    SELF_TEST_Z_GYRO   = 0x02,
    SELF_TEST_X_ACCEL  = 0x0D,
    SELF_TEST_Y_ACCEL  = 0x0E,
    SELF_TEST_Z_ACCEL  = 0x0F,
    XG_OFFSET_H        = 0x13,
    XG_OFFSET_L        = 0x14,
    YG_OFFSET_H        = 0x15,
    YG_OFFSET_L        = 0x16,
    ZG_OFFSET_H        = 0x17,
    ZG_OFFSET_L        = 0x18,
    SMPLRT_DIV         = 0x19,
    CONFIG             = 0x1A,
    GYRO_CONFIG        = 0x1B,
    ACCEL_CONFIG       = 0x1C,
    ACCEL_CONFIG2      = 0x1D,
    LP_ACCEL_ODR       = 0x1E,
    WOM_THR            = 0x1F,
    FIFO_EN            = 0x23,
    I2C_MST_CTRL       = 0x24,
    I2C_SLV0_ADDR      = 0x25,
    I2C_SLV0_REG       = 0x26,
    I2C_SLV0_CTRL      = 0x27,
    I2C_SLV1_ADDR      = 0x28,
    I2C_SLV1_REG       = 0x29,
    I2C_SLV1_CTRL      = 0x2A,
    I2C_SLV2_ADDR      = 0x2B,
    I2C_SLV2_REG       = 0x2C,
    I2C_SLV2_CTRL      = 0x2D,
    I2C_SLV3_ADDR      = 0x2E,
    I2C_SLV3_REG       = 0x2F,
    I2C_SLV3_CTRL      = 0x30,
    I2C_SLV4_ADDR      = 0x31,
    I2C_SLV4_REG       = 0x32,
    I2C_SLV4_DO        = 0x33,
    I2C_SLV4_CTRL      = 0x34,
    I2C_SLV4_DI        = 0x35,
    I2C_MST_STATUS     = 0x36,
    INT_PIN_CFG        = 0x37,
    INT_ENABLE         = 0x38,
    INT_STATUS         = 0x3A,
    ACCEL_XOUT_H       = 0x3B,
    ACCEL_XOUT_L       = 0x3C,
    ACCEL_YOUT_H       = 0x3D,
    ACCEL_YOUT_L       = 0x3E,
    ACCEL_ZOUT_H       = 0x3F,
    ACCEL_ZOUT_L       = 0x40,
    TEMP_OUT_H         = 0x41,
    TEMP_OUT_L         = 0x42,
    GYRO_XOUT_H        = 0x43,
    GYRO_XOUT_L        = 0x44,
    GYRO_YOUT_H        = 0x45,
    GYRO_YOUT_L        = 0x46,
    GYRO_ZOUT_H        = 0x47,
    GYRO_ZOUT_L        = 0x48,
    EXT_SENS_DATA_00   = 0x49,
    EXT_SENS_DATA_01   = 0x4A,
    EXT_SENS_DATA_02   = 0x4B,
    EXT_SENS_DATA_03   = 0x4C,
    EXT_SENS_DATA_04   = 0x4D,
    EXT_SENS_DATA_05   = 0x4E,
    EXT_SENS_DATA_06   = 0x4F,
    EXT_SENS_DATA_07   = 0x50,
    EXT_SENS_DATA_08   = 0x51,
    EXT_SENS_DATA_09   = 0x52,
    EXT_SENS_DATA_10   = 0x53,
    EXT_SENS_DATA_11   = 0x54,
    EXT_SENS_DATA_12   = 0x55,
    EXT_SENS_DATA_13   = 0x56,
    EXT_SENS_DATA_14   = 0x57,
    EXT_SENS_DATA_15   = 0x58,
    EXT_SENS_DATA_16   = 0x59,
    EXT_SENS_DATA_17   = 0x5A,
    EXT_SENS_DATA_18   = 0x5B,
    EXT_SENS_DATA_19   = 0x5C,
    EXT_SENS_DATA_20   = 0x5D,
    EXT_SENS_DATA_21   = 0x5E,
    EXT_SENS_DATA_22   = 0x5F,
    EXT_SENS_DATA_23   = 0x60,
    I2C_SLV0_DO        = 0x63,
    I2C_SLV1_DO        = 0x64,
    I2C_SLV2_DO        = 0x65,
    I2C_SLV3_DO        = 0x66,
    I2C_MST_DELAY_CTRL = 0x67,
    SIGNAL_PATH_RESET  = 0x68,
    MOT_DETECT_CTRL    = 0x69,
    USER_CTRL          = 0x6A,
    PWR_MGMT_1         = 0x6B,
    PWR_MGMT_2         = 0x6C,
    FIFO_COUNTH        = 0x72,
    FIFO_COUNTL        = 0x73,
    FIFO_R_W           = 0x74,
    WHO_AM_I           = 0x75,
    XA_OFFSET_H        = 0x77,
    XA_OFFSET_L        = 0x78,
    YA_OFFSET_H        = 0x7A,
    YA_OFFSET_L        = 0x7B,
    ZA_OFFSET_H        = 0x7D,
    ZA_OFFSET_L        = 0x7E,

    BURST_READ_POS = INT_STATUS,
  };

  static constexpr auto kMaxGyroDps = 1000.0f;  // ±1000DPS
  static constexpr auto kMaxAccelG  = 2.0f;     // ±2G
  static constexpr auto kUpdateRate = 500;      // 500Hz

  bool    init();
  void    writeReg(Regs reg, uint8_t value);
  uint8_t readReg(Regs reg);

  int   i2c_fd_ { 0 };
  float gyro_scale_factor_  { 0.0f };
  float accel_scale_factor_ { 0.0f };
};

/// EOF ****************************************************************************************************************
