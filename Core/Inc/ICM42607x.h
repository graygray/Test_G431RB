#pragma once

#include "global.h"

#define DEBUG_ICM42607x
#define ICM42607x_USE_BIAS

#define AD0_HIGH
#if defined(AD0_HIGH)

#define ICM42607x_ADDR (0x69 << 1)  // ICM-42670-P I2C address when AD0 is high

#else

#define ICM42607x_ADDR (0x68 << 1)  // ICM-42670-P I2C address when AD0 is low

#endif  // AD0_HIGH

// ICM-42670-P register addresses
#define ICM42607x_WHO_AM_I              0x75
#define ICM42607x_SIGNAL_PATH_RESET     0x02
#define ICM42607x_PWR_MGMT0             0x1F
#define ICM42607x_TEMP_DATA1            0x09    // high byte
#define ICM42607x_TEMP_DATA0            0x0A    // low byte
#define ICM42607x_ACCEL_DATA_X1         0x0B    // high byte
#define ICM42607x_ACCEL_DATA_X0         0x0C    // low byte
#define ICM42607x_ACCEL_DATA_Y1         0x0D    // high byte
#define ICM42607x_ACCEL_DATA_Y0         0x0E    // low byte
#define ICM42607x_ACCEL_DATA_Z1         0x0F    // high byte
#define ICM42607x_ACCEL_DATA_Z0         0x10    // low byte
#define ICM42607x_GYRO_DATA_X1          0x11    // high byte
#define ICM42607x_GYRO_DATA_X0          0x12    // low byte
#define ICM42607x_GYRO_DATA_Y1          0x13    // high byte
#define ICM42607x_GYRO_DATA_Y0          0x14    // low byte
#define ICM42607x_GYRO_DATA_Z1          0x15    // high byte
#define ICM42607x_GYRO_DATA_Z0          0x16    // low byte
#define ICM42607x_GYRO_CONFIG0          0x20
#define ICM42607x_ACCEL_CONFIG0         0x21
#define ICM42607x_WOM_CONFIG            0x27    // wake on motion

// ICM-42670-P config
// GYRO
#define ICM42607x_CONFIG_GYRO_2k_DPS     (0b00000000)
#define ICM42607x_CONFIG_GYRO_1k_DPS     (0b00100000)
#define ICM42607x_CONFIG_GYRO_500_DPS    (0b01000000)
#define ICM42607x_CONFIG_GYRO_250_DPS    (0b01100000)
// ACCEL
#define ICM42607x_CONFIG_ACCEL_16_G      (0b00000000)
#define ICM42607x_CONFIG_ACCEL_8_G       (0b00100000)
#define ICM42607x_CONFIG_ACCEL_4_G       (0b01000000)
#define ICM42607x_CONFIG_ACCEL_2_G       (0b01100000)
// RATE
#define ICM42607x_CONFIG_RATE_1p6_kHz    (0b00000101)
#define ICM42607x_CONFIG_RATE_800_Hz     (0b00000110)
#define ICM42607x_CONFIG_RATE_400_Hz     (0b00000111)
#define ICM42607x_CONFIG_RATE_200_Hz     (0b00001000)
#define ICM42607x_CONFIG_RATE_100_Hz     (0b00001001)
#define ICM42607x_CONFIG_RATE_50_Hz      (0b00001010)
#define ICM42607x_CONFIG_RATE_25_Hz      (0b00001011)
#define ICM42607x_CONFIG_RATE_12p5_Hz    (0b00001100)

// 在四軸姿態計算中，我們通常要把角度換算成弧度。我們知道2Pi代表360度，那麼1度轉換成弧度就是：
// 2Pi/360=(2*3.1415926)/360=0.0174532=1/57.30。 等於徑度
// 1 / (32768/2000) / (360 / (PI * 2 )
// 1 /    16.384    /      57.3
// 1 /    8.192     /      57.3
// ± 2000 dps
#define ICM42607x_GYRO_RATIO            (1 / (32768.0f/2000.0f) / (360.0f / (PI * 2 )))     // 0.001065

// Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
// Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84
// 与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
// 加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
// ±16 g
#define ICM42607x_ACCEL_RATIO           (32768.0f / (16.0f * 9.8))      // 208.979592

// ??
#define SAMPLING_FREQ                   17.1715f // 采样频率

// calibration
#define NUM_SAMPLES     200
#define GRAVITY         16384   // Assuming the accelerometer output is scaled such that 1g = 16384 (for ±2g range)

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} SensorRawXYZ;

typedef struct {
  float x;
  float y;
  float z;
} SensorFloatXYZ;

extern __IO SensorRawXYZ icm42607x_accel;
extern __IO SensorRawXYZ icm42607x_gyro;
extern __IO float icm42607x_temperature;
extern __IO float icm42607x_q0, icm42607x_q1, icm42607x_q2, icm42607x_q3;

// regisdter operation
uint8_t ICM42607x_ReadReg(uint16_t reg);
void ICM42607x_ReadRegMultiple(uint16_t reg, uint8_t* readData, int16_t readDataLen);
void ICM42607x_WriteReg(uint16_t reg, uint8_t writeData);

void ICM42607x_Init();
void ICM42607x_WhoAmI();
void ICM42607x_ReadPWRMGMT0();
void ICM42607x_WritePWRMGMT0();
void ICM42607x_ReadTemperature();
void ICM42607x_ReadTemperatureMultiple();
void ICM42607x_ReadAccelConfig();
void ICM42607x_ConfigAccel();
void ICM42607x_ReadAccel();
void ICM42607x_ReadAccelMultiple();
void ICM42607x_CalibrateAccelBias();
void ICM42607x_ReadGyroConfig();
void ICM42607x_ConfigGyro();
void ICM42607x_ReadGyro();
void ICM42607x_ReadGyroMultiple();
void ICM42607x_CalibrateGyroBias();

void ICM42607x_ReadAccelGyro();

void ICM42607x_EnableSoftwareReset();
void ICM42607x_ReadWOM();
void ICM42607x_WriteWOM();
void ICM42607x_QuaternionSolution();