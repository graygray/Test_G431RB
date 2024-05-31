#include <ICM42670P.h>

__IO bool isHALError = false;
__IO SensorRawXYZ icm42760p_accel = { 0 };        // default ± 16 g
__IO SensorRawXYZ icm42760p_gyro = { 0 };         // default ± 2000 dps
__IO SensorRawXYZ icm42760p_accelBias = { 0 }; 
__IO SensorRawXYZ icm42760p_gyroBias = { 0 }; 
__IO SensorFloatXYZ icm42760p_accels = { 0 };
__IO SensorFloatXYZ icm42760p_gyros = { 0 };
__IO float icm42760p_temperature = 0;

// sensitivity
uint16_t accelSensitivity = 2048;               // default
float gyroSensitivity = 16.4;                   // default

// quaternion
volatile float twoKp = 1.0f;                                                                        // 2 * proportional gain (Kp)
volatile float twoKi = 0.0f;                                                                        // 2 * integral gain (Ki)
volatile float icm42760p_q0 = 1.0f, icm42760p_q1 = 0.0f, icm42760p_q2 = 0.0f, icm42760p_q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;                          // integral error terms scaled by Ki

void resetI2C() {
    HAL_I2C_DeInit(&hi2c1);
    HAL_I2C_Init(&hi2c1);
}

uint8_t ICM42670P_ReadReg(uint16_t reg) {
  uint8_t data = 0;
  ICM42670P_ReadRegMultiple(reg, &data, 1);
  return data;
}

void ICM42670P_ReadRegMultiple(uint16_t reg, uint8_t* readData, int16_t readDataLen) {
  isHALError = false;
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, ICM42670P_ADDR, reg, I2C_MEMADD_SIZE_8BIT, readData, readDataLen, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    isHALError = true;
    xlog("%s:%d, HAL_I2C_Mem_Read error! status:%d \n\r", __func__, __LINE__, status);
    resetI2C();
  } 
}

void ICM42670P_WriteReg(uint16_t reg, uint8_t writeData) {
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, ICM42670P_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &writeData, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    xlog("%s:%d, HAL_I2C_Mem_Write error! status:%d \n\r", __func__, __LINE__, status);
    resetI2C();
  }
}

void ICM42670P_Init() {
  ICM42670P_WhoAmI();
  ICM42670P_ConfigAccel();
  ICM42670P_ConfigGyro();
  ICM42670P_WritePWRMGMT0();

#if defined(ICM42670P_USE_BIAS)
  ICM42670P_CalibrateAccelBias();
  ICM42670P_CalibrateGyroBias();
#endif  // ICM42670P_USE_BIAS
}

void ICM42670P_WhoAmI() {
  uint8_t data = ICM42670P_ReadReg(ICM42670P_WHO_AM_I);
  if (data != 0x67) {  // WHO_AM_I should return 0x67
    xlog("%s:%d, ICM42670P_WHO_AM_I:0x%x \n\r", __func__, __LINE__, data);
  }
}

void ICM42670P_ReadPWRMGMT0() {
  uint8_t data = ICM42670P_ReadReg(ICM42670P_PWR_MGMT0);
  xlog("%s:%d, ICM42670P_PWR_MGMT0:%d \n\r", __func__, __LINE__, data);
}

void ICM42670P_WritePWRMGMT0() {
  uint8_t data = 0b00001111;
  ICM42670P_WriteReg(ICM42670P_PWR_MGMT0, data);
}

void ICM42670P_ReadTemperature() {
  uint8_t data[2] = { 0 };
  uint16_t temperature = 0;

  data[0] = ICM42670P_ReadReg(ICM42670P_TEMP_DATA0);
  data[1] = ICM42670P_ReadReg(ICM42670P_TEMP_DATA1);

  temperature = ((data[1] << 8) | data[0]);
  icm42760p_temperature = (temperature / 128.0) + 25;
#if defined(ICM42670P_DEBUG)
  xlog("%s:%d, temperature:%d, degree:%.2f \n\r", __func__, __LINE__, temperature, icm42760p_temperature);
#endif  // ICM42670P_DEBUG
}

void ICM42670P_ReadTemperatureMultiple() {
  uint8_t data[2] = { 0 };
  uint16_t temperature = 0;

  ICM42670P_ReadRegMultiple(ICM42670P_TEMP_DATA1, data, 2);
  if (isHALError) {
    return;
  }

  temperature = ((data[0] << 8) | data[1]);
  icm42760p_temperature = (temperature / 128.0) + 25;
#if defined(ICM42670P_DEBUG)
  xlog("%s:%d, temperature:%d, degree:%.2f \n\r", __func__, __LINE__, temperature, icm42760p_temperature);
#endif  // ICM42670P_DEBUG
}

void ICM42670P_ReadAccelConfig() {
  uint8_t data = ICM42670P_ReadReg(ICM42670P_ACCEL_CONFIG0);
  xlog("%s:%d ICM42670P_ACCEL_CONFIG0:%d\n\r", __func__, __LINE__, data);
}

void ICM42670P_ConfigAccel() {
  uint8_t scale = ICM42670P_CONFIG_ACCEL_16_G;
  uint8_t odr = ICM42670P_CONFIG_RATE_25_Hz;
  uint8_t config = scale | odr;
  ICM42670P_WriteReg(ICM42670P_ACCEL_CONFIG0, config);

  switch (scale) {
    case ICM42670P_CONFIG_ACCEL_16_G:
      accelSensitivity = 2048;
      break;
    case ICM42670P_CONFIG_ACCEL_8_G:
      accelSensitivity = 4096;
      break;
    case ICM42670P_CONFIG_ACCEL_4_G:
      accelSensitivity = 8192;
      break;
    case ICM42670P_CONFIG_ACCEL_2_G:
      accelSensitivity = 16384;
      break;
    default:
      break;
  }
}

void ICM42670P_ReadAccel() {
  uint8_t data[2] = { 0 };

  data[0] = ICM42670P_ReadReg(ICM42670P_ACCEL_DATA_X0);
  data[1] = ICM42670P_ReadReg(ICM42670P_ACCEL_DATA_X1);
  icm42760p_accel.x = ((data[1] << 8) | data[0]);

  data[0] = ICM42670P_ReadReg(ICM42670P_ACCEL_DATA_Y0);
  data[1] = ICM42670P_ReadReg(ICM42670P_ACCEL_DATA_Y1);
  icm42760p_accel.y = ((data[1] << 8) | data[0]);

  data[0] = ICM42670P_ReadReg(ICM42670P_ACCEL_DATA_Z0);
  data[1] = ICM42670P_ReadReg(ICM42670P_ACCEL_DATA_Z1);
  icm42760p_accel.z = ((data[1] << 8) | data[0]);

#if defined(ICM42670P_DEBUG)
  xlog("Accel [%05d:%05d:%05d] \n\r",
       icm42760p_accel.x, icm42760p_accel.y, icm42760p_accel.z);
#endif  // ICM42670P_DEBUG
}

void ICM42670P_ReadAccelMultiple() {
  uint8_t data[6] = {0};

  ICM42670P_ReadRegMultiple(ICM42670P_ACCEL_DATA_X1, data, 6);
  if (isHALError) {
    return;
  }

  icm42760p_accel.x = ((data[0] << 8) | data[1]);
  icm42760p_accel.y = ((data[2] << 8) | data[3]);
  icm42760p_accel.z = ((data[4] << 8) | data[5]);

#if defined(ICM42670P_DEBUG)
  xlog("Accel [%05d:%05d:%05d] \n\r",
       icm42760p_accel.x, icm42760p_accel.y, icm42760p_accel.z);
#endif  // ICM42670P_DEBUG
}

void ICM42670P_CalibrateAccelBias() {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ICM42670P_ReadAccelMultiple();
        sum_x += (int16_t)icm42760p_accel.x;
        sum_y += (int16_t)icm42760p_accel.y;
        sum_z += (int16_t)icm42760p_accel.z;
        HAL_Delay(1);
    }

    xlog("[%ld:%ld:%ld] \n\r", sum_x, sum_y, sum_z);
    icm42760p_accelBias.x = (int16_t)(sum_x / (float)NUM_SAMPLES);
    icm42760p_accelBias.y = (int16_t)(sum_y / (float)NUM_SAMPLES);
    icm42760p_accelBias.z = (int16_t)(sum_z / (float)NUM_SAMPLES);

#if defined(ICM42670P_DEBUG)
  xlog("AccelBias [%05d:%05d:%05d]\n\r",
       icm42760p_accelBias.x, icm42760p_accelBias.y, icm42760p_accelBias.z);
#endif  // ICM42670P_DEBUG

}

void ICM42670P_ReadGyroConfig() {
  uint8_t data = ICM42670P_ReadReg(ICM42670P_GYRO_CONFIG0);
  xlog("%s:%d ICM42670P_GYRO_CONFIG0:%d\n\r", __func__, __LINE__, data);
}

void ICM42670P_ConfigGyro() {
  uint8_t scale = ICM42670P_CONFIG_GYRO_2k_DPS;
  uint8_t odr = ICM42670P_CONFIG_RATE_25_Hz;
  uint8_t config = scale | odr;
  ICM42670P_WriteReg(ICM42670P_GYRO_CONFIG0, config);
  switch (scale) {
    case ICM42670P_CONFIG_GYRO_2k_DPS:
      gyroSensitivity = 16.4;
      break;
    case ICM42670P_CONFIG_GYRO_1k_DPS:
      gyroSensitivity = 32.8;
      break;
    case ICM42670P_CONFIG_GYRO_500_DPS:
      gyroSensitivity = 65.5;
      break;
    case ICM42670P_CONFIG_GYRO_250_DPS:
      gyroSensitivity = 131;
      break;
    default:
      break;
  }
  // gyro needs a few millis to reconfigure
  HAL_Delay(20);
}

void ICM42670P_ReadGyro() {
  uint8_t data[2] = { 0 };

  data[0] = ICM42670P_ReadReg(ICM42670P_GYRO_DATA_X0);
  data[1] = ICM42670P_ReadReg(ICM42670P_GYRO_DATA_X1);
  icm42760p_gyro.x = ((data[1] << 8) | data[0]);

  data[0] = ICM42670P_ReadReg(ICM42670P_GYRO_DATA_Y0);
  data[1] = ICM42670P_ReadReg(ICM42670P_GYRO_DATA_Y1);
  icm42760p_gyro.y = ((data[1] << 8) | data[0]);

  data[0] = ICM42670P_ReadReg(ICM42670P_GYRO_DATA_Z0);
  data[1] = ICM42670P_ReadReg(ICM42670P_GYRO_DATA_Z1);
  icm42760p_gyro.z = ((data[1] << 8) | data[0]);

#if defined(ICM42670P_DEBUG)
  xlog("Gyro [%05d:%05d:%05d] \n\r",
       icm42760p_gyro.x, icm42760p_gyro.y, icm42760p_gyro.z);
#endif  // ICM42670P_DEBUG
}

void ICM42670P_ReadGyroMultiple() {
  uint8_t data[6] = {0};

  ICM42670P_ReadRegMultiple(ICM42670P_GYRO_DATA_X1, data, 6);
  if (isHALError) {
    return;
  }

  icm42760p_gyro.x = ((data[0] << 8) | data[1]);
  icm42760p_gyro.y = ((data[2] << 8) | data[3]);
  icm42760p_gyro.z = ((data[4] << 8) | data[5]);

#if defined(ICM42670P_DEBUG)
  xlog("Gyro [%05d:%05d:%05d] \n\r",
       icm42760p_gyro.x, icm42760p_gyro.y, icm42760p_gyro.z);
#endif  // ICM42670P_DEBUG
}

void ICM42670P_CalibrateGyroBias() {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
        ICM42670P_ReadGyroMultiple();
        sum_x += (int16_t)icm42760p_gyro.x;
        sum_y += (int16_t)icm42760p_gyro.y;
        sum_z += (int16_t)icm42760p_gyro.z;
        HAL_Delay(1);
    }

    xlog("[%ld:%ld:%ld] \n\r", sum_x, sum_y, sum_z);
    icm42760p_gyroBias.x = (int16_t)(sum_x / (float)NUM_SAMPLES);
    icm42760p_gyroBias.y = (int16_t)(sum_y / (float)NUM_SAMPLES);
    icm42760p_gyroBias.z = (int16_t)(sum_z / (float)NUM_SAMPLES);


#if defined(ICM42670P_DEBUG)
  xlog("GyroBias [%05d:%05d:%05d]\n\r",
       icm42760p_gyroBias.x, icm42760p_gyroBias.y, icm42760p_gyroBias.z);
#endif  // ICM42670P_DEBUG

}

void ICM42670P_ReadAccelGyro() {
  uint8_t data[12] = {0};

  ICM42670P_ReadRegMultiple(ICM42670P_ACCEL_DATA_X1, data, 12);
  if (isHALError) {
    return;
  }

  icm42760p_accel.x = ((data[0] << 8) | data[1]);
  icm42760p_accel.y = ((data[2] << 8) | data[3]);
  icm42760p_accel.z = ((data[4] << 8) | data[5]);
  icm42760p_gyro.x = ((data[6] << 8) | data[7]);
  icm42760p_gyro.y = ((data[8] << 8) | data[9]);
  icm42760p_gyro.z = ((data[10] << 8) | data[11]);

#if defined(ICM42670P_USE_BIAS)
  icm42760p_accel.x -= icm42760p_accelBias.x;
  icm42760p_accel.y -= icm42760p_accelBias.y;
  icm42760p_accel.z -= icm42760p_accelBias.z;
  icm42760p_gyro.x -= icm42760p_gyroBias.x;
  icm42760p_gyro.y -= icm42760p_gyroBias.y;
  icm42760p_gyro.z -= icm42760p_gyroBias.z;
#endif  // ICM42670P_USE_BIAS

#if defined(ICM42670P_DEBUG)
  xlog("Accel [%05d:%05d:%05d], Gyro [%05d:%05d:%05d]\n\r",
       icm42760p_accel.x, icm42760p_accel.y, icm42760p_accel.z,
       icm42760p_gyro.x, icm42760p_gyro.y, icm42760p_gyro.z);
  
  // icm42760p_accels.x = (icm42760p_accel.x * 1000) / accelSensitivity;
  // icm42760p_accels.y = (icm42760p_accel.y * 1000) / accelSensitivity;
  // icm42760p_accels.z = (icm42760p_accel.z * 1000) / accelSensitivity;
  // icm42760p_gyros.x = (icm42760p_gyro.x * 1000) / gyroSensitivity;
  // icm42760p_gyros.y = (icm42760p_gyro.y * 1000) / gyroSensitivity;
  // icm42760p_gyros.z = (icm42760p_gyro.z * 1000) / gyroSensitivity;
  // xlog("Accels [%.1f:%.1f:%.1f], Gyros [%05.1f:%.1f:%.1f]\n\r", 
  //      icm42760p_accels.x, icm42760p_accels.y, icm42760p_accels.z,
  //      icm42760p_gyros.x, icm42760p_gyros.y, icm42760p_gyros.z);

#endif  // ICM42670P_DEBUG

}

void ICM42670P_EnableSoftwareReset() {
  uint8_t data = 1 << 4;
  ICM42670P_WriteReg(ICM42670P_SIGNAL_PATH_RESET, data);
}

void ICM42670P_ReadWOM() {
  uint8_t data = ICM42670P_ReadReg(ICM42670P_WOM_CONFIG);
  xlog("%s:%d ICM42670P_WOM_CONFIG:%d\n\r", __func__, __LINE__, data);
}

void ICM42670P_WriteWOM() {
  uint8_t data = 0x1;
  ICM42670P_WriteReg(ICM42670P_WOM_CONFIG, data);
}

/**************************************
Date: May 31, 2020
Function: 平方根倒数 求四元数用到
***************************************/
float InvSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;
  x = number * 0.5F;
  y = number;
  i = *((long*)&y);
  i = 0x5f375a86 - (i >> 1);
  y = *((float*)&i);
  y = y * (f - (x * y * y));

  return y;
}

/**************************************
Date: May 31, 2020
Function: 四元数解算
***************************************/
void ICM42670P_QuaternionSolution() {
  // ?? to check
  SensorRawXYZ axisChanged_accel = {icm42760p_accel.x, -icm42760p_accel.y, icm42760p_accel.z};
  SensorRawXYZ axisChanged_gyro = {icm42760p_gyro.x, -icm42760p_gyro.y, -icm42760p_gyro.z};

  float gx, gy, gz, ax, ay, az;
  ax = axisChanged_accel.x / ICM42670P_ACCEL_RATIO;
  ay = axisChanged_accel.y / ICM42670P_ACCEL_RATIO;
  az = axisChanged_accel.z / ICM42670P_ACCEL_RATIO;
  gx = axisChanged_gyro.x * ICM42670P_GYRO_RATIO;
  gy = axisChanged_gyro.y * ICM42670P_GYRO_RATIO;
  gz = -(axisChanged_gyro.z * ICM42670P_GYRO_RATIO);

  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = icm42760p_q1 * icm42760p_q3 - icm42760p_q0 * icm42760p_q2;
    halfvy = icm42760p_q0 * icm42760p_q1 + icm42760p_q2 * icm42760p_q3;
    halfvz = icm42760p_q0 * icm42760p_q0 - 0.5f + icm42760p_q3 * icm42760p_q3;
    // 误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0 / SAMPLING_FREQ);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0 / SAMPLING_FREQ);
      integralFBz += twoKi * halfez * (1.0 / SAMPLING_FREQ);

      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f;  // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }
    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0 / SAMPLING_FREQ));  // pre-multiply common factors
  gy *= (0.5f * (1.0 / SAMPLING_FREQ));
  gz *= (0.5f * (1.0 / SAMPLING_FREQ));

  qa = icm42760p_q0;
  qb = icm42760p_q1;
  qc = icm42760p_q2;
  icm42760p_q0 += (-qb * gx - qc * gy - icm42760p_q3 * gz);
  icm42760p_q1 += (qa * gx + qc * gz - icm42760p_q3 * gy);
  icm42760p_q2 += (qa * gy - qb * gz + icm42760p_q3 * gx);
  icm42760p_q3 += (qa * gz + qb * gy - qc * gx);
  // Normalise quaternion
  recipNorm = InvSqrt(icm42760p_q0 * icm42760p_q0 + icm42760p_q1 * icm42760p_q1 + icm42760p_q2 * icm42760p_q2 + icm42760p_q3 * icm42760p_q3);
  icm42760p_q0 *= recipNorm;
  icm42760p_q1 *= recipNorm;
  icm42760p_q2 *= recipNorm;
  icm42760p_q3 *= recipNorm;
}
