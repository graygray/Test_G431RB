/*
 * BH1750.c
 *
 *  Created on: Jan 2, 2024
 *      Author: Gray.Lin
 */

#include "global.h"
#include "BH1750.h"

BH1750_Data dataBH1750;

void BH1750_Init(uint8_t mode, uint16_t address, I2C_HandleTypeDef* handle) {
  dataBH1750.s_mode_ = mode;
  dataBH1750.s_address_ = address << 1;
  dataBH1750.handle_ = handle;

  HAL_StatusTypeDef s_ret_ =
      HAL_I2C_Master_Transmit(dataBH1750.handle_, dataBH1750.s_address_, &dataBH1750.s_mode_, 1, HAL_MAX_DELAY);

  if (s_ret_ == HAL_OK) {
    dataBH1750.s_status_ = ALL_OK;
  }
}

float BH1750_ReadLux() {
  float lux = 0.0;

  if (dataBH1750.s_status_ == ALL_OK) {
    uint8_t readout_buffer[2];

    HAL_I2C_Master_Transmit(dataBH1750.handle_, dataBH1750.s_address_, &dataBH1750.s_mode_, 1, HAL_MAX_DELAY);
    HAL_StatusTypeDef s_ret_ =
        HAL_I2C_Master_Receive(dataBH1750.handle_, dataBH1750.s_address_, readout_buffer, 2, HAL_MAX_DELAY);

    if (s_ret_ == HAL_OK) {
        dataBH1750.s_status_ = ALL_OK;
    }

    uint16_t holder = ((uint16_t)readout_buffer[0] << 8 | readout_buffer[1]);
    lux = holder / 1.2;
  }
  return lux;
}
