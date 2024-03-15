/*
 * hal_extension.h
 *
 *  Created on: Mar 13, 2024
 *      Author: Gray.Lin
 */

#ifndef INC_HAL_EXTENSION_H_
#define INC_HAL_EXTENSION_H_

#include "global.h"

void ConfigFDCAN(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
uint8_t CAN1_Send(uint32_t id, uint8_t* msg);
uint8_t CAN1_Sendx(uint32_t id, uint8_t* msg);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_HAL_EXTENSION_H_ */
