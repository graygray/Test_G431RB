/*
 * hal_extension.c
 *
 *  Created on: Mar 13, 2024
 *      Author: Gray.Lin
 */

#include "hal_extension.h"
#include "main.h"
#include "zlac8015d.h"

void ConfigFDCAN(void) {
  xlog("%s:%d \n\r", __func__, __LINE__);
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x010;
  sFilterConfig.FilterID2 = 0x581;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    xlog("%s:%d, error \n\r", __func__, __LINE__);
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    xlog("%s:%d, error \n\r", __func__, __LINE__);
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    xlog("%s:%d, error \n\r", __func__, __LINE__);
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    xlog("%s:%d, error \n\r", __func__, __LINE__);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxBuffer[8];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxBuffer) != HAL_OK) {
      xlog("%s:%d, HAL_FDCAN_GetRxMessage error \n\r", __func__, __LINE__);
      return;
    }

    // xlog("DataLength:0x%lx, Identifier:0x%lx, IdType:0x%lx, RxFrameType:0x%lx \n\r", RxHeader.DataLength , RxHeader.Identifier, RxHeader.IdType, RxHeader.RxFrameType);

    // xlog("0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x \n\r",
    //      RxBuffer[0],
    //      RxBuffer[1],
    //      RxBuffer[2],
    //      RxBuffer[3],
    //      RxBuffer[4],
    //      RxBuffer[5],
    //      RxBuffer[6],
    //      RxBuffer[7]);
  }
}

uint8_t CAN1_Send(uint32_t id, uint8_t* msg) {
  FDCAN_TxHeaderTypeDef TxHeader;
  /* Prepare Tx Header */
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  // ??
  // if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_READY) {
  //   xlog("%s:%d, HAL_FDCAN_GetState not Ready \n\r", __func__, __LINE__);
  //   return 0;
  // }

  /* Start the Transmission process */
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, msg) != HAL_OK) {
    /* Transmission request Error */
    xlog("%s:%d, HAL_FDCAN_AddMessageToTxFifoQ error \n\r", __func__, __LINE__);
  }

  HAL_Delay(1);

  return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

  if (GPIO_Pin == USER_BUTTON_Pin) {
    toggleUserLED();
    printInfo();
  }
  
}
