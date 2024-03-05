#include "global.h"

// global vars
int testCounter = 0;
bool testBool = false;
int testInt = 0;
int testCase = 0;

GPIO_PinState testPinState = GPIO_PIN_RESET;
HAL_StatusTypeDef HALStatus = HAL_ERROR;

uint8_t rx_buff[12] = {0};

#ifdef __GNUC__
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xffff);
  return ch;
}
#else
int fputc(int ch, FILE* stream) {
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xffff);
  return ch;
}
#endif

// S-Curve, 定義邏輯斯諦函數，並加入斜率 k 的參數
double logistic_sigmoid(double x, double k) {
  return 1.0 / (1.0 + exp(-k * (x - 0.0)));  // 這裡的 0.0 可以替換成中心位置 x_0
}

void printInfo() {
  xlog("%s:%d, ========================== %d\n\r", __func__, __LINE__, testCounter++);

  testCase = 21;
  if (testCase == 0) {
    xlog("%s:%d, SystemCoreClock:%ld \n\r", __func__, __LINE__, SystemCoreClock);
    xlog("%s:%d, uwTick:%ld \n\r", __func__, __LINE__, uwTick);

  } else if (testCase == 1) {

    // 1. Color sensor TCS3472
    // >> 將 color sensor 的 RGB 與 clear 值讀出，顯示於 LCD 上.
#if defined(TEST_TCS3472)

    TCS3472_Color colorData;
    memset(&colorData, 0, sizeof(TCS3472_Color));
    TCS3472_readColor(&tcs3472, &colorData);
    xlog("%s:%d, r:%d, g:%d, b:%d, c:%d \n\r", __func__, __LINE__, colorData.r, colorData.g, colorData.b, colorData.c);
    float factor = colorData.c / 255.0;
    xlog("%s:%d, r:%.1f, g:%.1f, b:%.1f \n\r", __func__, __LINE__, colorData.r / factor, colorData.g / factor, colorData.b / factor);
#endif  // TEST_TCS3472

  } else if (testCase == 2) {

    // 2. Brightness sensor BH1750
    // >> 將 brightness 的 lux 值讀出，顯示於 LCD 上.
#if defined(TEST_BH1750)
    xlog("%s:%d BH1750_ReadLux:%.2f\n\r", __func__, __LINE__, BH1750_ReadLux());
#endif  // TEST_BH1750

  } else if (testCase == 3) {
    // 3. Timer
    // 1. 設計一個 0.1 秒的計時器:
    // - 利用 timer 計數與中斷，將此 0.1 秒的計數顯示於 LCD 上面
    // 2. 按b utton 啟動計時，再按一次停止計時
    // 3. button 中斷 ISR 來作
    // OK

    if (testBool) {
      testBool = false;
      HAL_TIM_Base_Stop_IT(&htim2);
    } else {
      HAL_TIM_Base_Start_IT(&htim2);
      testBool = true;
    }

  } else if (testCase == 4) {
    // 4. GPIO 控制 LED
    // OK
    toggleUserLED();

  } else if (testCase == 5) {
    // 5. 20KHz 的頻率且可調 duty cycle 100 階的 PWM 輸出
    // OK
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    testInt++;
    testInt %= 3;
    if (testInt == 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
    } else if (testInt == 1) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 60);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 90);
    }

  } else if (testCase == 6) {
    // 6. 使用 UART 輸出 printf 的 log
    xlog("%s:%d, SystemCoreClock:%ld \n\r", __func__, __LINE__, SystemCoreClock);

  } else if (testCase == 7) {
    // 7. ADC 取樣
    // read input pin
    testPinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    // read ADC
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    testInt = HAL_ADC_GetValue(&hadc1);
    xlog("%s:%d, read ADC value:%d \n\r", __func__, __LINE__, testInt);

  } else if (testCase == 8) {
    // 8. 中斷 Interrupt
    // OK

  } else if (testCase == 9) {
    // 9. SPI 的應用 — RFID Reader

  } else if (testCase == 10) {
    // 10. 使用 Timer 中斷驅動 ADC 取樣

  } else if (testCase == 11) {
    // 11. ultrasonic 測距
  } else if (testCase == 12) {
    // 12. 4×4 鍵盤掃描
    // >> 利用 DO與 DI 掃描 9*9 按鍵，透過 UART 顯示按鍵值於終端螢幕上

  } else if (testCase == 13) {
    // 13. 伺服馬達

  } else if (testCase == 14) {
    // 14. FreeRTOS

  } else if (testCase == 15) {
    // 15. FreeModbus - Slave
    // 1. 已下載 freeModbus source code from github。
    // 2. 已下載並安裝 ICDT MODBUS RTU Slave 供模擬測試。
    // 3. 已下載並安裝 ICDT MODBUS RTU Master 供模擬測試。
    // 4. 已實現 Modbus Slave Sampe 測試板，並與 ICDT MODBUS RTU Master 通訊正常。(7/3)
  } else if (testCase == 16) {
    // 16. FreeModbus - Master

    // } else if (testCase == 17) {

  } else if (testCase == 20) {
    // 測試邏輯斯諦函數，並指定斜率 k 的值
    double k = 2.0;  // 斜率的值可以更改

    double begin = -5.0;
    double end = 5.0;
    double out = 0.0;

    do {
      out = logistic_sigmoid(begin, k);
      xlog("%s:%d, input:%f, k:%f, r:%f \n\r", __func__, __LINE__, begin, k, out);
      begin += 0.2;
    } while (begin < end);

  } else if (testCase == 21) {
    getFWVersion_8015d();
    queryWheelSpeed_8015d(2);
    // getCurrent_8015d(0);
    // getCurrent_8015d(1);
  }
}

void setUserLED(LEDState state) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (state == LED_On) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void toggleUserLED() {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

#ifdef MCSDK

char* printMotorState(MCI_State_t state) {
  switch (state) {
    case ICLWAIT:
      return toString(ICLWAIT);
    case IDLE:
      return toString(IDLE);
    case ALIGNMENT:
      return toString(ALIGNMENT);
    case CHARGE_BOOT_CAP:
      return toString(CHARGE_BOOT_CAP);
    case OFFSET_CALIB:
      return toString(OFFSET_CALIB);
    case START:
      return toString(START);
    case SWITCH_OVER:
      return toString(START);
    case RUN:
      return toString(RUN);
    case STOP:
      return toString(STOP);
    case FAULT_NOW:
      return toString(FAULT_NOW);
    case FAULT_OVER:
      return toString(FAULT_OVER);
    case WAIT_STOP_MOTOR:
      return toString(WAIT_STOP_MOTOR);
    default:
      break;
  }

  return "N/A";
}

void printMotorError(uint16_t errors) {
  if (errors == MC_NO_ERROR) {
    xlog("%s:%d, MC_NO_ERROR \n\r", __func__, __LINE__);
    return;
  }

  char s[128] = "";
  if (errors & MC_DURATION) {
    strcat(s, " # ");
    strcat(s, toString(MC_DURATION));
    // xlog("%s:%d, MC_DURATION \n\r", __func__, __LINE__);
  }
  if (errors & MC_OVER_VOLT) {
    strcat(s, " # ");
    strcat(s, toString(MC_OVER_VOLT));
    // xlog("%s:%d, MC_OVER_VOLT \n\r", __func__, __LINE__);
  }
  if (errors & MC_UNDER_VOLT) {
    strcat(s, " # ");
    strcat(s, toString(MC_UNDER_VOLT));
    // xlog("%s:%d, MC_UNDER_VOLT \n\r", __func__, __LINE__);
  }
  if (errors & MC_OVER_TEMP) {
    strcat(s, " # ");
    strcat(s, toString(MC_OVER_TEMP));
    // xlog("%s:%d, MC_OVER_TEMP \n\r", __func__, __LINE__);
  }
  if (errors & MC_START_UP) {
    strcat(s, " # ");
    strcat(s, toString(MC_START_UP));
    // xlog("%s:%d, MC_START_UP \n\r", __func__, __LINE__);
  }
  if (errors & MC_SPEED_FDBK) {
    strcat(s, " # ");
    strcat(s, toString(MC_SPEED_FDBK));
    // xlog("%s:%d, MC_SPEED_FDBK \n\r", __func__, __LINE__);
  }
  if (errors & MC_BREAK_IN) {
    strcat(s, " # ");
    strcat(s, toString(MC_BREAK_IN));
    // xlog("%s:%d, MC_BREAK_IN \n\r", __func__, __LINE__);
  }
  if (errors & MC_SW_ERROR) {
    strcat(s, " # ");
    strcat(s, toString(MC_SW_ERROR));
    // xlog("%s:%d, MC_SW_ERROR \n\r", __func__, __LINE__);
  }

  xlog("%s:%d, error occur:%s \n\r", __func__, __LINE__, s);
}

#endif  // MCSDK

void configFDCAN(void) {
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

    xlog("DataLength:0x%lx, Identifier:0x%lx, IdType:0x%lx, RxFrameType:0x%lx \n\r", RxHeader.DataLength , RxHeader.Identifier, RxHeader.IdType, RxHeader.RxFrameType);

    xlog("0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x \n\r",
         RxBuffer[0],
         RxBuffer[1],
         RxBuffer[2],
         RxBuffer[3],
         RxBuffer[4],
         RxBuffer[5],
         RxBuffer[6],
         RxBuffer[7]);
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

  /* Start the Transmission process */
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, msg) != HAL_OK) {
    /* Transmission request Error */
    xlog("%s:%d, HAL_FDCAN_AddMessageToTxFifoQ error \n\r", __func__, __LINE__);
  }

  return 0;
}

uint32_t CanID_ZL8015D = 0x601;
void getFWVersion_8015d() {
  uint8_t data[8] = {0x40, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void queryWheelSpeed_8015d(uint8_t motorID) {
  uint8_t data[8] = {0x43, 0x6C, 0x60, motorID + 1, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void getCurrent_8015d(uint8_t motorID) {
  uint8_t data[8] = {0x40, 0x77, 0x60, motorID + 1, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}
