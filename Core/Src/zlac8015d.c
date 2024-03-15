
#include <zlac8015d.h>

uint32_t CanID_ZL8015D = 0x601;
uint32_t CanID_ZL8015D_Rx = 0x581;
SpeedModeCtrlFlag speedModeCtrlFlag = smcf_sync;

bool isInit_ZL8015D = false;
int16_t targetSpeed_new[2] = { 0, 0 };
int16_t targetSpeed_old[2] = { 0, 0 };
int16_t wheelSpeedRPM[2] = { 0, 0 };
int16_t wheelSpeedMS[2] = { 0, 0 };
float wheelSpeedMS_enc[2] = { 0, 0 };
uint16_t acctime_up = 2000;
uint16_t acctime_down = 1000;

void resetVars() {
  isInit_ZL8015D = false;
  targetSpeed_old[M1] = 0;
  targetSpeed_old[M2] = 0;
}

void setSyncFlag(SpeedModeCtrlFlag state) {
  uint8_t data[8] = {0x2B, 0x0F, 0x20, 0x00, state, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void setSpeedMode(){
  uint8_t data[8] = {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void setSCurveAccUpTime(uint16_t t) {
  uint8_t data[8] = {0x23, 0x83, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  data[4] = t & 0xff;
  data[5] = (t >> 8) & 0xff;
  data[6] = (t >> 16) & 0xff;
  data[7] = (t >> 24) & 0xff;
  CAN1_Send(CanID_ZL8015D, data);

  data[3] = 0x02;
  CAN1_Send(CanID_ZL8015D, data);
}

void setSCurveAccDownTime(uint16_t t) {
  uint8_t data[8] = {0x23, 0x84, 0x60, 0x01, 0x00, 0x00, 0x00, 0x00};
  data[4] = t & 0xff;
  data[5] = (t >> 8) & 0xff;
  data[6] = (t >> 16) & 0xff;
  data[7] = (t >> 24) & 0xff;
  CAN1_Send(CanID_ZL8015D, data);

  data[3] = 0x02;
  CAN1_Send(CanID_ZL8015D, data);
}

void setEnable(uint8_t stage) {
  uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (stage == 1) {
    data[4] = 0x06;
    CAN1_Send(CanID_ZL8015D, data);
  } else if (stage == 2) {
    data[4] = 0x07;
    CAN1_Send(CanID_ZL8015D, data);
  } else if (stage == 3){
    data[4] = 0x0F;
    CAN1_Send(CanID_ZL8015D, data);
  }
}

void setWheelSpeed(uint8_t motorID, int16_t targetSpeedRPM) {

  if (!(motorID == M1 || motorID == M2)) {
    return;
  }
  uint8_t data[8] = {0x23, 0xFF, 0x60, motorID + 1, 0x00, 0x00, 0x00, 0x00};
  data[4] = targetSpeedRPM & 0xff;
  data[5] = (targetSpeedRPM >> 8) & 0xff;
  data[6] = (targetSpeedRPM >> 16) & 0xff;
  data[7] = (targetSpeedRPM >> 24) & 0xff;
  CAN1_Send(CanID_ZL8015D, data);
}
void setWheelSpeed_sync(int16_t targetSpeedRPM_m1, int16_t targetSpeedRPM_m2) {
    uint8_t data[8] = {0x23, 0xFF, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00};
    data[4] = targetSpeedRPM_m1 & 0xff;
    data[5] = (targetSpeedRPM_m1 >> 8) & 0xff;
    data[6] = targetSpeedRPM_m2 & 0xff;
    data[7] = (targetSpeedRPM_m2 >> 8) & 0xff;
    CAN1_Send(CanID_ZL8015D, data);
}

void stopMachine() {
  resetVars();
  uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void emergencyStop(bool onoff) {
  uint8_t data[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  data[4] = onoff ? 0x02 : 0x0F;
  CAN1_Send(CanID_ZL8015D, data);
}

// value will be update in CAN bus callback : HAL_FDCAN_RxFifo0Callback
void queryWheelSpeed(uint8_t motorID) {
  uint8_t data[8] = {0x43, 0x6C, 0x60, motorID + 1, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void getCurrent(uint8_t motorID) {
  uint8_t data[8] = {0x40, 0x77, 0x60, motorID + 1, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void getFWVersion() {
  uint8_t data[8] = {0x40, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  // CAN1_Send(CanID_ZL8015D, data);
  CAN1_Sendx(CanID_ZL8015D, data);
}

void getEncoder(uint8_t motorID) {
  uint8_t data[8] = {0x43, 0x64, 0x60, motorID + 1, 0x00, 0x00, 0x00, 0x00};
  CAN1_Send(CanID_ZL8015D, data);
}

void enableMotor() {
  setEnable(1);
  setEnable(2);
  setEnable(3);
}

void speedMode_asyncInit() {

  if (!isInit_ZL8015D) {
    isInit_ZL8015D = true;
  } else {
    return;
  }

  setSyncFlag(smcf_async);
  setSpeedMode();
  setSCurveAccUpTime(acctime_up);
  setSCurveAccDownTime(acctime_down);
  enableMotor();
}

void speedMode_syncInit() {
  if (!isInit_ZL8015D) {
    isInit_ZL8015D = true;
  } else {
    return;
  }

  setSyncFlag(smcf_sync);
  setSpeedMode();
  setSCurveAccUpTime(acctime_up);
  setSCurveAccDownTime(acctime_down);
  enableMotor();
}

void setTargetSpeed(uint8_t motorID, int16_t targetSpeedRPM) {
//  if (isBumperSignalIn && bumperCounter < BUMPER_COUNTER_MAX) {
//    // temp disable
//    targetSpeed_new[motorID] = -3;
//    global_sm = GSM_SetMotorSpeed;
//    return;
//  }
//
//  if (isPauseSignalIn) {
//    return;
//  }

  targetSpeed_new[motorID] = targetSpeedRPM;
//  global_sm = GSM_SetMotorSpeed;
}
void setTargetSpeed_sync(int16_t targetSpeedRPM_m1, int16_t targetSpeedRPM_m2) {
//  if (isBumperSignalIn && bumperCounter < BUMPER_COUNTER_MAX) {
//    // temp disable
//    targetSpeed_new[M1] = -3;
//    targetSpeed_new[M2] = -3;
//    global_sm = GSM_SetMotorSpeed;
//    return;
//  }
//
//  if (isPauseSignalIn) {
//    return;
//  }

  targetSpeed_new[M1] = targetSpeedRPM_m1;
  targetSpeed_new[M2] = targetSpeedRPM_m2;
//  task_setMotorSpeed = true;
//  global_sm = GSM_SetMotorSpeed;
}

// M1 : left motor
// M2 : right motor
void setTargetSpeed_task() {

  if (speedModeCtrlFlag == smcf_sync) {

      // if (targetSpeed_new[M1] != targetSpeed_old[M1] || targetSpeed_new[M2] != targetSpeed_old[M2]) {
        // target speed change
        speedMode_syncInit();
        // xlog("%s:%d, M1:%d, M2:%d\n\r", __func__, __LINE__, targetSpeed_new[M1], targetSpeed_new[M2]);
        targetSpeed_old[M1] = targetSpeed_new[M1];
        targetSpeed_old[M2] = targetSpeed_new[M2];
        setWheelSpeed_sync(targetSpeed_new[M1], -targetSpeed_new[M2]);
      // }

  } else if (speedModeCtrlFlag == smcf_async) {

    for (int i = 0; i < 2; i++) {
      if (targetSpeed_new[i] != targetSpeed_old[i]) {
        // target speed change
        speedMode_asyncInit();
        // xlog("%s:%d, M%d, speed:%d \n\r", __func__, __LINE__, i + 1, targetSpeed_new[i]);
        targetSpeed_old[i] = targetSpeed_new[i];
        if (i == M1) {
          setWheelSpeed(M1, targetSpeed_new[i]);
        } else if (i == M2) {
          setWheelSpeed(M2, -targetSpeed_new[i]);
        }
        // both M1/M2 target speed == 0
        // if (targetSpeed_old[M1] == 0 && targetSpeed_old[M2] == 0) {
        //     stopMachine();
        // }
      }
    }
  }
}

// rpm to meter/s
// #define  CONV_RPM2RMPS (SPEED_RESOLUTION * WHEEL_PERIMETER / 60)
float rpm2mps(float rpm) {
 return rpm * WHEEL_PERIMETER / (float)U_RPM;
}

int mps2rpm(float mps) {
 return (int)( mps * (float)U_RPM / WHEEL_PERIMETER );
}

int16_t limitMinRPM(float inputRPM) {
  int16_t rt = (int16_t)inputRPM;
  if (inputRPM > 0.0 && inputRPM < 1.0) {
    rt = 1.0;
  } else if (inputRPM < 0.0 && inputRPM > -1.0) {
    rt = -1.0;
  }
  return rt;
}
