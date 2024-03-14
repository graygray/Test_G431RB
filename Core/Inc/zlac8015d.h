#pragma once

#include "global.h"
#include "pstwo.h"

#define M1    (uint8_t)(0x0)    // Motor 1
#define M2    (uint8_t)(0x1)    // Motor 2
#define MBoth (uint8_t)(0x2)    // Motor 1 + Motor 2

#define U_RPM 60
#define WHEEL_PERIMETER (0.173 * PI)  // 2Rπ = 0.173(mm) * 3.1415926, >> circumference

typedef enum { 
  smcf_async = 0U, 
  smcf_sync 
} SpeedModeCtrlFlag;

// extern int currentSpeed_mps[2];
// extern int currentSpeed_rpm[2];
extern SpeedModeCtrlFlag speedModeCtrlFlag;
extern uint32_t CanID_ZL8015D;
extern uint32_t CanID_ZL8015D_Rx;
extern int16_t wheelSpeedRPM[2];
extern int16_t wheelSpeedMS[2];
extern float wheelSpeedMS_enc[2];
extern uint16_t acctime_up;
extern uint16_t acctime_down;

void setSyncFlag(SpeedModeCtrlFlag state);
void setSpeedMode();
void speedMode_asyncInit();
void speedMode_syncInit();

void setTargetSpeed(uint8_t motorID, int16_t targetSpeedRPM);
void setTargetSpeed_sync(int16_t targetSpeedRPM_m1, int16_t targetSpeedRPM_m2);
void setTargetSpeed_task();
void setWheelSpeed(uint8_t motorID, int16_t targetSpeedRPM);
void setWheelSpeed_sync(int16_t targetSpeedRPM_m1, int16_t targetSpeedRPM_m2);
void stopMachine();
void enableMotor();
void setEnable(uint8_t stage);
void setSCurveAccUpTime(uint16_t t);
void setSCurveAccDownTime(uint16_t t);
void emergencyStop(bool onoff);
void queryWheelSpeed(uint8_t motorID);
void getCurrent(uint8_t motorID);
void getFWVersion();
void getEncoder(uint8_t motorID);

extern float rpm2mps(float rpm);
extern int mps2rpm(float mps);
extern int16_t limitMinRPM(float inputRPM);
