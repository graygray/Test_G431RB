#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include "pstwo.h"
#include "zlac8015d.h"

#define DELAY_TIME  delay_us(2); 

int16_t targetSpeed = 5;
uint8_t targetSpeedStep = 1;
float_t targetSpeedMS = 0.05;
float_t targetSpeedStepMS = 0.1;

float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 
float Move_X, Move_Y, Move_Z; 
float RC_Velocity=100; 
uint8_t  PS2_ON_Flag = 1;
//Button value reading, zero time storage
//??????????????ϫ
uint16_t Handkey;	
//Start the order. Request data
//???????????????
uint8_t Comd[2]={0x01,0x42};	
//Data store array
//????ϫ????
uint8_t Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 

Rocker_OPs rocker_op_last = Rocker_OP_NA;

uint16_t MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	}; //Key value and key name //???????????

void delay_us(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;
 
  startval = SysTick->VAL;
  tickn = HAL_GetTick();
  //sysc = 65000;  //SystemCoreClock / (1000U / uwTickFreq);
  delays =udelay * 170; //sysc / 1000 * udelay;
  if(delays > startval)
  {
    while(HAL_GetTick() == tickn)
    { }
    wait = 170000 + startval - delays;
    while(wait < SysTick->VAL)
    { }
  }
  else
  {
    wait = startval - delays;
    while(wait < SysTick->VAL && HAL_GetTick() == tickn)
    { }
  }
}  
/**************************************************************************
Function: Ps2 handle task
Input   : none
Output  : none
?????????PS2???????
??????????
????  ?????
**************************************************************************/	
#if 0
void pstwo_task(void *pvParameters)
{
    uint32_t lastWakeTime = getSysTickCnt();
    while(1)
    {	
			//The task is run at 100hz
			//????????100Hz????????? 	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));
			//Read the ps2 data
			//???PS2??????	
      PS2_Read(); 	
    }
}  
#endif
/**************************************************************************
Function: Ps2 handle initializer
Input   : none
Output  : none
?????????PS2????????
??????????
????  ?????
**************************************************************************/	
void PS2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DO_PORT, DO_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DO_PIN | CS_PIN | CKL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DI_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DI_PORT, &GPIO_InitStruct);
}
/**************************************************************************
Function: Read the control of the ps2 handle
Input   : none
Output  : none
????????????PS2??????????
??????????
????  ?????
**************************************************************************/	
void PS2_Read(void)
{
  // Reading key
  // ??????????
  PS2_KEY = PS2_DataKey();
//   xlog("PS2_KEY:%03.0f \n\r", PS2_KEY);
  // Read the analog of the remote sensing x axis on the left
  // ?????????X??????????
  PS2_LX = PS2_AnologData(PSS_LX);
  // Read the analog of the directional direction of remote sensing on the left
  // ?????????Y??????????
  PS2_LY = PS2_AnologData(PSS_LY);
  // Read the analog of the remote sensing x axis
  // ?????????X??????????
  PS2_RX = PS2_AnologData(PSS_RX);
  // Read the analog of the directional direction of the remote sensing y axis
  // ?????????Y??????????
  PS2_RY = PS2_AnologData(PSS_RY);
//   xlog("PS2_LX:%03.0f, PS2_LY:%03.0f, PS2_RX:%03.0f, PS2_RY:%03.0f \n\r", PS2_LX, PS2_LY, PS2_RX, PS2_RY);

#if 0
	static int Strat;
	if(PS2_KEY==4&&PS2_ON_Flag==0) 
		//The start button on the // handle is pressed
		//??????Start??????????
		Strat=1; 
	
	if(Strat&&(PS2_LY<118)&&PS2_ON_Flag==0&&Deviation_Count>=CONTROL_DELAY)
		//When the button is pressed, you need to push the right side forward to the formal ps2 control car
		//Start???????????????????????????????????PS2???????
		PS2_ON_Flag=1,Remote_ON_Flag=0,APP_ON_Flag=0,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
#endif        
}
/**************************************************************************
Function: Send commands to the handle
Input   : none
Output  : none
??????????????????????
??????????
????  ?????
**************************************************************************/	
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
                  DO_H;     //Output a control bit //????�f????�f
		}
		else DO_L;

		CLK_H;      //Clock lift //???????
		DELAY_TIME;
		CLK_L;
		DELAY_TIME;
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
	delay_us(10);
}
/**************************************************************************
Function: Whether it is a red light mode, 0x41= analog green light, 0x73= analog red light
Input   : none
Output  : 0: red light mode, other: other modes
?????????????????????,0x41=???????0x73=?????
??????????
????  ???0?????????????????????
**************************************************************************/	
uint8_t PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders //???????
	PS2_Cmd(Comd[1]);  //Request data //????????
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
/**************************************************************************
Function: Read the handle data
Input   : none
Output  : none
???????????????????
??????????
????  ?????
**************************************************************************/	
void PS2_ReadData(void)
{
	volatile uint8_t byte=0;
	volatile uint16_t ref=0x01;
	CS_L;
	PS2_Cmd(Comd[0]);  //Start orders //???????
	PS2_Cmd(Comd[1]);  //Request data //????????
	for(byte=2;byte<9;byte++) //Start receiving data //???????????
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
		  if(DI)
		    Data[byte] = ref|Data[byte];
		}
        delay_us(10);
	}
	CS_H;
}
/**************************************************************************
Function: Handle the data of the read 2 and handle only the key parts
Input   : none
Output  : 0: only one button presses the next time; No press
??????????????????PS2????????????,????????????? 
??????????
????  ???0: ???????????????????; 1: �_????
**************************************************************************/	
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3]; //This is 16 buttons, pressed down to 0, and not pressed for 1  //????16?????????????0??�_?????1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;  //No buttons //????�e????????
}
/**************************************************************************
Function: Get a simulation of a rocker
Input   : Rocker
Output  : Simulation of rocker, range 0~ 256
????????????????????????
???????????
????  ????????????, ??�Y0~256
**************************************************************************/
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button];
}
/**************************************************************************
Function: Clear data buffer
Input   : none
Output  : none
?????????????????????
??????????
????  ?????
**************************************************************************/
void PS2_ClearData()
{
	uint8_t a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function: Handle vibration function
Input   : motor1: the right small vibrator, 0x00, other
          motor2: the left big shock motor 0x40~ 0xff motor is open, and the value is greater
Output  : none
?????????????????
????????motor1:???????? 0x00?????????
	        motor2:???????? 0x40~0xFF ??????????? ?????
????  ?????
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	CS_L;
	delay_us(16);
  PS2_Cmd(0x01); //Start order //???????
	PS2_Cmd(0x42); //Request data //????????
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(10);  
}
/**************************************************************************
Function: Short press
Input   : none
Output  : none
????????????
??????????
????  ?????
**************************************************************************/
void PS2_ShortPoll(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Enter configuration
Input   : none
Output  : none
?????????????????
??????????
????  ?????
**************************************************************************/
void PS2_EnterConfing(void)
{
  CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Send mode Settings
Input   : none
Output  : none
???????????????????
??????????
????  ?????
**************************************************************************/
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  Software Settings send mode ???????��?????
	PS2_Cmd(0x03); //0x03 lock storage setup, which cannot be set by the key "mode" set mode. //0x03????????????????????????MODE??????????
				         //0xee non-locking software Settings can be set by the key "mode" set mode.//0xEE??????????????????????????MODE??????????
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Vibration setting
Input   : none
Output  : none
???????????????
??????????
????  ?????
**************************************************************************/
void PS2_VibrationMode(void)
{
	CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	delay_us(16);	
}
/**************************************************************************
Function: Complete and save the configuration
Input   : none
Output  : none
?????????????????????
??????????
????  ?????
**************************************************************************/
void PS2_ExitConfing(void)
{
    CS_L;
	delay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	delay_us(16);
}
/**************************************************************************
Function: Handle configuration initialization
Input   : none
Output  : none
????????????????????
??????????
????  ?????
**************************************************************************/
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		  //Enter configuration mode //??????????
	PS2_TurnOnAnalogMode();	//The "traffic light" configuration mode and select whether to save //?????????????????????????
	//PS2_VibrationMode();	//Open vibration mode //????????
	PS2_ExitConfing();		  //Complete and save the configuration //????????????
}
/**************************************************************************
Function: Read the handle information
Input   : none
Output  : none
??????????????????
??????????
????  ?????
**************************************************************************/
void PS2_Receive (void)
{
	if(PS2_ON_Flag)
	{
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_RX=PS2_AnologData(PSS_RX);
		PS2_RY=PS2_AnologData(PSS_RY);
	}
	PS2_KEY=PS2_DataKey();
}

void updateTargetSpeed() {
    // if (targetSpeed >= MAX_TARGET_SPEED) {
    //   targetSpeed = MAX_TARGET_SPEED;
    // } else if (targetSpeed <= targetSpeedStep) {
    //   targetSpeed = targetSpeedStep;
    // }
    if (targetSpeedMS >= SpeedMS_MAX) {
      targetSpeedMS = SpeedMS_MAX;
    } else if (targetSpeedMS <= targetSpeedStepMS) {
      targetSpeedMS = targetSpeedStepMS;
    }
    // update target speed rpm
    targetSpeed = mps2rpm(targetSpeedMS);
}

void updateTargetSpeedStep() {
    if (targetSpeedStep > MAX_SPEED_STEP) {
      targetSpeedStep = targetSpeedStep % MAX_SPEED_STEP;
    }
    targetSpeedStepMS = targetSpeedStep / 10.0;
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
**************************************************************************/
void PS2_control(void) {
  bool isParamChanged = false;

  if (PS2_RedLight()) {
    xlog("Switch PS2 Mode to (G|R light on)\n\r");
//    task_stopMotor = true;
    return;
  }

  if (PS2_KEY == PSB_TRIANGLE) {
    // ++ target speed
    isParamChanged = true;
    targetSpeedMS += targetSpeedStepMS;
    updateTargetSpeed();
  } else if (PS2_KEY == PSB_CROSS) {
    // -- target speed
    isParamChanged = true;
    targetSpeedMS -= targetSpeedStepMS;
    updateTargetSpeed();
  } else if (PS2_KEY == PSB_SQUARE) {
    // change ++/-- speed step
    isParamChanged = true;
    targetSpeedStep++;
    updateTargetSpeedStep();
  } else if (PS2_KEY == PSB_CIRCLE) {
    // just print info
    xlog("Speed:%.1f meter/s (%d rpm), Step:%.1f \n\r", targetSpeedMS, targetSpeed, targetSpeedStepMS);
    if (isPrintLog) {
      isPrintLog = false;
    } else {
      isPrintLog = true;
    }

  } else if (PS2_KEY == PSB_R1) {
    xlog("Stop Machine \n\r");
//    task_stopMotor = true;
  } else if (PS2_KEY == PSB_R2) {
    xlog("Emergency Stop \n\r");
//    task_EMOStop = true;
  } else if (PS2_KEY == PSB_PAD_LEFT) {
    // 0.5 meter/s
    isParamChanged = true;
    acctime_up = 2000;
    targetSpeedMS = SpeedMS_MIN;
    targetSpeed = mps2rpm(targetSpeedMS); ;
  } else if (PS2_KEY == PSB_PAD_UP) {
    // 0.8 meter/s
    isParamChanged = true;
    acctime_up = 2000;
    targetSpeedMS = SpeedMS_MID;
    targetSpeed = mps2rpm(targetSpeedMS); ;
  } else if (PS2_KEY == PSB_PAD_RIGHT) {
    // 1.2 meter/s
    isParamChanged = true;
    acctime_up = 4000;
    targetSpeedMS = SpeedMS_MAX;
    targetSpeed = mps2rpm(targetSpeedMS); ;
  } else if (PS2_KEY == PSB_PAD_DOWN) {
    // 0.1 meter/s
    isParamChanged = true;
    acctime_up = 2000;
    targetSpeedStepMS = 0.1;
    targetSpeedStep = 1;
    targetSpeedMS = 0.1;
    targetSpeed = mps2rpm(targetSpeedMS); ;
  }
  if (isParamChanged) {
    xlog("Set Speed:%.1f meter/s (%d rpm), Step:%.1f \n\r", targetSpeedMS, targetSpeed, targetSpeedStepMS);
  }

  float rocker_center = 127.0;

  // left rocker
  if (PS2_LY < rocker_center - (rocker_center * 0.1)) {
    if ((rocker_op_last != Rocker_OP_MF) || isParamChanged) {
      xlog(">>>>>>>>>> Move Forward, Speed:%.1f meter/s (%d rpm) \n\r", targetSpeedMS, targetSpeed);
      isPrintLog = true;
      setTargetSpeed_sync(-targetSpeed, -targetSpeed);
    }
    rocker_op_last = Rocker_OP_MF;

  } else if (PS2_LY > rocker_center + (rocker_center * 0.1)) {
    if ((rocker_op_last != Rocker_OP_MB) || isParamChanged) {
      xlog(">>>>>>>>>> Move Backward, Speed:%.1f meter/s (%d rpm) \n\r", targetSpeedMS, targetSpeed);
      isPrintLog = true;
      setTargetSpeed_sync(targetSpeed, targetSpeed);
    }
    rocker_op_last = Rocker_OP_MB;
  } else {
  	// right rocker
    if (PS2_RX < rocker_center - (rocker_center * 0.1)) {
      if ((rocker_op_last != Rocker_OP_RL) || isParamChanged) {
        xlog(">>>>>>>>>> Rotate Left, Speed:%.1f meter/s (%d rpm) \n\r", targetSpeedMS, targetSpeed);
        isPrintLog = true;
        setTargetSpeed_sync(-targetSpeed, targetSpeed);
      }
      rocker_op_last = Rocker_OP_RL;
    } else if (PS2_RX > rocker_center + (rocker_center * 0.1)) {
      if ((rocker_op_last != Rocker_OP_RR) || isParamChanged) {
        xlog(">>>>>>>>>> Rotate Right, Speed:%.1f meter/s (%d rpm) \n\r", targetSpeedMS, targetSpeed);
        isPrintLog = true;
        setTargetSpeed_sync(targetSpeed, -targetSpeed);
      }
      rocker_op_last = Rocker_OP_RR;
    } else {
      if ((rocker_op_last != Rocker_OP_NA)) {
        xlog(">>>>>>>>>> Speed Zero \n\r");
        isPrintLog = false;
        setTargetSpeed_sync(0, 0);
      }
      rocker_op_last = Rocker_OP_NA;
    }
  }

  if (isParamChanged) {
    isParamChanged = false;
  }
}
