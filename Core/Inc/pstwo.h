#ifndef __PSTWO_H
#define __PSTWO_H
//#include "delay.h"
//#include "sys.h"
//#include "system.h"
#include "stm32g4xx_hal.h"

#include "main.h"

#define PI 3.14159265358979323846

typedef enum
{
  Rocker_OP_NA  = 0U,   // Forward
  Rocker_OP_MF,         // Forward
  Rocker_OP_MB,         // Backbackward
  Rocker_OP_RL,         // Rotate Left
  Rocker_OP_RR          // Rotate Right
} Rocker_OPs;

#if !defined(USE_PS2)
#define PS2_DI_Pin 0
#define PS2_DO_Pin 0
#define PS2_CS_Pin 0
#define PS2_CLK_Pin 0
#endif // USE_PS2

#define SpeedMS_MIN 0.4
#define SpeedMS_MID 0.8
#define SpeedMS_MAX 1.2

#define MAX_TARGET_SPEED 120   // meter/s
#define MAX_SPEED_STEP   5

// map pin define
#define DI_PORT     GPIOC
#define DI_PIN      PS2_DI_Pin
#define DO_PORT     GPIOC
#define DO_PIN      PS2_DO_Pin
#define CS_PORT     GPIOC
#define CS_PIN      PS2_CS_Pin
#define CLK_PORT    GPIOC
#define CKL_PIN     PS2_CLK_Pin
      
#define PS2_TASK_PRIO		4     //Task priority //�������ȼ�
#define PS2_STK_SIZE 		256   //Task stack size //�����ջ��С

#define DI   HAL_GPIO_ReadPin(DI_PORT, DI_PIN)    //Input pin //��������

#define DO_H HAL_GPIO_WritePin(DO_PORT, DO_PIN, GPIO_PIN_SET);   //Command height //����λ��
#define DO_L HAL_GPIO_WritePin(DO_PORT, DO_PIN, GPIO_PIN_RESET);   //Command low //����λ��

#define CS_H HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);  //Cs pull up //CS����
#define CS_L HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);  //Cs drawdown //CS����

#define CLK_H HAL_GPIO_WritePin(CLK_PORT, CKL_PIN, GPIO_PIN_SET); //Clock lift //ʱ������
#define CLK_L HAL_GPIO_WritePin(CLK_PORT, CKL_PIN, GPIO_PIN_RESET); //Clock down //ʱ������
// -----------------------------------------------
//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

#define PSS_RX 5   //Right rocker x shaft data   //��ҡ��X������
#define PSS_RY 6   //The left rocker y axis data //��ҡ��Y������
#define PSS_LX 7   //Right rocker x axis data    //��ҡ��X������
#define PSS_LY 8   //The left rocker y axis data //��ҡ��Y������

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;
void PS2_Read(void);
void PS2_Init(void);
void PS2_control(void);
uint8_t PS2_RedLight(void);   
void PS2_ReadData(void); 
void PS2_Cmd(uint8_t CMD);		 
uint8_t PS2_DataKey(void);		 
uint8_t PS2_AnologData(uint8_t button); 
void PS2_ClearData(void);	    
void PS2_Vibration(uint8_t motor1, uint8_t motor2);

void PS2_EnterConfing(void);	   
void PS2_TurnOnAnalogMode(void); 
void PS2_VibrationMode(void);  
void PS2_ExitConfing(void);	 
void PS2_SetInit(void);		    
void PS2_Receive (void);
void pstwo_task(void *pvParameters);
#endif





