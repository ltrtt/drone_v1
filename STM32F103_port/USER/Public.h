#ifndef __PUBLIC
#define __PUBLIC

#include "stm32f10x.h"
#include "stdio.h"	
#include "sys_function.h"
#include "Usart.h"
#include "sys_config.h"

#define MAX_SPEED	998



void Key_Task(void);

#pragma pack(1)
typedef struct
{
	uint8_t Head;
	uint8_t Len;
	uint8_t Cmd;
	uint16_t Motor1_PWM;
	uint16_t Motor2_PWM;
	uint16_t Motor3_PWM;
	uint16_t Motor4_PWM;
	uint16_t CRC16;
}Motor_PWM_T;
extern Motor_PWM_T Motor_PWM;
#define MOTORPWMSIZE sizeof(Motor_PWM_T)
	

extern volatile uint16_t testdelay;

#endif

