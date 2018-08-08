#include "public.h"
#include "usart2.h"
#include "motorcontrol.h"
Sys_Para_t Sys_Para;
Motor_Cotrol_t Motor_Cotrol;
Motor_PWM_T Motor_PWM;
volatile uint16_t testdelay;

#define ARR(x)	(x - 1)

void Led_Task(void);
uint8_t pscControl;
uint16_t pscT, arrT, sped;
uint8_t setyoumen, pwrControl, testControl;
int main(void)
{
	//NVIC_SystemReset();

	System_Initialization();
	
	pscT = 72;
	arrT = 2000;
	Monoter_Control_Init(ARR(arrT), pscT);
	if (setyoumen)
		TIM_SetCompare4(TIM4, 1999);
	System_Standby(BOOT);
	while(1)
	{
		Led_Task();
		Key_Task();
//		if (pwrControl)
//			POWER_CONTROL1 = 0;
//		else
//			POWER_CONTROL1 = 1;
		if (pscControl)
		{
			pscControl = 0;
			Monoter_Control_Init(ARR(arrT), pscT);
		}
		if (testControl == 0)
			USART2_Task();
		SpeedControl();
	}
}

void Key_Task(void)
{
	Sys_Para.Key_Value = Key_Scan(NOLOOP);
	if (Sys_Para.Key_Value == 2)
	{
		System_Standby(RUN);
	}
}

void Led_Task(void)
{
	if (Sys_Para.Led_Delay == 0)
	{
		Sys_Para.Led_State = !Sys_Para.Led_State;
		LED_STATE = Sys_Para.Led_State;
		Sys_Para.Led_Delay = 500;
	}
}
