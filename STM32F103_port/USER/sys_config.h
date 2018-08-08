#ifndef __SYS_CONFIG
#define __SYS_CONFIG

#include "public.h"
#include "GPIO_bit.h"
/*		Macro Defenition	*/	
#define SHORT_PRESS	1000
#define LONG_PRESS	2000
typedef enum { NOLOOP = 0, LOOP } LoopState;
typedef enum { BOOT = 0, RUN } RunState;


/*		GPIO Configration	*/
#define POWER_CONTROL1	PBout(0)		// output 0 is enable the power
#define POWER_CONTROL2	PBout(1)		// output 0 is enable the power

#define LED_STATE		PAout(4)		// output 0 is enable the LED_STATE
#define LED1			PCout(0)		// output 0 is enable the LED1
#define LED2			PCout(1)		// output 0 is enable the LED2
#define LED3			PCout(2)		// output 0 is enable the LED3
#define LED4			PCout(3)		// output 0 is enable the LED4

#define POWER_KEY		PAin(0)			// input Key

/*
	USART1 is used to print system state
*/

/*
	USART2 is used to communication with STM32F4 microcontroller
*/

typedef struct
{
	uint8_t Led_State;				// The STM32F1 current state
	uint8_t ESC_Power_Enable;		// The ESC power control
	uint8_t STM32F4_Power_Enable;	// The STM32F4 power control
	volatile uint16_t Led_Delay;	// Used for STM32F1 state led flash
	volatile uint16_t Key_Delay;	// Used for key delay
	uint16_t Key_Press_Count;		// Used for record the key press time
	uint8_t Key_Press_Enable;		// Used for enable the 'Key_Press_Count' founction
	uint8_t Key_Value;
}Sys_Para_t;
extern Sys_Para_t Sys_Para;

typedef struct
{
	uint16_t Moto1_PWM;				// The motor1 speed
	uint16_t Moto2_PWM;				// The motor2 speed
	uint16_t Moto3_PWM;				// The motor3 speed
	uint16_t Moto4_PWM;				// The motor4 speed
}Motor_Cotrol_t;
#define MOTOR_CONTROL_SIZE	sizeof(Motor_Cotrol_t)
extern Motor_Cotrol_t Motor_Cotrol;
#endif // !__SYS_CONFIG


