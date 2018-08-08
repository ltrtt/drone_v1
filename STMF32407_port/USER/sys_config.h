#ifndef __SYS_CONFIG
#define __SYS_CONFIG

#include "public.h"
#include "GPIO_bit.h"
/*		Macro Defenition	*/	



/*		GPIO Configration	*/
#define MPU_SCL		PAout(1)
#define MPU_SDA_O	PAout(2)
#define MPU_SDA_I	PAin(2)

#define NRF2401_CE	PAout(8)
#define NRF2401_CS	PAout(4)

#define WIFI_EN		PDout(4)


/*
	USART1 is used to print system state
*/

/*
	USART2 is used to communication with STM32F4 microcontroller
*/



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


