#ifndef	__SYS_FUNCTION
#define __SYS_FUNCTION

#include "public.h"
#include "sys_config.h"


typedef struct {
	uint32_t RCC_AHB1Periph_GPIOx;
	GPIO_TypeDef* Portx;
	uint16_t Pinx;
	GPIOSpeed_TypeDef GPIO_Speed;
	GPIOOType_TypeDef GPIO_OType;
	GPIOPuPd_TypeDef GPIO_PuPd;
	GPIOMode_TypeDef GPIO_Mode;
}GPIO_INIT_T;

typedef struct {
	short Gyro_X;
	short Gyro_Y;
	short Gyro_Z;
}Gyro_Para_T;

typedef struct {
	short Accel_X;
	short Accel_Y;
	short Accel_Z;
}Accel_Para_T;

typedef struct {
	short Compass_X;
	short Compass_Y;
	short Compass_Z;
}Compass_Para_T;

typedef struct
{
	long			Quaternions[4];
	Gyro_Para_T		Gyro_para;
	Accel_Para_T	Accel_para;
	Compass_Para_T	Compass_para;
}Attitude_Parameter_T;
extern Attitude_Parameter_T Attitude_Parameter;



extern uint16_t TimeTick;

void Systick_Init(void);
void SysTick_Handler(void);
void GPIOx_Init(GPIO_INIT_T *GPIO_init);
void System_Initialization(void);
uint16_t CRC_16(uint8_t *ptr, uint16_t n);
void Delay_ms(uint16_t time);








#endif

