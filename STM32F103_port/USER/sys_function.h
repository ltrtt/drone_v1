#ifndef	__SYS_FUNCTION
#define __SYS_FUNCTION

#include "public.h"
#include "sys_config.h"

typedef struct 
{
	uint32_t RCC_APB2Periph_GPIOx;
	GPIO_TypeDef* Portx;
	uint16_t Pinx;
	GPIOSpeed_TypeDef GPIO_Speed;
	GPIOMode_TypeDef GPIO_Mode;
}GPIO_INIT_T;


void Systick_Init(void);
void SysTick_Handler(void);
void GPIOx_Init(GPIO_INIT_T *GPIO_init);
void System_Initialization(void);
uint8_t Key_Scan(LoopState loop);
void System_Standby(RunState state);
uint16_t CRC_16(uint8_t *ptr, uint16_t n);







#endif

