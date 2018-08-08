#include "sys_function.h"

const GPIO_INIT_T POWER_GPIO_INIT = {
	.RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOB,
	.Portx = GPIOB,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_Mode = GPIO_Mode_Out_PP,
	.Pinx = GPIO_Pin_0 | GPIO_Pin_1
};

const GPIO_INIT_T LED_STATE_GPIO_INIT = {
	.RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOA,
	.Portx = GPIOA,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_Mode = GPIO_Mode_Out_PP,
	.Pinx = GPIO_Pin_4
};

const GPIO_INIT_T LED_GPIO_INIT = {
	.RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOC,
	.Portx = GPIOC,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_Mode = GPIO_Mode_Out_PP,
	.Pinx = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
};

const GPIO_INIT_T POWER_KEY_GPIO_INIT = {
	.RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOA,
	.Portx = GPIOA,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_Mode = GPIO_Mode_IPD,
	.Pinx = GPIO_Pin_0
};
/*
#pragma import(__use_no_semihosting)               
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

};

FILE __stdout;  
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
}    */


int fputc(int ch, FILE *f)
{      
//	while((USART1->SR & 0X40) == 0){};
    USART1->DR = (uint8_t) (ch & 0x000000FF);    
	while (!(USART1->SR & USART_FLAG_TXE));		
	return ch;
}



/**
  *	@breif  SysTick initialization
  * @param  None
  * @retval None
  */
void Systick_Init(void)
{
	uint32_t Frequence;
	RCC_ClocksTypeDef Clock;
	RCC_GetClocksFreq(&Clock);
	Frequence = Clock.SYSCLK_Frequency;
	while(SysTick_Config(Frequence / 1000));	
}

/**
  * @brief  systick interrupt handler
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if (Sys_Para.Key_Delay)	Sys_Para.Key_Delay--;
	if (Sys_Para.Key_Press_Enable && (Sys_Para.Key_Press_Count < 20000))	Sys_Para.Key_Press_Count++;
	if (testdelay)		testdelay--;
	if (Sys_Para.Led_Delay)	Sys_Para.Led_Delay--;
	if (Communication_Usart.Rx_Idle < 1000)	Communication_Usart.Rx_Idle++;
	if (Communication_Usart.Tx_Delay)	Communication_Usart.Tx_Delay--;
}

/**
  * @brief  GPIO initialization
  * @param  None
  * @retval None
  */
void GPIOx_Init(GPIO_INIT_T *GPIO_init)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(GPIO_init->RCC_APB2Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_init->Pinx;
	GPIO_InitStructure.GPIO_Speed = GPIO_init->GPIO_Speed;
	GPIO_InitStructure.GPIO_Mode = GPIO_init->GPIO_Mode;

	GPIO_Init(GPIO_init->Portx, &GPIO_InitStructure);
}

/**
  * @brief  System initialization
  * @param  None
  * @retval None
  */
void System_Initialization(void)
{
	GPIOx_Init((GPIO_INIT_T *)&POWER_KEY_GPIO_INIT);
	GPIOx_Init((GPIO_INIT_T *)&POWER_GPIO_INIT);
	GPIOx_Init((GPIO_INIT_T *)&LED_STATE_GPIO_INIT);
	GPIOx_Init((GPIO_INIT_T *)&LED_GPIO_INIT);
	POWER_CONTROL1 = 1;
	POWER_CONTROL2 = 1;
	LED_STATE = 1;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Systick_Init();
	Usart_Initialization();
	printf("\r\n");
	printf("                Welecome to Use WPM\r\n");
	printf("  This is PCP(Power Control Part), this part is	responsible	\r\n");
	printf("for controlling the power supply of the	flight control		\r\n");
	printf("system, and motor power control.							\r\n");

	printf("  The WPM will enter standby mode after connect battery,	\r\n");
	printf("please press the button for 1 second to start WPM, and		\r\n");
	printf("press the button for 3 second to close WPM.					\r\n");

	printf("  When WPM works, this usart will printf the parameters to	\r\n");
	printf("the motor power.											\r\n");

	printf("  If you have any quesqion about how to use WPM, please to	\r\n");
	printf("contact me in following way. I will answer your questions	\r\n");
	printf("in time.													\r\n");
	printf("QQ: 475182290												\r\n");
	printf("E-mail: 475182290@qq.com									\r\n");
}

/**
  * @brief  Key scan function
  * @param  loop: 0: the function enter endless loop
  *				  1: the function does not enter endless loop
  * @retval 0: No key press
  *			1: The power key short press
  *			2: The power key long press
  */
uint8_t Key_Scan(LoopState loop)
{
	uint8_t Return_Val = 0;
	static uint8_t Key_Step = 0;
	do
	{
		if ((Sys_Para.Key_Delay > 0) && (loop == 0))	return Return_Val;	// Used for debouncing

		switch (Key_Step)
		{
		case 0:
			if (POWER_KEY)						// If the power key is pressed for the first time
			{
				Key_Step = 1;					// Jump to next step
				Sys_Para.Key_Delay = 30;		// debounce time equal thiry millisecond
			}
			else
				return 0;
			break;
		case 1:
			Key_Step = 0;						// If the first press is bounced, step return to 0
			Sys_Para.Key_Press_Count = 0;
			if (POWER_KEY)						// If the press is not bounce, preform the key function
			{
				Sys_Para.Key_Press_Enable = 1;
				Key_Step = 2;
			}
			break;
		case 2:
			if (POWER_KEY == 0)					// If the power key is release
			{
				Key_Step = 0;					// The step return to 0, waitting to next press
				Sys_Para.Key_Press_Enable = 0;
				if ((SHORT_PRESS < Sys_Para.Key_Press_Count) && (Sys_Para.Key_Press_Count < LONG_PRESS))		// If the power key short press
				{
					Sys_Para.Key_Press_Count = 0;
					Return_Val = 1;
					return Return_Val;									// Return the press type
				}
				else if (Sys_Para.Key_Press_Count >= LONG_PRESS)		// If the power key long press
				{
					Sys_Para.Key_Press_Count = 0;
					Return_Val = 2;
					return Return_Val;									// Return the press type
				}
			}
			break;
		default:
			Key_Step = 0;
			break;
		}
	}while(loop);
	return Return_Val;						// Return the press type
}

/**
  * @brief  system standby mode
  * @param  state: 0: this function is used at boot
  *				   1: this function is used after startup
  * @retval None
  */
void System_Standby(RunState state)
{
//	uint32_t i = 10000000;
	if (state == BOOT)													// If the function is used at boot
	{
		if (Key_Scan(LOOP))												// If the buttom press for more than 1 second, enable the WPM
		{
			printf("System online\r\n");

			POWER_CONTROL1 = 0;
			POWER_CONTROL2 = 0;
			LED_STATE = 0;
		}
		else															// If the buttom is not pressed
		{
			POWER_CONTROL1 = 1;
			POWER_CONTROL2 = 1;
			printf("System enter stanby mode!\r\n");
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);			// Enable the Power-Register clock
			PWR_WakeUpPinCmd(ENABLE);									// Enables the WakeUp Pin functionality.
			PWR_EnterSTANDBYMode();										// Enter the Standby mode
		}	
	}
	else																// If the function is used after startup
	{
		POWER_CONTROL1 = 1;
		POWER_CONTROL2 = 1;
//		printf("System enter stanby mode!\r\n");
//		while(i--);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);			// Enable the Power-Register clock
		PWR_WakeUpPinCmd(ENABLE);									// Enables the WakeUp Pin functionality.
		PWR_EnterSTANDBYMode();										// Enter the Standby mode
	}
}

uint16_t CRC_16(uint8_t *ptr, uint16_t n)
{
	uint16_t i;
	uint16_t crc = 0xFFFF;
	do {
		crc ^= *ptr++;
		for (i = 8; i != 0; i--)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else crc >>= 1;
		}
	} while (--n);
	return crc;
}

