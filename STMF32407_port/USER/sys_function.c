#include "sys_function.h"
#include "mpu9250_driver.h"
#include "adc.h"
uint16_t TimeTick;
const GPIO_INIT_T WiFiPWR_GPIO_INIT = {
	.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOD,
	.Portx = GPIOD,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_OType = GPIO_OType_PP,
	.GPIO_Mode = GPIO_Mode_OUT,
	.GPIO_PuPd = GPIO_PuPd_NOPULL,
	.Pinx = GPIO_Pin_4
};

const GPIO_INIT_T LED_STATE_GPIO_INIT = {
	.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA,
	.Portx = GPIOA,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_OType = GPIO_OType_PP,
	.GPIO_Mode = GPIO_Mode_OUT,
	.GPIO_PuPd = GPIO_PuPd_UP,
	.Pinx = GPIO_Pin_4
};

const GPIO_INIT_T LED_GPIO_INIT = {
	.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOC,
	.Portx = GPIOC,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_OType = GPIO_OType_PP,
	.GPIO_Mode = GPIO_Mode_OUT,
	.GPIO_PuPd = GPIO_PuPd_UP,
	.Pinx = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
};

const GPIO_INIT_T POWER_KEY_GPIO_INIT = {
	.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA,
	.Portx = GPIOA,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_OType = GPIO_OType_PP,
	.GPIO_Mode = GPIO_Mode_IN,
	.GPIO_PuPd = GPIO_PuPd_UP,
	.Pinx = GPIO_Pin_0
};

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
} 

//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR & 0X40) == 0){};
    USART1->DR = (uint8_t) ch;      
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

uint16_t second;
void TimeTickUpdate(void)
{
	TimeTick++;
	if (TimeTick == 1000)
	{
		second++;
		TimeTick = 0;
	}
}

uint16_t DelayMsCount;

void Delay_ms(uint16_t time)
{
	DelayMsCount = time;
	while(DelayMsCount);
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
//	if (time)	time--;
//	g_ul_ms_ticks++;
	TimeTickUpdate();	
	if (DelayMsCount) DelayMsCount--;
	if (Communication_Usart.Tx_Delay)	Communication_Usart.Tx_Delay--;
	if (Ultrasonic_Usart.Tx_Delay)		Ultrasonic_Usart.Tx_Delay--;
	if (adc_samplate_delay)	adc_samplate_delay--;
	NoThrottleCount++;
	if (Ultrasonic_Usart.Rx_Idle < 100)	Ultrasonic_Usart.Rx_Idle++;
}


/**
  * @brief  GPIO initialization
  * @param  None
  * @retval None
  */
void GPIOx_Init(GPIO_INIT_T *GPIO_init)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(GPIO_init->RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Pin   = GPIO_init->Pinx;
	GPIO_InitStructure.GPIO_Speed = GPIO_init->GPIO_Speed;
	GPIO_InitStructure.GPIO_Mode  = GPIO_init->GPIO_Mode;
	GPIO_InitStructure.GPIO_OType = GPIO_init->GPIO_OType;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_init->GPIO_PuPd;
	GPIO_Init(GPIO_init->Portx, &GPIO_InitStructure);
}

/**
  * @brief  System initialization
  * @param  None
  * @retval None
  */
void System_Initialization(void)
{
	/*GPIOx_Init((GPIO_INIT_T *)&POWER_KEY_GPIO_INIT);
	GPIOx_Init((GPIO_INIT_T *)&POWER_GPIO_INIT);
	GPIOx_Init((GPIO_INIT_T *)&LED_STATE_GPIO_INIT);
	GPIOx_Init((GPIO_INIT_T *)&LED_GPIO_INIT);*/

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Systick_Init();
	Usart_Initialization();
	Delay_Timer_Init();
	GPIOx_Init((GPIO_INIT_T *)&WiFiPWR_GPIO_INIT);
	printf("\r\n");
	printf("                Welecome to Use WPM\r\n");
	printf("  This is PCP(Power Control Part), the part is responsible	\r\n");
	printf("for controlling the power supply of the	flight control		\r\n");
	printf("system, and motor power control.							\r\n");

	printf("  The WPM will enter standby mode after connect battery,	\r\n");
	printf("please press the button for 1 second to start WPM, and		\r\n");
	printf("press the button for 3 second to close WPM.					\r\n");

	printf("  When WPM works, this usart will printf the parameters to	\r\n");
	printf("the motor power.											\r\n");

	printf("  If you have any quesqion about use WPM, please to	contact \r\n");
	printf("me in following way. I will reply you as soon as possible	\r\n");
	printf("QQ: 475182290												\r\n");
	printf("E-mail: 475182290@qq.com									\r\n");
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

