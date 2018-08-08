#include "usart.h"
#include "stdlib.h"
USARTx_Parameter_T Print_Usart, Communication_Usart;

const USART_INIT_T USART1_Init =	
{
	.RCC_APBxPeriph_USART = RCC_APB2Periph_USART1,
	.RCC_APBxPeriph_GPIO = RCC_APB2Periph_GPIOA,
	.Usartx = USART1,
	.Portx = GPIOA,
	.RX_Pin = GPIO_Pin_10,
	.TX_Pin = GPIO_Pin_9,
	.NVIC_IRQChannel = USART1_IRQn,
	.NVIC_IRQChannelPreemptionPriority = 3,
	.NVIC_IRQChannelSubPriority = 3,
	.USART_BaudRate = 115200,
	.USART_WordLength = USART_WordLength_8b,
	.USART_StopBits = USART_StopBits_1,
	.USART_Parity = USART_Parity_No,
	.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	.USART_HardwareFlowControl = USART_HardwareFlowControl_None
};

const USART_INIT_T USART2_Init =
{
	.RCC_APBxPeriph_USART = RCC_APB1Periph_USART2,
	.RCC_APBxPeriph_GPIO = RCC_APB2Periph_GPIOA,
	.Usartx = USART2,
	.Portx = GPIOA,
	.RX_Pin = GPIO_Pin_10,
	.TX_Pin = GPIO_Pin_9,
	.NVIC_IRQChannel = USART2_IRQn,
	.NVIC_IRQChannelPreemptionPriority = 1,
	.NVIC_IRQChannelSubPriority = 1,
	.USART_BaudRate = 460800,
	.USART_WordLength = USART_WordLength_8b,
	.USART_StopBits = USART_StopBits_1,
	.USART_Parity = USART_Parity_No,
	.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	.USART_HardwareFlowControl = USART_HardwareFlowControl_None
};

/**
  * @brief  Usart function initialization
  * @param  None
  * @retval None
  */
void Usart_Init(USART_INIT_T *USARTx)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(USARTx->RCC_APBxPeriph_GPIO, ENABLE);
	if (USARTx->Usartx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	else
		RCC_APB1PeriphClockCmd(USARTx->RCC_APBxPeriph_USART, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = USARTx->TX_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(USARTx->Portx, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = USARTx->RX_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(USARTx->Portx, &GPIO_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USARTx->NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USARTx->NVIC_IRQChannelPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USARTx->NVIC_IRQChannelSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = USARTx->USART_BaudRate;
	USART_InitStructure.USART_WordLength = USARTx->USART_WordLength;
	USART_InitStructure.USART_StopBits = USARTx->USART_StopBits;
	USART_InitStructure.USART_Parity = USARTx->USART_Parity;
	USART_InitStructure.USART_HardwareFlowControl = USARTx->USART_HardwareFlowControl;
	USART_InitStructure.USART_Mode = USARTx->USART_Mode;

	USART_Init(USARTx->Usartx, &USART_InitStructure);
	USART_ITConfig(USARTx->Usartx, USART_IT_RXNE, ENABLE);
	USART_Cmd(USARTx->Usartx, ENABLE);
}

void Usart_Initialization(void)		//All Usart initialization
{
	Print_Usart.Usartx_Init = (USART_INIT_T *)&USART1_Init;
	Usart_Init(Print_Usart.Usartx_Init);
	Print_Usart.RX_DataBuff = malloc(PRINTF_USART_RX_SIZE * sizeof(uint8_t));
	Print_Usart.TX_DataBuff = malloc(PRINTF_USART_TX_SIZE * sizeof(uint8_t));

	Communication_Usart.Usartx_Init = (USART_INIT_T *)&USART2_Init;
	Usart_Init(Communication_Usart.Usartx_Init);
	Communication_Usart.RX_DataBuff = malloc(COMMUNICATION_USART_RX_SIZE * sizeof(uint8_t));
	Communication_Usart.TX_DataBuff = malloc(COMMUNICATION_USART_TX_SIZE * sizeof(uint8_t));
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Print_Usart.RX_DataBuff[Print_Usart.RX_Count] = USART_ReceiveData(Print_Usart.Usartx_Init->Usartx);
		Print_Usart.RX_Count = (Print_Usart.RX_Count == (PRINTF_USART_RX_SIZE - 1)) ? 0 : (Print_Usart.RX_Count + 1);
		Print_Usart.Rx_Idle = 0;
	}
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if (Print_Usart.TX_Size > 0)
		{
			USART_SendData(USART1, Print_Usart.TX_DataBuff[Print_Usart.TX_Count]);
			if (Print_Usart.TX_Count < 256)	Print_Usart.TX_Count++;
			Print_Usart.TX_Size--;
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			Print_Usart.TX_Count = 0;
		}
	}
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET) {
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Communication_Usart.RX_DataBuff[Communication_Usart.RX_Count] = USART_ReceiveData(Communication_Usart.Usartx_Init->Usartx);
		Communication_Usart.RX_Count = (Communication_Usart.RX_Count == (PRINTF_USART_RX_SIZE - 1)) ? 0 : (Communication_Usart.RX_Count + 1);
		Communication_Usart.Rx_Idle = 0;
	}
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{
		if (Communication_Usart.TX_Size > 0)
		{
			USART_SendData(USART1, Communication_Usart.TX_DataBuff[Communication_Usart.TX_Count]);
			if (Communication_Usart.TX_Count < 256)	Communication_Usart.TX_Count++;
			Communication_Usart.TX_Size--;
		}
		else
		{
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			Communication_Usart.TX_Count = 0;
		}
	}
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET) {
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_ORE);
	}
}


