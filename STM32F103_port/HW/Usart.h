#ifndef __USART_H
#define __USART_H

/*	Add Macro define	*/
#define PRINTF_USART_RX_SIZE			512
#define PRINTF_USART_TX_SIZE			512	
#define COMMUNICATION_USART_TX_SIZE		128
#define COMMUNICATION_USART_RX_SIZE		128




/*	Add Header File	*/
#include "public.h"







typedef struct 
{
	uint32_t RCC_APBxPeriph_USART;
	uint32_t RCC_APBxPeriph_GPIO;
	USART_TypeDef* Usartx;
	GPIO_TypeDef* Portx;
	uint16_t RX_Pin;
	uint16_t TX_Pin;
	uint8_t NVIC_IRQChannel;
	uint8_t NVIC_IRQChannelPreemptionPriority;
	uint8_t NVIC_IRQChannelSubPriority;
	uint32_t USART_BaudRate;
	uint16_t USART_WordLength;
	uint16_t USART_StopBits;
	uint16_t USART_Parity;
	uint16_t USART_Mode;
	uint16_t USART_HardwareFlowControl;
}USART_INIT_T;

typedef struct
{
	USART_INIT_T *Usartx_Init;
	uint8_t *RX_DataBuff;
	uint8_t *TX_DataBuff;
	uint16_t RX_Count;
	uint16_t TX_Size;
	uint16_t TX_Count;
	uint16_t Rx_Idle;
	volatile uint16_t Tx_Delay;
}USARTx_Parameter_T;

extern USARTx_Parameter_T Print_Usart, Communication_Usart;


void Usart_Init(USART_INIT_T *USARTx);
void Usart_Initialization(void);


#endif

