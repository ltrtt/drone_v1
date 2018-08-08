#ifndef __USART_H
#define __USART_H

/*	Add Macro define	*/
#define PRINTF_USART_RX_SIZE			512
#define PRINTF_USART_TX_SIZE			512	
#define COMMUNICATION_USART_TX_SIZE		128
#define COMMUNICATION_USART_RX_SIZE		128
#define ULTRASONIC_USART_TX_SIZE		2
#define ULTRASONIC_USART_RX_SIZE		4

/*	Add Header File	*/
//#include "public.h"
#include "stm32f4xx.h"


typedef struct 
{
	uint32_t RCC_APBxPeriph_USART;
	uint32_t RCC_AHBxPeriph_GPIO;
	USART_TypeDef* Usartx;
	GPIO_TypeDef* Portx;
	uint16_t PinSource1;
	uint16_t PinSource2;
	uint8_t AF_tar;			//¸´ÓÃÄ¿±ê
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

extern USARTx_Parameter_T Print_Usart, Ultrasonic_Usart, Communication_Usart;


void Usart_Init(USART_INIT_T *USARTx);
void Usart_Initialization(void);
uint8_t Usart_Send_Data(USARTx_Parameter_T* pUsart, uint8_t *dat, uint16_t len);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8);
void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar);


#endif

