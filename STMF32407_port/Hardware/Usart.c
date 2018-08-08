#include "Usart.h"
#include "usart2.h"
#include "stdlib.h"
#include "string.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


USARTx_Parameter_T Print_Usart, Communication_Usart, Ultrasonic_Usart;


const USART_INIT_T USART1_Init =
{
	.RCC_APBxPeriph_USART = RCC_APB2Periph_USART1,
	.RCC_AHBxPeriph_GPIO = RCC_AHB1Periph_GPIOA,
	.Usartx = USART1,
	.Portx = GPIOA,
	.PinSource1 = GPIO_PinSource9,
	.PinSource2 = GPIO_PinSource10,
	.AF_tar = GPIO_AF_USART1,
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

const USART_INIT_T USART3_Init =
{
	.RCC_APBxPeriph_USART = RCC_APB1Periph_USART3,
	.RCC_AHBxPeriph_GPIO = RCC_AHB1Periph_GPIOC,
	.Usartx = USART3,
	.Portx = GPIOC,
	.PinSource1 = GPIO_PinSource11,
	.PinSource2 = GPIO_PinSource10,
	.AF_tar = GPIO_AF_USART3,
	.RX_Pin = GPIO_Pin_11,
	.TX_Pin = GPIO_Pin_10,
	.NVIC_IRQChannel = USART3_IRQn,
	.NVIC_IRQChannelPreemptionPriority = 1,
	.NVIC_IRQChannelSubPriority = 1,
	.USART_BaudRate = 460800,
	.USART_WordLength = USART_WordLength_8b,
	.USART_StopBits = USART_StopBits_1,
	.USART_Parity = USART_Parity_No,
	.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	.USART_HardwareFlowControl = USART_HardwareFlowControl_None
};

const USART_INIT_T USART6_Init = {
	.RCC_APBxPeriph_USART = RCC_APB2Periph_USART6,
	.RCC_AHBxPeriph_GPIO = RCC_AHB1Periph_GPIOC,
	.Usartx = USART6,
	.Portx = GPIOC,
	.PinSource1 = GPIO_PinSource6,
	.PinSource2 = GPIO_PinSource7,
	.AF_tar = GPIO_AF_USART6,
	.RX_Pin = GPIO_Pin_7,
	.TX_Pin = GPIO_Pin_6,
	.NVIC_IRQChannel = USART6_IRQn,
	.NVIC_IRQChannelPreemptionPriority = 1,
	.NVIC_IRQChannelSubPriority = 2,
	.USART_BaudRate = 9600,
	.USART_WordLength = USART_WordLength_8b,
	.USART_StopBits = USART_StopBits_1,
	.USART_Parity = USART_Parity_No,
	.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	.USART_HardwareFlowControl = USART_HardwareFlowControl_None
};

uint8_t data_to_send[30];
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt = 0;
	vs16 _temp;
	vs32 _temp2 = alt;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x01;
	data_to_send[_cnt++] = 0;

	_temp = (int)(angle_rol * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(angle_pit * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = (int)(angle_yaw * 100);
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[_cnt++] = BYTE3(_temp2);
	data_to_send[_cnt++] = BYTE2(_temp2);
	data_to_send[_cnt++] = BYTE1(_temp2);
	data_to_send[_cnt++] = BYTE0(_temp2);

	data_to_send[_cnt++] = fly_model;

	data_to_send[_cnt++] = armed;

	data_to_send[3] = _cnt - 4;

	u8 sum = 0;
	for (u8 i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	Usart_Send_Data(&Print_Usart, data_to_send, _cnt);
	//ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_MotoPWM(u16 m_1, u16 m_2, u16 m_3, u16 m_4, u16 m_5, u16 m_6, u16 m_7, u16 m_8)
{
	u8 _cnt = 0;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x06;
	data_to_send[_cnt++] = 0;

	data_to_send[_cnt++] = BYTE1(m_1);
	data_to_send[_cnt++] = BYTE0(m_1);
	data_to_send[_cnt++] = BYTE1(m_2);
	data_to_send[_cnt++] = BYTE0(m_2);
	data_to_send[_cnt++] = BYTE1(m_3);
	data_to_send[_cnt++] = BYTE0(m_3);
	data_to_send[_cnt++] = BYTE1(m_4);
	data_to_send[_cnt++] = BYTE0(m_4);
	data_to_send[_cnt++] = BYTE1(m_5);
	data_to_send[_cnt++] = BYTE0(m_5);
	data_to_send[_cnt++] = BYTE1(m_6);
	data_to_send[_cnt++] = BYTE0(m_6);
	data_to_send[_cnt++] = BYTE1(m_7);
	data_to_send[_cnt++] = BYTE0(m_7);
	data_to_send[_cnt++] = BYTE1(m_8);
	data_to_send[_cnt++] = BYTE0(m_8);

	data_to_send[3] = _cnt - 4;

	u8 sum = 0;
	for (u8 i = 0; i<_cnt; i++)
		sum += data_to_send[i];

	data_to_send[_cnt++] = sum;

	Usart_Send_Data(&Print_Usart, data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x, s16 a_y, s16 a_z, s16 g_x, s16 g_y, s16 g_z, s16 m_x, s16 m_y, s16 m_z, s32 bar)
{
	u8 _cnt = 0;
	vs16 _temp;

	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;

	_temp = a_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = g_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = m_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_z;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;

	u8 sum = 0;
	for (u8 i = 0; i<_cnt; i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	Usart_Send_Data(&Print_Usart, data_to_send, _cnt);
}


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

	RCC_AHB1PeriphClockCmd(USARTx->RCC_AHBxPeriph_GPIO, ENABLE);
	if (USARTx->Usartx == USART1 || USARTx->Usartx == USART6)
		RCC_APB2PeriphClockCmd(USARTx->RCC_APBxPeriph_USART, ENABLE);
	else
		RCC_APB1PeriphClockCmd(USARTx->RCC_APBxPeriph_USART, ENABLE);

	GPIO_PinAFConfig(USARTx->Portx, USARTx->PinSource1, USARTx->AF_tar);
	GPIO_PinAFConfig(USARTx->Portx, USARTx->PinSource2, USARTx->AF_tar);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = USARTx->TX_Pin | USARTx->RX_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
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

	Communication_Usart.Usartx_Init = (USART_INIT_T *)&USART3_Init;
	Usart_Init(Communication_Usart.Usartx_Init);
	Communication_Usart.RX_DataBuff = malloc(COMMUNICATION_USART_RX_SIZE * sizeof(uint8_t));
	Communication_Usart.TX_DataBuff = malloc(COMMUNICATION_USART_TX_SIZE * sizeof(uint8_t));
	
	Ultrasonic_Usart.Usartx_Init = (USART_INIT_T *)&USART6_Init;
	Usart_Init(Ultrasonic_Usart.Usartx_Init);
	Ultrasonic_Usart.RX_DataBuff = malloc(ULTRASONIC_USART_RX_SIZE * sizeof(uint8_t));
	Ultrasonic_Usart.TX_DataBuff = malloc(ULTRASONIC_USART_TX_SIZE * sizeof(uint8_t));

	Motor_PWM.Len = MOTORPWMSIZE - 4;
	Motor_PWM.Cmd = 2;
	Motor_PWM.Head = 0x12;
}

void USART_Send(USARTx_Parameter_T *pUsart)
{
	USART_ITConfig(pUsart->Usartx_Init->Usartx, USART_IT_TXE, ENABLE);
}

uint8_t Usart_Send_Data(USARTx_Parameter_T* pUsart, uint8_t *dat, uint16_t len)
{
	pUsart->TX_Count = 0;
	pUsart->TX_Size = len;
	memcpy(pUsart->TX_DataBuff, dat, len);
	USART_Send(pUsart);
	return 1;
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

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(Communication_Usart.Usartx_Init->Usartx, USART_IT_RXNE) != RESET)
	{
		Communication_Usart.RX_DataBuff[Communication_Usart.RX_Count] = USART_ReceiveData(Communication_Usart.Usartx_Init->Usartx);
		Communication_Usart.RX_Count = (Communication_Usart.RX_Count == (COMMUNICATION_USART_TX_SIZE - 1)) ? 0 : (Communication_Usart.RX_Count + 1);
		Communication_Usart.Rx_Idle = 0;
	}
	if (USART_GetITStatus(Communication_Usart.Usartx_Init->Usartx, USART_IT_TXE) != RESET)
	{
		if (Communication_Usart.TX_Size > 0)
		{
			USART_SendData(Communication_Usart.Usartx_Init->Usartx, Communication_Usart.TX_DataBuff[Communication_Usart.TX_Count]);
			if (Communication_Usart.TX_Count < 256)	Communication_Usart.TX_Count++;
			Communication_Usart.TX_Size--;
		}
		else
		{
			USART_ITConfig(Communication_Usart.Usartx_Init->Usartx, USART_IT_TXE, DISABLE);
			Communication_Usart.TX_Count = 0;
		}
	}
	if (USART_GetFlagStatus(Communication_Usart.Usartx_Init->Usartx, USART_FLAG_ORE) == SET) {
		USART_ReceiveData(Communication_Usart.Usartx_Init->Usartx);
		USART_ClearFlag(Communication_Usart.Usartx_Init->Usartx, USART_FLAG_ORE);
	}
}

void USART6_IRQHandler(void)
{
	if (USART_GetITStatus(Ultrasonic_Usart.Usartx_Init->Usartx, USART_IT_RXNE) != RESET)
	{
		Ultrasonic_Usart.RX_DataBuff[Ultrasonic_Usart.RX_Count] = USART_ReceiveData(Ultrasonic_Usart.Usartx_Init->Usartx);
		Ultrasonic_Usart.RX_Count = (Ultrasonic_Usart.RX_Count == (COMMUNICATION_USART_TX_SIZE - 1)) ? 0 : (Ultrasonic_Usart.RX_Count + 1);
		Ultrasonic_Usart.Rx_Idle = 0;
	}
	if (USART_GetITStatus(Ultrasonic_Usart.Usartx_Init->Usartx, USART_IT_TXE) != RESET)
	{
		if (Ultrasonic_Usart.TX_Size > 0)
		{
			USART_SendData(Ultrasonic_Usart.Usartx_Init->Usartx, Ultrasonic_Usart.TX_DataBuff[Ultrasonic_Usart.TX_Count]);
			if (Ultrasonic_Usart.TX_Count < 256)	Ultrasonic_Usart.TX_Count++;
			Ultrasonic_Usart.TX_Size--;
		}
		else
		{
			USART_ITConfig(Ultrasonic_Usart.Usartx_Init->Usartx, USART_IT_TXE, DISABLE);
			Ultrasonic_Usart.TX_Count = 0;
		}
	}
	if (USART_GetFlagStatus(Ultrasonic_Usart.Usartx_Init->Usartx, USART_FLAG_ORE) == SET) {
		USART_ReceiveData(Ultrasonic_Usart.Usartx_Init->Usartx);
		USART_ClearFlag(Ultrasonic_Usart.Usartx_Init->Usartx, USART_FLAG_ORE);
	}
}