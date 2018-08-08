#include "usart2.h"
#include "usart.h"
#include "string.h"

void USART2_Task(void)
{
	Usart2_Rx_Task();
	Usart2_Tx_Task();
}

void Usart2_Tx_Task(void)
{
}

void Usart2_Rx_Task(void)
{
	uint16_t CRC16, len, CRC16T;
	if (Communication_Usart.Rx_Idle > 1 && Communication_Usart.RX_Count > 2)
	{
		Communication_Usart.RX_Count = 0;
		len = Communication_Usart.RX_DataBuff[1];
		CRC16T = CRC_16(Communication_Usart.RX_DataBuff, len + 2);
		memcpy(&CRC16, (uint8_t *)&Communication_Usart.RX_DataBuff[len + 2], 2);
		if (CRC16T == CRC16)
		{
			switch(Communication_Usart.RX_DataBuff[2])
			{
				case 2:
					memcpy((uint8_t *)&Motor_PWM, Communication_Usart.RX_DataBuff, MOTORPWMSIZE);
					break;
				
			}
		}
	}
}

