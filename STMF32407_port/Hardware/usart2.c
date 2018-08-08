#include "usart2.h"
#include "string.h"
uint8_t Usart2_Cmd[8], Zero[8];
//volatile uint8_t Usart2SendDelay;
void USART2_Task(void)
{
	Usart2_Tx_Task();
	Usart2_Rx_Task();
}

void Usart2_Tx_Task(void)
{
	if (memcmp(Usart2_Cmd, Zero, 8) != 0 && Communication_Usart.Tx_Delay == 0)
	{
		do
		{
			if (SEND_MOTOR_PARA)
			{
				SEND_MOTOR_PARA = 0;
				Motor_PWM.CRC_16 = CRC_16((uint8_t *)&Motor_PWM, MOTORPWMSIZE - 2);
				Usart_Send_Data(&Communication_Usart, (uint8_t *)&Motor_PWM, MOTORPWMSIZE);
				break;
			}
		}while(0);
		Communication_Usart.Tx_Delay = 2;
	}
}

void Usart2_Rx_Task(void)
{
	
}

void Usart6_Tx_Task(void)
{
	uint8_t sendbuf = 0x55;
	if (Sensor_Para.Ultrasonic_SamplingFlag && Ultrasonic_Usart.Tx_Delay == 0)
	{
		Usart_Send_Data(&Ultrasonic_Usart, &sendbuf, 1);
		Ultrasonic_Usart.Tx_Delay = 10;
	}
}

void Usart6_Rx_Task(void)
{
	static uint16_t Pn, PN_1;
	uint16_t position;
	if (Ultrasonic_Usart.RX_Count > 1 && Ultrasonic_Usart.Rx_Idle > 5)
	{
		Ultrasonic_Usart.RX_Count = 0;
//		position = Ultrasonic_Usart.RX_DataBuff[0] * 256 + Ultrasonic_Usart.RX_DataBuff[1];
//		if (((position - Sensor_Para.Ultra_Hight) < 20) && ((Sensor_Para.Ultra_Hight - position) < 30))
		Sensor_Para.Ultra_Hight = Ultrasonic_Usart.RX_DataBuff[0] * 256 + Ultrasonic_Usart.RX_DataBuff[1];
		Sensor_Para.Ultrasonic_SamplingFlag = 0;
		Pn = Sensor_Para.Ultra_Hight;
		Sensor_Para.Ascending_Velocity = (float)(Pn - PN_1) / 50;
		
//		if (Sensor_Para.Ascending_Velocity > 40 || Sensor_Para.Ascending_Velocity < -40)
//			while(1);
		PN_1 = Pn;
	}
}