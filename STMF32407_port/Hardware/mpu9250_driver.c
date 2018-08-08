#include "public.h"
#include "mpu9250_driver.h"

#define DELAY_US	1


short MPU_Get_Temperature(void)
{
	u8 buf[2];
	short raw;
	float temp;
	IIC_Read(MPU9250_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
	raw = ((u16)buf[0] << 8) | buf[1];
	temp = 21 + ((double)raw) / 333.87;
	return temp * 100;;
}

u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	res = IIC_Read(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*gx = ((u16)buf[0] << 8) | buf[1];
		*gy = ((u16)buf[2] << 8) | buf[3];
		*gz = ((u16)buf[4] << 8) | buf[5];
	}
	return res;;
}

u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	res = IIC_Read(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if (res == 0)
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
	}
	return res;;
}


u8 MPU_Get_Magnetometer(short *mx, short *my, short *mz)
{
	u8 buf[6], res;
	res = IIC_Read(AK8963_ADDR, MAG_XOUT_L, 6, buf);
	if (res == 0)
	{
		*mx = ((u16)buf[1] << 8) | buf[0];
		*my = ((u16)buf[3] << 8) | buf[2];
		*mz = ((u16)buf[5] << 8) | buf[4];
	}
	IIC_WriteByte(AK8963_ADDR, MAG_CNTL1, 0X11); 
	return res;;
}









































void IIC_Start(void)
{
	SDA_OUT();
	MPU_SDA_O = 1;
	MPU_SCL = 1;
	Delay_us(DELAY_US);//4
	MPU_SDA_O = 0;
	Delay_us(DELAY_US);//4
	MPU_SCL = 0;
}

void IIC_Stop(void)
{
	SDA_OUT();
	MPU_SDA_O = 0;
	MPU_SCL = 0;
	Delay_us(DELAY_US);//4
	MPU_SDA_O = 1;
	MPU_SCL = 1;
	Delay_us(DELAY_US);//4
}

uint8_t IIC_WaitACK(void)
{
	uint8_t ErrTime = 0;
	SDA_IN();
	MPU_SDA_O = 1;
	Delay_us(DELAY_US);
	MPU_SCL = 1;
	Delay_us(DELAY_US);
	while (MPU_SDA_I)
	{
		ErrTime++;
		if (ErrTime > 250)
		{
			IIC_Stop();
			return 1;
		}		
	}
	MPU_SCL = 0;
	return 0;
}

void IIC_Ack(void)
{
	MPU_SCL = 0;
	SDA_OUT();
	MPU_SDA_O = 0;
	Delay_us(DELAY_US);
	MPU_SCL = 1;
	Delay_us(DELAY_US);
	MPU_SCL = 0;
}

void IIC_NAck(void)
{
	MPU_SCL = 0;
	SDA_OUT();
	MPU_SDA_O = 1;
	Delay_us(DELAY_US);
	MPU_SCL = 1;
	Delay_us(DELAY_US);
	MPU_SCL = 0;
}

/*
	IIC发送一个字节	
*/
void IIC_SendByte(uint8_t txd)
{
	uint8_t t;
	SDA_OUT();
	MPU_SCL = 0;
	for (t = 0; t<8; t++)
	{
		MPU_SDA_O = (txd & 0x80) >> 7;
		txd <<= 1;
		Delay_us(DELAY_US);
		MPU_SCL = 1;
		Delay_us(DELAY_US);
		MPU_SCL = 0;
		Delay_us(DELAY_US);
	}
}

/*
	IIC接收一个字节
*/
uint8_t IIC_ReceiveByte(uint8_t ack)
{
	uint8_t i, receive = 0;
	SDA_IN();
	for (i = 0; i < 8; i++)
	{
		MPU_SCL = 0;
		Delay_us(DELAY_US);
		MPU_SCL = 1;
		receive <<= 1;
		if (MPU_SDA_I)	receive++;
		Delay_us(DELAY_US);
	}
	if (!ack)
		IIC_NAck();
	else
		IIC_Ack();
	return receive;
}

/*
	IIC从指定位置读数据
*/
uint8_t IIC_ReadByte(uint8_t devaddr, uint8_t addr)
{
	uint8_t temp = 0;
	IIC_Start();
	IIC_SendByte(devaddr);  
	IIC_WaitACK();
	IIC_SendByte(addr);  
	IIC_WaitACK();

	IIC_Start();
	IIC_SendByte(devaddr | 1);	   
	IIC_WaitACK();
	temp = IIC_ReceiveByte(0);
	IIC_Stop();	    
	return temp;
}

/*
	IIC从指定位置连续读多个字节的数据
*/
uint8_t IIC_Read(uint8_t devaddr, uint8_t addr, uint8_t len, uint8_t *buf)
{
	int i = 0;
	IIC_Start();
	IIC_SendByte((devaddr << 1) | 0);
	if (IIC_WaitACK())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(addr);
	IIC_WaitACK();

	IIC_Start();
	IIC_SendByte((devaddr << 1) | 1);
	IIC_WaitACK();
	for (i = 0; i < len; i++)
	{
		if (i == len - 1)
		{
			buf[i] = IIC_ReceiveByte(0);
		}
		else
			buf[i] = IIC_ReceiveByte(1);
	}
	IIC_Stop();
	return 0;
}

/*
	IIC从指定位置写一个字节
*/
void IIC_WriteByte(uint8_t Devaddr, uint8_t addr, uint8_t data)
{
	IIC_Start();
	IIC_SendByte(Devaddr); 
	IIC_WaitACK();
	IIC_SendByte(addr);
	IIC_WaitACK();
	IIC_SendByte(data);						   
	IIC_WaitACK();
	IIC_Stop();
}

/*
	IIC从指定位置连续写多个字节
*/
uint8_t IIC_Write(uint8_t Devaddr, uint8_t addr, uint8_t len, uint8_t *buf)
{
	int i = 0;
	IIC_Start();
	IIC_SendByte((Devaddr << 1) | 0);
	if (IIC_WaitACK())
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(addr); 
	IIC_WaitACK();
	for (i = 0; i < len; i++)
	{
		IIC_SendByte(buf[i]);
		if (IIC_WaitACK())
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_Stop();
	return 0;
}










void mget_ms(unsigned long *time)
{
    //*time=(unsigned long)HAL_GetTick();
}

void Delay_Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInitStruct.TIM_Period = 100 - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = (84 - 1);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);


	while ((TIM3->SR & TIM_FLAG_Update) != SET);
	TIM3->SR = (uint16_t)~TIM_FLAG_Update;
}



void Delay_us(uint32_t us_cnt)
{
	TIM3->CNT = us_cnt - 1;
	TIM3->CR1 |= TIM_CR1_CEN;
	while ((TIM3->SR & TIM_FLAG_Update) != SET);
	TIM3->SR = (uint16_t)~TIM_FLAG_Update;
	TIM3->CR1 &= ~TIM_CR1_CEN;
}
/*
void Delay_ms(uint32_t ms_cnt)
{
	for (uint32_t i = 0; i < ms_cnt; i++)	
		Delay_us(1000);
}*/

