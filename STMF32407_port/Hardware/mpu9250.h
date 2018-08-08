#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f4xx.h"


#define MPU9250_Init_RetryTime	5

uint8_t run_self_test(void);

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)
#define COMPASS_READ_MS (100)

























typedef struct
{
	float pitch;		// ������
	float roll;			// �����
	float yaw;			// ƫ����
}Euler_t;	// ŷ����

typedef struct
{
	float x;
	float y;
	float z;
}Angle_t;	// ���ٶ�


typedef struct
{
	short accel_x;		// x����ٶ�ֵ
	short accel_y;		// y����ٶ�ֵ
	short accel_z;		// z����ٶ�ֵ
}Accel_t;	// ���ٶ�

typedef struct
{
	short gyro_x;		// x����ٶ�ֵ
	short gyro_y;		// y����ٶ�ֵ
	short gyro_z;		// z����ٶ�ֵ
}Gyro_t;	// ���ٶ�

typedef struct
{
	short compass_x;	// x�������ֵ
	short compass_y;	// y�������ֵ
	short compass_z;	// z�������ֵ
}Compass_t;	// ����

typedef struct
{
	long Quanternion[4];	//��Ԫ��
}Quanternion_t;	// ��Ԫ��

typedef struct
{
	uint8_t SamplingFlag;
	uint8_t RetryCount;
}MPU9250_State_t;


typedef struct
{
	Euler_t Euler;
	Accel_t Accel;
	Gyro_t	Gyro;
	Compass_t	Compass;
	Angle_t GyroAngleVelocity;
	Quanternion_t Quant;
	MPU9250_State_t MPU9250_State;		// ����ʱ���־λ����ʱ���������־λ��1
}MPU9250_Para_t;	// MPU9250����״ֵ̬
#define MPU9250_SIZE	sizeof(MPU9250_Para_t)
extern MPU9250_Para_t MPU9250_Para;

/*typedef struct
{

}BMP280_Para_t;*/



// ��������
void MPU_INT_GPIO_Config(void);
uint8_t IMU_MPU9250_Init(void);
uint8_t Get_IMU_SensorData(void);



#endif

