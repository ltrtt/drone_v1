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
	float pitch;		// 俯仰角
	float roll;			// 横滚角
	float yaw;			// 偏航角
}Euler_t;	// 欧拉角

typedef struct
{
	float x;
	float y;
	float z;
}Angle_t;	// 角速度


typedef struct
{
	short accel_x;		// x轴加速度值
	short accel_y;		// y轴加速度值
	short accel_z;		// z轴加速度值
}Accel_t;	// 加速度

typedef struct
{
	short gyro_x;		// x轴角速度值
	short gyro_y;		// y轴角速度值
	short gyro_z;		// z轴角速度值
}Gyro_t;	// 角速度

typedef struct
{
	short compass_x;	// x轴磁力计值
	short compass_y;	// y轴磁力计值
	short compass_z;	// z轴磁力计值
}Compass_t;	// 罗盘

typedef struct
{
	long Quanternion[4];	//四元数
}Quanternion_t;	// 四元数

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
	MPU9250_State_t MPU9250_State;		// 采样时间标志位，到时间了这个标志位置1
}MPU9250_Para_t;	// MPU9250所有状态值
#define MPU9250_SIZE	sizeof(MPU9250_Para_t)
extern MPU9250_Para_t MPU9250_Para;

/*typedef struct
{

}BMP280_Para_t;*/



// 函数声明
void MPU_INT_GPIO_Config(void);
uint8_t IMU_MPU9250_Init(void);
uint8_t Get_IMU_SensorData(void);



#endif

