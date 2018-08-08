#ifndef __PUBLIC
#define __PUBLIC

#include "stm32f4xx.h"
#include "stdio.h"	
#include "sys_function.h"
#include "Usart.h"
#include "sys_config.h"
#include "mpu9250.h"


#define SAMPLING_RATE	4






//#define AXIS1
//#define NRF
#define THROTTLE


#define PROGRAM_STOP	while(1)

#pragma pack(1)
typedef struct
{
	uint8_t Head;
	uint8_t Len;
	uint8_t Cmd;
	uint16_t Motor1_PWM;
	uint16_t Motor2_PWM;
	uint16_t Motor3_PWM;
	uint16_t Motor4_PWM;
	uint16_t CRC_16;
}Motor_PWM_T;
extern Motor_PWM_T Motor_PWM;
#define MOTORPWMSIZE sizeof(Motor_PWM_T)

typedef struct
{
	MPU9250_Para_t *MPU_9520;
	uint16_t Ultra_Hight;
	float Ascending_Velocity;
	uint8_t Ultrasonic_Work;
	uint8_t Ultrasonic_SamplingFlag;
}Sensor_Para_t;
#define SENSOR_SIZE	sizeof(Sensor_Para_t)
extern Sensor_Para_t Sensor_Para;


















extern volatile uint32_t time;
//extern volatile uint32_t g_ul_ms_ticks;
extern USARTx_Parameter_T Print_Usart, Communication_Usart;
extern uint16_t NoThrottleCount;


static void read_from_mpl(void);
void GPIO_Config(void);
void MPU_Init(void);
u8 mpu_mpl_get_data(float *pitch, float *roll, float *yaw);

#endif

