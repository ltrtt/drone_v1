#include "mpu9250.h"						// 本文件配套的头文件
#include "Public.h"							// 公共文件集合
#include "inv_mpu.h"						// 调用MPU相关函数需要用到
#include "mpl.h"							// MPL库需要用到
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"					// 欧拉角输出需要用到
#include "inv_mpu_dmp_motion_driver.h"		// DMP组件需要用到


#include "nrf2401.h"
MPU9250_Para_t MPU9250_Para;

#define q16  65536.0f

const GPIO_INIT_T MPU_GPIO_INIT = {
	.RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA,
	.Portx = GPIOA,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_OType = GPIO_OType_OD,//GPIO_OType_PP,
	.GPIO_Mode = GPIO_Mode_OUT,
	.GPIO_PuPd = GPIO_PuPd_UP,
	.Pinx = GPIO_Pin_1 | GPIO_Pin_2
};

struct platform_data_s {
	signed char orientation[9];
};
// 可以修改为const数组

const static struct platform_data_s gyro_pdata = {
	.orientation = { 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1 }
};
const static struct platform_data_s compass_pdata = {
	.orientation = { 0, 1, 0,
					 1, 0, 0,
					 0, 0, -1 }
};/*
const static struct platform_data_s gyro_pdata = {
	.orientation = { 1, 0, 0,
					 0,-1, 0,
					 0, 0,-1 }
};
const static struct platform_data_s compass_pdata = {
	.orientation = { 0,-1, 0,
					 1, 0, 0,
					 0, 0, 1 }
};*/

unsigned long sensor_timestamp, timestamp;
short sensors;
uint8_t more;
long temperature_9250;
long euler_data[9], Gyro_Anglevelocity[9], Compass_Angle[9];
int8_t accuracy;
uint32_t ttttime;
float dongnanxibei[3];
uint8_t Get_IMU_SensorData(void)
{
	long accel[3], compass[3];
	if (dmp_read_fifo(	(short *)&MPU9250_Para.Gyro.gyro_x, (short *)&MPU9250_Para.Accel.accel_x, MPU9250_Para.Quant.Quanternion,
						&sensor_timestamp, &sensors, &more))
		return 1;
	
	if (sensors & INV_XYZ_GYRO)
	{
		inv_build_gyro((short *)&MPU9250_Para.Gyro.gyro_x, sensor_timestamp);
		// 温度不用一直读
		mpu_get_temperature(&temperature_9250, &sensor_timestamp);
		inv_build_temp(temperature_9250, sensor_timestamp);
	}
	if (sensors & INV_XYZ_ACCEL)
	{
		accel[0] = (long)MPU9250_Para.Accel.accel_x;
		accel[1] = (long)MPU9250_Para.Accel.accel_y;
		accel[2] = (long)MPU9250_Para.Accel.accel_z;
		inv_build_accel(accel, 0, sensor_timestamp);     
	}
	if (!mpu_get_compass_reg((short *)&MPU9250_Para.Compass.compass_x, &sensor_timestamp))
	{
		compass[0] = (long)MPU9250_Para.Compass.compass_x;
		compass[1] = (long)MPU9250_Para.Compass.compass_y;
		compass[2] = (long)MPU9250_Para.Compass.compass_z;
		inv_build_compass(compass, 0, sensor_timestamp);
	}
	inv_execute_on_data();
	inv_get_sensor_type_euler(euler_data, &accuracy, &timestamp);
	inv_get_sensor_type_gyro(Gyro_Anglevelocity, &accuracy, &timestamp);
	inv_get_sensor_type_compass(Compass_Angle, &accuracy, &timestamp);
	
	MPU9250_Para.Euler.roll = (euler_data[0] / q16);
	MPU9250_Para.Euler.pitch = -(euler_data[1] / q16);
	MPU9250_Para.Euler.yaw = -euler_data[2] / q16;
	
	MPU9250_Para.GyroAngleVelocity.x = (Gyro_Anglevelocity[0] / q16);
	MPU9250_Para.GyroAngleVelocity.y = -(Gyro_Anglevelocity[1] / q16);
	MPU9250_Para.GyroAngleVelocity.z = -(Gyro_Anglevelocity[2] / q16);
	
	dongnanxibei[0] = Compass_Angle[0] / q16;
	dongnanxibei[1] = Compass_Angle[1] / q16;
	dongnanxibei[2] = Compass_Angle[2] / q16;
	return 0;
}


uint8_t IMU_MPU9250_Init(void)
{
	struct int_param_s int_param;
	uint8_t Init_step = 0, accel_fsr;
	uint16_t gyro_rate, gyro_fsr, compass_fsr;

	Sensor_Para.MPU_9520 = &MPU9250_Para;

	while (1)
	{
		switch (Init_step)
		{
		case 0:
			GPIOx_Init((GPIO_INIT_T *)&MPU_GPIO_INIT);
			MPU_INT_GPIO_Config();
			

			if (mpu_init(&int_param))	// 初始化MPU9250
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 1;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 1:
			if (inv_init_mpl())	// 初始化MPL库
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 2;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 2:
			inv_enable_quaternion();
			inv_enable_9x_sensor_fusion();
			inv_enable_fast_nomot();
			inv_enable_gyro_tc();
			inv_enable_vector_compass_cal();
			inv_enable_magnetic_disturbance();
			inv_enable_eMPL_outputs();
			mpu_set_bypass(1);		//设置为旁路模式
			Init_step++;
			break;
		case 3:
			if (inv_start_mpl())	// 开启MPL
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 3;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 4:
			if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 4;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 5:
			if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 5;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 6:
			if (mpu_set_sample_rate(DEFAULT_MPU_HZ))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 6;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 7:
			if (mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 7;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 8:
			mpu_get_sample_rate(&gyro_rate);
			mpu_get_gyro_fsr(&gyro_fsr);
			mpu_get_accel_fsr(&accel_fsr);
			mpu_get_compass_fsr(&compass_fsr);

			inv_set_gyro_sample_rate(1000000L / gyro_rate);
			inv_set_accel_sample_rate(1000000L / gyro_rate);
			inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
		
			inv_set_gyro_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)gyro_fsr << 15);
			inv_set_accel_orientation_and_scale(inv_orientation_matrix_to_scalar(gyro_pdata.orientation), (long)accel_fsr << 15);
			inv_set_compass_orientation_and_scale(inv_orientation_matrix_to_scalar(compass_pdata.orientation), (long)compass_fsr << 15);
			Init_step++;
			break;
		case 9:
			if (dmp_load_motion_driver_firmware())	// 加载DMP固件
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 8;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 10:
			// 设置陀螺仪方向
			if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation)))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 9;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 11:
			// 设置DMP功能
			if (dmp_enable_feature(	DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
									DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
									DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 10;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 12:
			if (dmp_set_fifo_rate(DEFAULT_MPU_HZ))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 11;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 13:
			if (mpu_set_dmp_state(1))
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 12;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				Init_step++;
			break;
		case 14:
			// 使能DMP
			
			if (run_self_test())
			{
				MPU9250_Para.MPU9250_State.RetryCount++;
				if (MPU9250_Para.MPU9250_State.RetryCount == MPU9250_Init_RetryTime)
					return 13;
				else
				{
					Init_step = 0;
					break;
				}
			}
			else
				return 0;
			// 至此，MPU9250初始化完毕.
		//	break;
		}
	}
	
}


void MPU_INT_GPIO_Config(void)
{

	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	//  GPIO_DeInit(GPIOA);
	//  GPIO_DeInit(GPIOC);

	/* Enable GPIOB clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure invensense sensor interrupt pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOE, &GPIO_InitStructure); //GPIOA

										   /* Connect EXTI Line to inv sensor interrupt pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);

	/* Configure EXTI Line1 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line Interrupt to the highest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	MPU_SCL = 1;	// 拉高时钟线
	MPU_SDA_O = 1;	// 拉高数据线
}


void EXTI2_IRQHandler(void)
{
	/* Handle new gyro*/
//	hal.new_gyro = 1;
	MPU9250_Para.MPU9250_State.SamplingFlag = 1;
	EXTI_ClearITPendingBit(EXTI_Line2);
}

uint8_t run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_6500_self_test(gyro, accel,0);
	if (result == 0x7) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
        unsigned short accel_sens;
		float gyro_sens;

		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
        //inv_set_gyro_bias(gyro, 3);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
       // inv_set_accel_bias(accel, 3);
		dmp_set_accel_bias(accel);
		return 0;
	}else return result;
}