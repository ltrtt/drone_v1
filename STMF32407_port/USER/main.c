#include "public.h"
#include "mpu9250_driver.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpl.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "packet.h"
#include "bmp280_driver.h"
#include "pid.h"
#include "usart2.h"
#include "adc.h"
#include "nrf2401.h"


Sensor_Para_t Sensor_Para;
Motor_PWM_T Motor_PWM;
uint8_t NRF_RXBuff[10];
uint16_t Throttle;	//油门
uint16_t NoThrottleCount;
Attitude_Parameter_T Attitude_Parameter;


uint8_t a[] = {0x68, 0x01, 0x16};

//#define COMPASS_ENABLED 1
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";





uint8_t ResetPID, dir, send_flag, test_flag, test_result, mgaddr, wifi_pwr;
float KP, TI, TD;
float okp, oki, okd, ikp, iki, ikd;
uint16_t errorCount;
uint16_t youmen, Hight;
int main(void)
{	
//	short aacx,aacy,aacz;	        
//	short gyrox,gyroy,gyroz;        
//	short temp;		               
//	float pressure, temperature, elevation;
	System_Initialization();
	Usart_Send_Data((USARTx_Parameter_T *)&Print_Usart, a, 3);
	PidOutMax = 200;
	ErrMax = 25000;
	okp = 5;
	oki = 0;
	okd = 0;
	ikp = 2;
	iki = 0;
	ikd = 0;
	dir = 0;
	send_flag = 1;
	Pid_Init();
	//Adc_Init();
	NRF2401_Init();

	if ((test_result = IMU_MPU9250_Init()) > 0)
	{
		// 通知F1重启飞控电路, 重启2次还无法正常读取MPU状态，则状态灯快速闪动
		errorCount++;
		while (1);
	}
#ifdef NRF
	if (NRF24L01_Check())
	{
		while (1);
	}
	NRF24L01_RX_Mode();
#endif
	mpu_set_bypass(1);		//设置为旁路模式
	bmp280Init();
	
	
	while (1) 
	{
		
		if (test_flag)	{
			mpu_set_bypass(1);		//设置为旁路模式
			Delay_ms(50);
			mgaddr = IIC_ReadByte(0x18, 0x00);
			mpu_set_bypass(0);
			test_result = run_self_test();
			test_flag = 0;
		}
		#ifdef NRF
		if (TimeTick % 2 == 0){
			if (NRF24L01_RxPacket(NRF_RXBuff) == 0) {
				if (NRF_RXBuff[0] == 0x12) {
					#ifdef THROTTLE
					Throttle = (NRF_RXBuff[3] << 8) | NRF_RXBuff[4];
					#else
					Pid_Altitude.Sv = (NRF_RXBuff[3] << 8) | NRF_RXBuff[4];
					#endif
					NoThrottleCount = 0;
				}
			}
		}
		
		#endif
		if (TimeTick % 50 == 0)
			Sensor_Para.Ultrasonic_SamplingFlag = 1;
		if (TimeTick % SAMPLING_RATE == 0)	{
			Get_IMU_SensorData();
			
			if (MPU9250_Para.Euler.pitch > 50 || MPU9250_Para.Euler.pitch < -50)
				errorCount++;
		}
		if (ResetPID)
		{
			ResetPID = 0;
			//Pid_Init();
			Pid_Pitch.Inner_Sek = Pid_Pitch.Outer_Sek	= Pid_Pitch.Outer_Out	= Pid_Pitch.Out =
			Pid_Roll.Outer_Out	= Pid_Roll.Inner_Sek	= Pid_Roll.Outer_Sek	= Pid_Roll.Out =
			Pid_Yaw.Inner_Sek	= Pid_Yaw.Outer_Sek		= Pid_Yaw.Outer_Out		= Pid_Yaw.Out = 0;
		}
		if (TimeTick % SAMPLING_RATE == 0)
		{
			if (Pid_Altitude.Sv > 150 || Throttle > 10)
			{
				Double_PID_Calculate(&Pid_Pitch, MPU9250_Para.Euler.pitch, MPU9250_Para.GyroAngleVelocity.y);
				Double_PID_Calculate(&Pid_Roll, MPU9250_Para.Euler.roll, MPU9250_Para.GyroAngleVelocity.x);
				Double_PID_Calculate(&Pid_Yaw, MPU9250_Para.Euler.yaw, MPU9250_Para.GyroAngleVelocity.z);
				Double_PID_Calculate(&Pid_Hight, (float)Sensor_Para.Ultra_Hight, Sensor_Para.Ascending_Velocity);
				Pid_Calculate(Sensor_Para.Ultra_Hight);
#ifdef AXIS1
//				Motor_PWM.Motor1_PWM = (Throttle/* + Pid_Pitch.Out*/ + Pid_Roll.Out) > 1000 ? 1000 : (Throttle/* + Pid_Pitch.Out*/ + Pid_Roll.Out);
//				Motor_PWM.Motor2_PWM = (Throttle/* - Pid_Pitch.Out*/ + Pid_Roll.Out) > 1000 ? 1000 : (Throttle/* - Pid_Pitch.Out*/ + Pid_Roll.Out);
//				Motor_PWM.Motor3_PWM = (Throttle/* - Pid_Pitch.Out*/ - Pid_Roll.Out) > 1000 ? 1000 : (Throttle/* - Pid_Pitch.Out*/ - Pid_Roll.Out);
//				Motor_PWM.Motor4_PWM = (Throttle/* + Pid_Pitch.Out*/ - Pid_Roll.Out) > 1000 ? 1000 : (Throttle/* + Pid_Pitch.Out*/ - Pid_Roll.Out);

				Motor_PWM.Motor1_PWM = (Throttle + Pid_Hight.Out) > 1000 ? 1000 : (Throttle + Pid_Hight.Out);
				Motor_PWM.Motor2_PWM = (Throttle + Pid_Hight.Out) > 1000 ? 1000 : (Throttle + Pid_Hight.Out);
				Motor_PWM.Motor3_PWM = (Throttle + Pid_Hight.Out) > 1000 ? 1000 : (Throttle + Pid_Hight.Out);
				Motor_PWM.Motor4_PWM = (Throttle + Pid_Hight.Out) > 1000 ? 1000 : (Throttle + Pid_Hight.Out);
#else
				Motor_PWM.Motor1_PWM = (Throttle + Pid_Pitch.Out + Pid_Roll.Out + Pid_Yaw.Out + Pid_Hight.Out) > 1000 ? 1000 : (Throttle + Pid_Pitch.Out + Pid_Roll.Out + Pid_Yaw.Out + Pid_Hight.Out);
				Motor_PWM.Motor2_PWM = (Throttle - Pid_Pitch.Out + Pid_Roll.Out - Pid_Yaw.Out + Pid_Hight.Out) > 1000 ? 1000 : (Throttle - Pid_Pitch.Out + Pid_Roll.Out - Pid_Yaw.Out + Pid_Hight.Out);
				Motor_PWM.Motor3_PWM = (Throttle - Pid_Pitch.Out - Pid_Roll.Out + Pid_Yaw.Out + Pid_Hight.Out) > 1000 ? 1000 : (Throttle - Pid_Pitch.Out - Pid_Roll.Out + Pid_Yaw.Out + Pid_Hight.Out);
				Motor_PWM.Motor4_PWM = (Throttle + Pid_Pitch.Out - Pid_Roll.Out - Pid_Yaw.Out + Pid_Hight.Out) > 1000 ? 1000 : (Throttle + Pid_Pitch.Out - Pid_Roll.Out - Pid_Yaw.Out + Pid_Hight.Out);
#endif
				
				if (MPU9250_Para.Euler.pitch > 60 || MPU9250_Para.Euler.pitch < -60)
					Motor_PWM.Motor1_PWM = Motor_PWM.Motor2_PWM = Motor_PWM.Motor3_PWM = Motor_PWM.Motor4_PWM = 0;
				
				SEND_MOTOR_PARA = 1;
			}
			else if (Pid_Altitude.Sv <= 150 || Throttle < 300)
			{
				Motor_PWM.Motor1_PWM = Throttle;
				Motor_PWM.Motor2_PWM = Throttle;
				Motor_PWM.Motor3_PWM = Throttle;			
				Motor_PWM.Motor4_PWM = Throttle;
				SEND_MOTOR_PARA = 1;
				ResetPID = 1;
			}
		}
		#ifdef NRF
		if (NoThrottleCount > 1000) {
			Motor_PWM.Motor1_PWM = 0;
			Motor_PWM.Motor2_PWM = 0;
			Motor_PWM.Motor3_PWM = 0;			
			Motor_PWM.Motor4_PWM = 0;
		}
		#endif
		if (TimeTick % 5 == 0) {
			
			/*ANO_DT_Send_Status(Sensor_Para.Ascending_Velocity, MPU9250_Para.Accel.accel_z, 
							   MPU9250_Para.Euler.yaw, Sensor_Para.Ultra_Hight, 0, 0);*/
			ANO_DT_Send_Senser(	MPU9250_Para.Accel.accel_x, MPU9250_Para.Accel.accel_y, MPU9250_Para.Accel.accel_z,
								MPU9250_Para.Gyro.gyro_x, MPU9250_Para.Gyro.gyro_y, MPU9250_Para.Gyro.gyro_z,
								MPU9250_Para.Compass.compass_x, MPU9250_Para.Compass.compass_y, MPU9250_Para.Compass.compass_z, 0);

		}
		if (TimeTick % 50 == 0) {
			ANO_DT_Send_MotoPWM(Motor_PWM.Motor1_PWM, Motor_PWM.Motor2_PWM, Motor_PWM.Motor3_PWM, Motor_PWM.Motor4_PWM,
								0, 0, 0, 0);
		}
		if (send_flag)
			USART2_Task();
		Usart6_Tx_Task();
		Usart6_Rx_Task();

	}
}



//u8 mpu_mpl_get_data(float *pitch, float *roll, float *yaw)
//{
//	unsigned long sensor_timestamp, timestamp;
//	short gyro[3], accel_short[3], compass_short[3], sensors;
//	unsigned char more;
//	long compass[3], accel[3], quat[4], temperature;
//	long data[9];
//	int8_t accuracy;

//	if (dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more))return 1;

//	if (sensors & INV_XYZ_GYRO)
//	{
//		inv_build_gyro(gyro, sensor_timestamp);          //°ÑÐÂÊý¾Ý·¢ËÍ¸øMPL
//		mpu_get_temperature(&temperature, &sensor_timestamp);
//		inv_build_temp(temperature, sensor_timestamp);   //°ÑÎÂ¶ÈÖµ·¢¸øMPL£¬Ö»ÓÐÍÓÂÝÒÇÐèÒªÎÂ¶ÈÖµ
//	}

//	if (sensors & INV_XYZ_ACCEL)
//	{
//		accel[0] = (long)accel_short[0];
//		accel[1] = (long)accel_short[1];
//		accel[2] = (long)accel_short[2];
//		inv_build_accel(accel, 0, sensor_timestamp);      //°Ñ¼ÓËÙ¶ÈÖµ·¢¸øMPL
//	}

//	if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
//	{
//		compass[0] = (long)compass_short[0];
//		compass[1] = (long)compass_short[1];
//		compass[2] = (long)compass_short[2];
//		inv_build_compass(compass, 0, sensor_timestamp); //°Ñ´ÅÁ¦¼ÆÖµ·¢¸øMPL
//	}
//	inv_execute_on_data();
//	inv_get_sensor_type_euler(data, &accuracy, &timestamp);

//	*roll = (data[0] / q16);
//	*pitch = -(data[1] / q16);
//	*yaw = -data[2] / q16;
//	return 0;
//}



//void read_from_mpl(void)
//{
//	long msg, data[9];
//	int8_t accuracy;
//	unsigned long timestamp;
//	float float_data[3] = { 0 };

//	if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
//		/* Sends a quaternion packet to the PC. Since this is used by the Python
//		* test app to visually represent a 3D quaternion, it's sent each time
//		* the MPL has new data.
//		*/
//		eMPL_send_quat(data);

//		/* Specific data packets can be sent or suppressed using USB commands. */
//		if (hal.report & PRINT_QUAT) {}
//		//	eMPL_send_data(PACKET_DATA_QUAT, data);
//	}

//	if (hal.report & PRINT_ACCEL) {
//		if (inv_get_sensor_type_accel(data, &accuracy,
//			(inv_time_t*)&timestamp)) {
//		}
//		//	eMPL_send_data(PACKET_DATA_ACCEL, data);
//	}
//	if (hal.report & PRINT_GYRO) {
//		if (inv_get_sensor_type_gyro(data, &accuracy,
//			(inv_time_t*)&timestamp)) {
//		}
//		//	eMPL_send_data(PACKET_DATA_GYRO, data);
//	}
//#ifdef COMPASS_ENABLED
//	if (hal.report & PRINT_COMPASS) {
//		if (inv_get_sensor_type_compass(data, &accuracy,
//			(inv_time_t*)&timestamp)) {
//		}
//		//	eMPL_send_data(PACKET_DATA_COMPASS, data);
//	}
//#endif
//	if (hal.report & PRINT_EULER) {
//		if (inv_get_sensor_type_euler(data, &accuracy,
//			(inv_time_t*)&timestamp)) {
//		}
//		//	eMPL_send_data(PACKET_DATA_EULER, data);
//	}
//	if (hal.report & PRINT_ROT_MAT) {
//		if (inv_get_sensor_type_rot_mat(data, &accuracy,
//			(inv_time_t*)&timestamp)){ }
//		//	eMPL_send_data(PACKET_DATA_ROT, data);
//	}
//	if (hal.report & PRINT_HEADING) {
//		if (inv_get_sensor_type_heading(data, &accuracy,
//			(inv_time_t*)&timestamp)) {
//		}
//		//	eMPL_send_data(PACKET_DATA_HEADING, data);
//	}
//	if (hal.report & PRINT_LINEAR_ACCEL) {
//		if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
//			MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
//				float_data[0], float_data[1], float_data[2]);
//		}
//	}
//	if (hal.report & PRINT_GRAVITY_VECTOR) {
//		if (inv_get_sensor_type_gravity(float_data, &accuracy,
//			(inv_time_t*)&timestamp))
//			MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
//				float_data[0], float_data[1], float_data[2]);
//	}
//	if (hal.report & PRINT_PEDO) {
//		unsigned long timestamp;
//		get_tick_count(&timestamp);
//		if (timestamp > hal.next_pedo_ms) {
//			hal.next_pedo_ms = timestamp + PEDO_READ_MS;
//			unsigned long step_count, walk_time;
//			dmp_get_pedometer_step_count(&step_count);
//			dmp_get_pedometer_walk_time(&walk_time);
//			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
//				walk_time);
//		}
//	}

//	/* Whenever the MPL detects a change in motion state, the application can
//	* be notified. For this example, we use an LED to represent the current
//	* motion state.
//	*/
//	msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
//		INV_MSG_NO_MOTION_EVENT);
//	if (msg) {
//		if (msg & INV_MSG_MOTION_EVENT) {
//			MPL_LOGI("Motion!\n");
//		}
//		else if (msg & INV_MSG_NO_MOTION_EVENT) {
//			MPL_LOGI("No motion!\n");
//		}
//	}
//}

