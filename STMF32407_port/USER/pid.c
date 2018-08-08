#include "pid.h"
#include "string.h"
#include "mpu9250.h"
#include "public.h"
Cascade_PID_T Pid_Pitch, Pid_Roll, Pid_Yaw, Pid_Hight;
PID_T Pid_Altitude;
//个人感觉用增量式PID会更好计算

uint16_t PidOutMax, ErrMax;

/*static void PID_Parameter_Init(Cascade_PID_T *pid)
{
	pid->Outer_Kp = okp;
	pid->Outer_Ki = oki;
	pid->Outer_Kd = okd;
	pid->Inner_Kp = ikp;
	pid->Inner_Ki = iki;
	pid->Inner_Kd = ikd;
}*/

void Pid_Init(void)
{
	memset(&Pid_Pitch, 0, CASCADEPIDSIZE);
	memset(&Pid_Roll, 0, CASCADEPIDSIZE);
	memset(&Pid_Yaw, 0, CASCADEPIDSIZE);
	
	#ifndef THROTTLE
	Pid_Altitude.Outer_Kp = 2.5f;
	Pid_Altitude.Outer_Ki = 0.01f;
	Pid_Altitude.Outer_Kd = 5;
	#endif
	//PID_Parameter_Init(&Pid_Pitch);
	Pid_Pitch.Outer_Kp = 1.7f;
	Pid_Pitch.Outer_Ki = 0.01f;
	Pid_Pitch.Outer_Kd = 0;
	Pid_Pitch.Inner_Kp = 2;
	Pid_Pitch.Inner_Ki = 0.001f;
	Pid_Pitch.Inner_Kd = 5;
	Pid_Roll.Outer_Kp = 3.5f;
	Pid_Roll.Outer_Ki = 0.01f;
	Pid_Roll.Outer_Kd = 5;
	Pid_Roll.Inner_Kp = 1;
	Pid_Roll.Inner_Ki = 0.001f;
	Pid_Roll.Inner_Kd = 10;
	Pid_Yaw.Outer_Kp = 5;
	Pid_Yaw.Outer_Ki = 0.001f;
	Pid_Yaw.Inner_Kp = 2;
//	PID_Parameter_Init(&Pid_Roll);
//	PID_Parameter_Init(&Pid_Yaw);
}

//SV期望 PV实际
void Pid_Calculate(uint16_t Pv)
{
	float Pout, Iout, Dout;
	
	Pid_Altitude.Outer_Err = Pid_Altitude.Sv - Pv;
	
	Pid_Altitude.Outer_Sek += Pid_Altitude.Outer_Err;
	
	Pout = Pid_Altitude.Outer_Kp * Pid_Altitude.Outer_Err;
	Iout = Pid_Altitude.Outer_Ki * Pid_Altitude.Outer_Sek;
	Dout = Pid_Altitude.Outer_Kd * (Pid_Altitude.Outer_Err - Pid_Altitude.Outer_Err1);
	
	Pid_Altitude.Outer_Err1 = Pid_Altitude.Outer_Err;
	
	Pid_Altitude.Out = Pout + Iout + Dout;
	
	if (Pid_Altitude.Out < -1000)		Pid_Altitude.Out = -1000;	// 输出限幅
	else if (Pid_Altitude.Out > 500)	Pid_Altitude.Out =  500 ;	// 输出限幅
}


uint8_t Double_PID_Calculate(Cascade_PID_T * pid, float OPv, float IPv)
{
	float OPout, OIout, ODout, IPout, IIout, IDout;
	
//	OPout = OIout = ODout = IPout = IIout = IDout = 0;
	/*-*-*-*-*-*-*-*-*-*-*-* 外环计算-角度环 *-*-*-*-*-*-*-*-*-*-*-*/
	pid->Outer_Err = pid->Sv - OPv;											// 计算当前角度误差
	if (pid->Outer_Err < 20 && pid->Outer_Err > -20 && Sensor_Para.Ultra_Hight > 160)
		pid->Outer_Sek += pid->Outer_Err;									// 累加角度偏差
	else
		pid->Outer_Sek = 0;
	
	if (pid->Outer_Sek < -ErrMax)		pid->Outer_Sek = -ErrMax;			// 积分限幅
	else if (pid->Outer_Sek > ErrMax)	pid->Outer_Sek = ErrMax;			// 积分限幅

	OPout = pid->Outer_Kp * pid->Outer_Err;									// 计算角度环P部分
	OIout = pid->Outer_Ki * pid->Outer_Sek;									// 计算角度环I部分
	ODout = pid->Outer_Kd * (pid->Outer_Err - pid->Outer_Err1);				// 计算角度环D部分
	pid->Outer_Err1 = pid->Outer_Err;
	
	pid->Outer_Out = (OPout + OIout + ODout);								// 计算角度环PI输出(期望角速度)
	
	/*-*-*-*-*-*-*-*-*-*-*-* 内环计算-角速度环 *-*-*-*-*-*-*-*-*-*-*-*/
	pid->Inner_Err = pid->Outer_Out - IPv;									// 计算角速度环误差
	if (pid->Inner_Err < 50 && pid->Inner_Err > -50 && Sensor_Para.Ultra_Hight > 160)
		pid->Inner_Sek += pid->Inner_Err;									// 累加角速度偏差
	else
		pid->Inner_Sek = 0;
	if (pid->Inner_Sek < -ErrMax)		pid->Inner_Sek = -ErrMax;			// 积分限幅
	else if (pid->Inner_Sek > ErrMax)	pid->Inner_Sek = ErrMax;			// 积分限幅

	IPout = pid->Inner_Kp * pid->Inner_Err;									// 计算角速度环P部分
	IIout = pid->Inner_Ki * pid->Inner_Sek;									// 计算角速度环I部分
	IDout = pid->Inner_Kd * (pid->Inner_Err - pid->Inner_Err1);				// 计算角速度环D部分
	pid->Inner_Err1 = pid->Inner_Err;										// 记录当前角速度环误差

	pid->Out = IPout + IIout + IDout;										// 计算PID输出值

	if (pid->Out < -PidOutMax)		pid->Out = -PidOutMax;	// 输出限幅
	else if (pid->Out > PidOutMax)	pid->Out =  PidOutMax;	// 输出限幅

	return 0;
}


