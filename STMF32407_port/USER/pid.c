#include "pid.h"
#include "string.h"
#include "mpu9250.h"
#include "public.h"
Cascade_PID_T Pid_Pitch, Pid_Roll, Pid_Yaw, Pid_Hight;
PID_T Pid_Altitude;
//���˸о�������ʽPID����ü���

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

//SV���� PVʵ��
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
	
	if (Pid_Altitude.Out < -1000)		Pid_Altitude.Out = -1000;	// ����޷�
	else if (Pid_Altitude.Out > 500)	Pid_Altitude.Out =  500 ;	// ����޷�
}


uint8_t Double_PID_Calculate(Cascade_PID_T * pid, float OPv, float IPv)
{
	float OPout, OIout, ODout, IPout, IIout, IDout;
	
//	OPout = OIout = ODout = IPout = IIout = IDout = 0;
	/*-*-*-*-*-*-*-*-*-*-*-* �⻷����-�ǶȻ� *-*-*-*-*-*-*-*-*-*-*-*/
	pid->Outer_Err = pid->Sv - OPv;											// ���㵱ǰ�Ƕ����
	if (pid->Outer_Err < 20 && pid->Outer_Err > -20 && Sensor_Para.Ultra_Hight > 160)
		pid->Outer_Sek += pid->Outer_Err;									// �ۼӽǶ�ƫ��
	else
		pid->Outer_Sek = 0;
	
	if (pid->Outer_Sek < -ErrMax)		pid->Outer_Sek = -ErrMax;			// �����޷�
	else if (pid->Outer_Sek > ErrMax)	pid->Outer_Sek = ErrMax;			// �����޷�

	OPout = pid->Outer_Kp * pid->Outer_Err;									// ����ǶȻ�P����
	OIout = pid->Outer_Ki * pid->Outer_Sek;									// ����ǶȻ�I����
	ODout = pid->Outer_Kd * (pid->Outer_Err - pid->Outer_Err1);				// ����ǶȻ�D����
	pid->Outer_Err1 = pid->Outer_Err;
	
	pid->Outer_Out = (OPout + OIout + ODout);								// ����ǶȻ�PI���(�������ٶ�)
	
	/*-*-*-*-*-*-*-*-*-*-*-* �ڻ�����-���ٶȻ� *-*-*-*-*-*-*-*-*-*-*-*/
	pid->Inner_Err = pid->Outer_Out - IPv;									// ������ٶȻ����
	if (pid->Inner_Err < 50 && pid->Inner_Err > -50 && Sensor_Para.Ultra_Hight > 160)
		pid->Inner_Sek += pid->Inner_Err;									// �ۼӽ��ٶ�ƫ��
	else
		pid->Inner_Sek = 0;
	if (pid->Inner_Sek < -ErrMax)		pid->Inner_Sek = -ErrMax;			// �����޷�
	else if (pid->Inner_Sek > ErrMax)	pid->Inner_Sek = ErrMax;			// �����޷�

	IPout = pid->Inner_Kp * pid->Inner_Err;									// ������ٶȻ�P����
	IIout = pid->Inner_Ki * pid->Inner_Sek;									// ������ٶȻ�I����
	IDout = pid->Inner_Kd * (pid->Inner_Err - pid->Inner_Err1);				// ������ٶȻ�D����
	pid->Inner_Err1 = pid->Inner_Err;										// ��¼��ǰ���ٶȻ����

	pid->Out = IPout + IIout + IDout;										// ����PID���ֵ

	if (pid->Out < -PidOutMax)		pid->Out = -PidOutMax;	// ����޷�
	else if (pid->Out > PidOutMax)	pid->Out =  PidOutMax;	// ����޷�

	return 0;
}


