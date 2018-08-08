#ifndef __PID_H_
#define __PID_H_

#include "stm32f4xx.h"

typedef struct
{
	float Sv;	// 期望值

	float Outer_Kp;		// 外环比例系数
	float Outer_Ki;		// 外环积分常数
	float Outer_Kd;		// 外环微分常数
	float Outer_Sek;	// 外环偏差总和
	float Outer_Err;	// 外环本次偏差
	float Outer_Err1;	// 外环上次偏差
	float Outer_Out;

	float Out;
}PID_T;
extern PID_T Pid_Altitude;

typedef struct
{
	float Sv;	// 期望值

	/*	外环-角度环PI	*/
	float Outer_Kp;		// 外环比例系数
	float Outer_Ki;		// 外环积分常数
	float Outer_Kd;		// 外环微分常数
	float Outer_Sek;	// 外环偏差总和
	float Outer_Err;	// 外环本次偏差
	float Outer_Err1;	// 外环上次偏差
	float Outer_Out;


	/*	内环-角速度环PID	*/
	float Inner_Kp;		// 内环比例系数
	float Inner_Ki;		// 内环积分常数
	float Inner_Kd;		// 内环微分常数
	float Inner_Sek;	// 内环偏差总和
	float Inner_Err;	// 内环本次偏差
	float Inner_Err1;	// 内环上次偏差
	float Out;
}Cascade_PID_T;
#define CASCADEPIDSIZE sizeof(Cascade_PID_T)
extern Cascade_PID_T Pid_Pitch, Pid_Roll, Pid_Yaw, Pid_Hight;

extern float KP, TI, TD, okp, oki, okd, ikp, iki, ikd;
extern uint16_t PidOutMax, ErrMax;


void Pid_Init(void);
uint8_t Double_PID_Calculate(Cascade_PID_T * pid, float OPv, float IPv);
void Pid_Calculate(uint16_t Pv);

#endif

