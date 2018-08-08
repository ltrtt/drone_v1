#ifndef __PID_H_
#define __PID_H_

#include "stm32f4xx.h"

typedef struct
{
	float Sv;	// ����ֵ

	float Outer_Kp;		// �⻷����ϵ��
	float Outer_Ki;		// �⻷���ֳ���
	float Outer_Kd;		// �⻷΢�ֳ���
	float Outer_Sek;	// �⻷ƫ���ܺ�
	float Outer_Err;	// �⻷����ƫ��
	float Outer_Err1;	// �⻷�ϴ�ƫ��
	float Outer_Out;

	float Out;
}PID_T;
extern PID_T Pid_Altitude;

typedef struct
{
	float Sv;	// ����ֵ

	/*	�⻷-�ǶȻ�PI	*/
	float Outer_Kp;		// �⻷����ϵ��
	float Outer_Ki;		// �⻷���ֳ���
	float Outer_Kd;		// �⻷΢�ֳ���
	float Outer_Sek;	// �⻷ƫ���ܺ�
	float Outer_Err;	// �⻷����ƫ��
	float Outer_Err1;	// �⻷�ϴ�ƫ��
	float Outer_Out;


	/*	�ڻ�-���ٶȻ�PID	*/
	float Inner_Kp;		// �ڻ�����ϵ��
	float Inner_Ki;		// �ڻ����ֳ���
	float Inner_Kd;		// �ڻ�΢�ֳ���
	float Inner_Sek;	// �ڻ�ƫ���ܺ�
	float Inner_Err;	// �ڻ�����ƫ��
	float Inner_Err1;	// �ڻ��ϴ�ƫ��
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

