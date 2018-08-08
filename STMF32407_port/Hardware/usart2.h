#ifndef __USART2_H
#define __USART2_H

#include "public.h"

#define SEND_RESTART	Usart2_Cmd[0]
#define SEND_MOTOR_PARA	Usart2_Cmd[1]

extern uint8_t Usart2_Cmd[];
//extern volatile uint8_t Usart2SendDelay;

void USART2_Task(void);
void Usart2_Tx_Task(void);
void Usart2_Rx_Task(void);
void Usart6_Tx_Task(void);
void Usart6_Rx_Task(void);


#endif

