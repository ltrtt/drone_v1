#include "motorcontrol.h"

const GPIO_INIT_T PWM_GPIO_Init = 
{
	.RCC_APB2Periph_GPIOx = RCC_APB2Periph_GPIOB,
	.Portx = GPIOB,
	.Pinx = GPIO_Pin_6 |GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9,
	.GPIO_Speed = GPIO_Speed_50MHz,
	.GPIO_Mode = GPIO_Mode_AF_PP
};





void SpeedControl(void)
{
	if (Motor_PWM.Motor1_PWM <= 1000)	TIM_SetCompare1(TIM4, Motor_PWM.Motor1_PWM + MAX_SPEED);
	else								TIM_SetCompare1(TIM4, 1000 + MAX_SPEED);
	if (Motor_PWM.Motor2_PWM <= 1000)	TIM_SetCompare2(TIM4, Motor_PWM.Motor2_PWM + MAX_SPEED);
	else								TIM_SetCompare2(TIM4, 1000 + MAX_SPEED);
	if (Motor_PWM.Motor3_PWM <= 1000)	TIM_SetCompare3(TIM4, Motor_PWM.Motor3_PWM + MAX_SPEED);
	else								TIM_SetCompare3(TIM4, 1000 + MAX_SPEED);
	if (Motor_PWM.Motor4_PWM <= 1000)	TIM_SetCompare4(TIM4, Motor_PWM.Motor4_PWM + MAX_SPEED);
	else								TIM_SetCompare4(TIM4, 1000 + MAX_SPEED);
}

void Monoter_Control_Init(uint16_t Arr, uint16_t Psc)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);							//ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);		//ʹ�ܸ��ù���
//	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);							//Timer3���Ÿ���  TIM3_CH2->PB5    
	
	//����GPIO��ʼ��
	GPIOx_Init((GPIO_INIT_T *)&PWM_GPIO_Init);

	//����TIM
	TIM_TimeBaseStructure.TIM_Period = Arr;						//�Զ����ؼ�������ֵ	Ҫ��1
	TIM_TimeBaseStructure.TIM_Prescaler = Psc;					//��Ƶϵ��   ���ü�1
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//����PWM
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;				//ѡ��ʱ��ģʽ��TIM�����ȵ���ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;		//������ԣ�TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);						//ָ��ͨ��2���
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);						//ָ��ͨ��2���
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);						//ָ��ͨ��2���
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);						//ָ��ͨ��2���

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);				//ʹ��TIM4��ͨ��1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);				//ʹ��TIM4��ͨ��2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);				//ʹ��TIM4��ͨ��3�ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);				//ʹ��TIM4��ͨ��4�ϵ�Ԥװ�ؼĴ���

	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3

}

