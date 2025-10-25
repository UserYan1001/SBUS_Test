

#include "y_spwm.h"
#include <stdio.h>

#include "robot.h"

#define  PWM_DEAD_ZONE   10  //PWM��λ����

static uint8_t   ch1_status = 0;           //���벶���־
static uint16_t  ch1_tim_up;               //����������ʱ��
static uint16_t  ch1_tim_dowm;             //�����½���ʱ��
static int32_t   ch1_tim_value = 1500;     //ͨ��ʱ����ֵ
static int16_t   ch1_value;                //ͨ����ֵ

static uint8_t   ch2_status = 0;           //���벶���־
static uint16_t  ch2_tim_up;               //����������ʱ��
static uint16_t  ch2_tim_dowm;             //�����½���ʱ��
static int32_t   ch2_tim_value = 1500;     //ͨ����ֵ
static int16_t   ch2_value; 

static uint8_t   ch3_status = 0;           //���벶���־
static uint16_t  ch3_tim_up;               //����������ʱ��
static uint16_t  ch3_tim_dowm;             //�����½���ʱ��
int32_t   ch3_tim_value = 1500;     //ͨ����ֵ

static uint8_t   ch4_status = 0;           //���벶���־
static uint16_t  ch4_tim_up;               //����������ʱ��
static uint16_t  ch4_tim_dowm;             //�����½���ʱ��
static int32_t   ch4_tim_value = 1500;     //ͨ����ֵ

static uint8_t   spwm_ctl_flag = 0;        //ң�������Ʊ�־λ


/**
  * @��  ��  PWM�����ʼ��
  * @��  ��  ��
  * @����ֵ	 ��
  */
void SPWM_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	
	//ʱ��ʹ��    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	
	
	//���ù�������
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
	
	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;       //����
    GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	
    //��ʼ����ʱ��  TIM8_Cap_Init(9999,168-1);  //�߼���ʱ��TIM8��ʱ��Ƶ��Ϊ168M    
    TIM_DeInit(TIM8);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 168-1;  //168��Ƶ��Ƶ��Ϊ1M  
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;    //
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	
	//��ʼ�����벶��ͨ��
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x06;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure); 
 
 	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x06;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure); 
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x06;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure); 
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	
	TIM_ICInitStructure.TIM_ICFilter = 0x06;	  
	TIM_ICInit(TIM8, &TIM_ICInitStructure); 
 
    //����NVIC 
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	 
 
 
    //ʹ�ܲ����ж�CC1IE,CC2IE,CC3IE,CC4IE
	TIM_ITConfig(TIM8, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE); 
	
	//�߼���ʱ���������ʹ�����	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 	
	
	//ʹ�ܶ�ʱ��
	TIM_Cmd(TIM8, ENABLE);    	
}

/**
  * @��  ��  ��ʱ�����벶���ж�
  * @��  ��  �� 
  * @����ֵ  ��
  */
void TIM8_CC_IRQHandler(void)
{
	//ͨ��1���������¼�
	if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) 
	{
		//����жϱ�־λ
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1); 
		
		//����һ��������
		if (ch1_status == 0x00)
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch1_tim_up = TIM_GetCapture1(TIM8); 
	
			//����Ѳ���������
			ch1_status = 0x01;
			
			//����Ϊ�½��ز���
			TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Falling); 				
		}
		
		//�����½���
		else 
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch1_tim_dowm = TIM_GetCapture1(TIM8); 
			
			
			//�������������
			if (ch1_tim_dowm < ch1_tim_up)
			{
				//����ߵ�ƽ��ʱ��
				ch1_tim_value = ch1_tim_dowm - ch1_tim_up + 0xFFFF;
			}
			else
			{
				//����ߵ�ƽ��ʱ��
				ch1_tim_value = ch1_tim_dowm - ch1_tim_up;	
			}
					
			//������ϣ�ִ�ж���		
			//printf("@ %d  \r\n",ch1_tim_value );
			
			//���Ʊ�־��Ч
			if(spwm_ctl_flag != 0)
			{
				//W�����˶�����	
				if(ch1_tim_value > (1500 + PWM_DEAD_ZONE))
					ch1_value = (int16_t)(ch1_tim_value - (1500 + PWM_DEAD_ZONE));
				else if(ch1_tim_value < (1500 - PWM_DEAD_ZONE))
					ch1_value = (int16_t)(ch1_tim_value - (1500 - PWM_DEAD_ZONE));	
				else
					ch1_value = 0;
				
				//����ǰ�����������
				#if (ROBOT_TYPE == ROBOT_AKM)
					ax_akm_angle = (int16_t)( -3.2 * ch1_value);
				#else
					Vel.TG_IW = (int16_t)( -10.0 * ch1_value);
				#endif
			}
			
			//�����־����
			ch1_status = 0; 
			
			//����Ϊ�����ز���
			TIM_OC1PolarityConfig(TIM8, TIM_ICPolarity_Rising); 
		}
	}
	
	//ͨ��2���������¼�
	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) != RESET) 
	{
		//����жϱ�־λ
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2); 
		
		//����һ��������
		if (ch2_status == 0x00)
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch2_tim_up = TIM_GetCapture2(TIM8); 
	
			//����Ѳ���������
			ch2_status = 0x01;
			
			//����Ϊ�½��ز���
			TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Falling); 				
		}
		
		//�����½���
		else 
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch2_tim_dowm = TIM_GetCapture2(TIM8); 
			
			
			//�������������
			if (ch2_tim_dowm < ch2_tim_up)
			{
				//����ߵ�ƽ��ʱ��
				ch2_tim_value = ch2_tim_dowm - ch2_tim_up + 0xFFFF;
			}
			else
			{
				//����ߵ�ƽ��ʱ��
				ch2_tim_value = ch2_tim_dowm - ch2_tim_up;	
			}
					
			//������ϣ�ִ�ж���		
			printf("@ %d  \r\n",ch2_tim_value );
			
			//���Ʊ�־��Ч
			if(spwm_ctl_flag != 0)
			{
				//X�����˶�����
				if(ch2_tim_value > (1500 + PWM_DEAD_ZONE))
					ch2_value = (int16_t)(ch2_tim_value - (1500 + PWM_DEAD_ZONE));
				else if(ch2_tim_value < (1500 - PWM_DEAD_ZONE))
					ch2_value = (int16_t)(ch2_tim_value - (1500 - PWM_DEAD_ZONE));	
				else
					ch2_value = 0;
				
				Vel.TG_IX = 1.2 * ch2_value; 
			}
			
			//�����־����
			ch2_status = 0; 
			
			//����Ϊ�����ز���
			TIM_OC2PolarityConfig(TIM8, TIM_ICPolarity_Rising); 
		}
	}
	
	//ͨ��3���������¼�
	if (TIM_GetITStatus(TIM8, TIM_IT_CC3) != RESET) 
	{
		//����жϱ�־λ
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC3); 
		
		//����һ��������
		if (ch3_status == 0x00)
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch3_tim_up = TIM_GetCapture3(TIM8); 
	
			//����Ѳ���������
			ch3_status = 0x01;
			
			//����Ϊ�½��ز���
			TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Falling); 				
		}
		
		//�����½���
		else 
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch3_tim_dowm = TIM_GetCapture3(TIM8); 
			
			
			//�������������
			if (ch3_tim_dowm < ch3_tim_up)
			{
				//����ߵ�ƽ��ʱ��
				ch3_tim_value = ch3_tim_dowm - ch3_tim_up + 0xFFFF;
			}
			else
			{
				//����ߵ�ƽ��ʱ��
				ch3_tim_value = ch3_tim_dowm - ch3_tim_up;	
			}
					
			//������ϣ�ִ�ж���		
			//printf("@ %d  \r\n",ch3_tim_value );
			
			//�����־����
			ch3_status = 0; 
			
			//����Ϊ�����ز���
			TIM_OC3PolarityConfig(TIM8, TIM_ICPolarity_Rising); 
		}
	}
	
	//ͨ��4���������¼�
	if (TIM_GetITStatus(TIM8, TIM_IT_CC4) != RESET) 
	{
		//����жϱ�־λ
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC4); 
		
		//����һ��������
		if (ch4_status == 0x00)
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch4_tim_up = TIM_GetCapture4(TIM8); 
	
			//����Ѳ���������
			ch4_status = 0x01;
			
			//����Ϊ�½��ز���
			TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Falling); 				
		}
		
		//�����½���
		else 
		{
			//��¼��ʱ�Ķ�ʱ������ֵ
			ch4_tim_dowm = TIM_GetCapture4(TIM8); 
			
			
			//�������������
			if (ch4_tim_dowm < ch4_tim_up)
			{
				//����ߵ�ƽ��ʱ��
				ch4_tim_value = ch4_tim_dowm - ch4_tim_up + 0xFFFF;
			}
			else
			{
				//����ߵ�ƽ��ʱ��
				ch4_tim_value = ch4_tim_dowm - ch4_tim_up;	
			}
					
			//������ϣ�ִ�ж���	
			//printf("@ %d  \r\n",ch4_tim_value );
			
			//ͨ��4��ߵ����������			
			if(ch4_tim_value > 1800)
			{
				spwm_ctl_flag = 1;
			}
			else
			{
				spwm_ctl_flag = 0;
			}
			
			//�����־����
			ch4_status = 0; 
			
			//����Ϊ�����ز���
			TIM_OC4PolarityConfig(TIM8, TIM_ICPolarity_Rising); 
		}
	}
	
	


}

/**************************************************************************
Function: TIM1 Update Interrupt
Input   : none
Output  : none
�������ܣ���ʱ��8�����ж�
��ڲ�������
����  ֵ���� 
**************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void) 
{ 
	//Clear the interrupt flag bit
	//����жϱ�־λ 
  TIM8->SR&=~(1<<0);	    
}

/******************* (C) ��Ȩ 2022 XTARK **************************************/













