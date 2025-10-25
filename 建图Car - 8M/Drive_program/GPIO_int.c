
/****************************************************************************
 *	@����	��	YJH
 *	@����	��	2025-09-09
 *	@����	��	�������¿Ƽ����޹�˾
 *****************************************************************************/
#include "main.h"


// ������ʼ��
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // ����GPIOʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOC, ENABLE);
    
    // ����KEY1 (PE10) ����ģʽ
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // �������룬����δ����ʱΪ�ߵ�ƽ
    GPIO_Init(KEY1_PORT, &GPIO_InitStructure);
    
    // ����KEY2 (PC15) ����ģʽ
    GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  // �������룬����δ����ʱΪ�ߵ�ƽ
    GPIO_Init(KEY2_PORT, &GPIO_InitStructure);
}

// ����ɨ�躯������������
uint8_t KEY_Scan(void)
{
   
    if((	GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0 || 
					GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == 0))
    {
        // ��ʱ����
        for(int i = 0; i < 0xFFFF; i++);
        
        
        // ���KEY1�Ƿ���
        if(GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0)
        {
            return KEY1_PRESSED;
        }
        // ���KEY2�Ƿ���
        else if(GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == 0)
        {
            return KEY2_PRESSED;
        }
    }
    
    return KEY_NONE;  // �ް�������
}


// 4·����ѭ����ʼ��
/* X1=E0 X2=E2 X3=E3 X4=E4 */
void TRACK_IR4_Init(void)
{
		
		GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(TRTACK_IR4_X1_CLK | TRTACK_IR4_X2_CLK | TRTACK_IR4_X3_CLK | TRTACK_IR4_X4_CLK, ENABLE); /* ʹ�� ��� �˿�ʱ�� */

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       
    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X1_PIN;         
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      /*����*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_Init(TRTACK_IR4_X1_PORT, &GPIO_InitStructure); /*��ʼ��IO*/

    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X2_PIN;
    GPIO_Init(TRTACK_IR4_X2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X3_PIN;
    GPIO_Init(TRTACK_IR4_X3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X4_PIN;
    GPIO_Init(TRTACK_IR4_X4_PORT, &GPIO_InitStructure);
}



/***********************************************
    ultrasonic_sensor_init()
    ���ܽ��ܣ���ʼ��������������
    ������������
    ����ֵ��	��
 ***********************************************/
void ultrasonic_sensor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // ʹ��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);  // ��ʱ��13ʱ��
    RCC_AHB1PeriphClockCmd(TRIG_RCC, ENABLE);  // GPIOBʱ�ӣ��޸�ΪGPIOB��

    // ��ʼ��Trig���ţ�PB6 - �����
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // �������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       // �������������ģʽ�ɲ��ã�
    GPIO_Init(TRIG_PORT, &GPIO_InitStructure);
    
    // ��ʼ��Echo���ţ�PB7 - ���룩
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       // ��������������ʵ������ɸ�Ϊ������
    GPIO_Init(ECHO_PORT, &GPIO_InitStructure);
    
    // ��ʼ����ʱ��13�����ڼ�ʱ��
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 167;             // 168MHz/168=1MHz����������1us��
    TIM_TimeBaseStructure.TIM_Period = 65535;              // ������65535us��Լ65ms����Ӧ�������11�ף�
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
    
    // ��ʼ״̬Trig�õ�
    TRIG_LOW();
}

/**
 * @��������: ��ȡ����
 * @return {float}���ؾ���(cm)��-1��ʾ����������Χ��-2��ʾ��ʱ����
 * ����=�ߵ�ƽʱ��*���٣�340M/S�� /2
 */
float sensor_sr_ultrasonic_read(void)
{
    uint16_t time_us = 0;
    uint32_t timeout = 0;
    
    // ����10us���ϵĴ�������
    TRIG_HIGH();
    Delay_us(20);  // 20us��������
    TRIG_LOW();
    
    // �ȴ�Echo��Ϊ�ߵ�ƽ���ȴ�������ʼ��
    timeout = 0;
    while (ECHO_STATE() == 0)
    {
        Delay_us(1);
        timeout++;
        if (timeout > 10000)  // ��ʱ10ms���޻����źţ�
        {
            return -2.0f;
        }
    }
    
    // ��ʼ��ʱ
    TIM_SetCounter(TIM13, 0);
    TIM_Cmd(TIM13, ENABLE);
    
    // �ȴ�Echo��Ϊ�͵�ƽ���ȴ�����������
    timeout = 0;
    while (ECHO_STATE() == 1)
    {
        Delay_us(1);
        timeout++;
        if (timeout > 60000)  // ��ʱ60ms��������������Χ��
        {
            TIM_Cmd(TIM13, DISABLE);
            return -1.0f;
        }
    }
    
    // ֹͣ��ʱ����ȡʱ��
    TIM_Cmd(TIM13, DISABLE);
    time_us = TIM_GetCounter(TIM13);
    
    // ������루340m/s = 0.034cm/us���������������2��
    if (time_us < 60000)  // С��60ms��Լ10.2�ף�
    {
        return (float)time_us * 0.034f / 2.0f;
    }
    else
    {
        return -1.0f;  // ����������Χ
    }
}

/**
  * @��  ��  ��ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void BEEP_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//GPIO���� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	//����GPIO  �������ģʽ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//�رշ�����
	GPIO_ResetBits(GPIOC,  GPIO_Pin_12);	

}	

/**
 * @��������: ����������ʱ�䣬��λms
 * @param {uint16_t} times
 * @return {*}
 */
void beep_on_times(int times, int delay)
{
    int i;
    for (i = 0; i < times; i++)
    {
        BEEP_On();
        Delay_ms(delay);
        BEEP_Off();
        Delay_ms(delay);
    }
}


// LED��ʼ������
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ʹ��GPIODʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // ����PD14��PD15Ϊ���
    GPIO_InitStructure.GPIO_Pin = LED1_PIN | LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        // ��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // ��������
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    
    // Ĭ��Ϩ�𣨸ߵ�ƽϨ�𣬿ɸ���Ӳ���޸ģ�
    GPIO_ResetBits(LED_PORT, LED1_PIN | LED2_PIN);
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 

    // ����PC2Ϊ��������ģʽ (HAL�ⷽʽ)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        // ��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        // �������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      // ��������
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    // Ĭ��Ϩ�𣨸ߵ�ƽϨ�𣬿ɸ���Ӳ���޸ģ�
    GPIO_SetBits(GPIOC,GPIO_Pin_2);
}

// LED1����
void LED1_On(void)
{
    GPIO_ResetBits(LED_PORT, LED1_PIN);  // ����͵�ƽ����
}

// LED1Ϩ��
void LED1_Off(void)
{
    GPIO_SetBits(LED_PORT, LED1_PIN);    // ����ߵ�ƽϨ��
}

// LED1״̬��ת
void LED1_Turn(void)
{
    // ��ȡ��ǰ״̬����ת
    if(GPIO_ReadOutputDataBit(LED_PORT, LED1_PIN))
    {
        LED1_On();
    }
    else
    {
        LED1_Off();
    }
}

// LED2����
void LED2_On(void)
{
    GPIO_ResetBits(LED_PORT, LED2_PIN);  // ����͵�ƽ����
}

// LED2Ϩ��
void LED2_Off(void)
{
    GPIO_SetBits(LED_PORT, LED2_PIN);    // ����ߵ�ƽϨ��
}

// LED2״̬��ת
void LED2_Turn(void)
{
    // ��ȡ��ǰ״̬����ת
    if(GPIO_ReadOutputDataBit(LED_PORT, LED2_PIN))
    {
        LED2_On();
    }
    else
    {
        LED2_Off();
    }
}
    

