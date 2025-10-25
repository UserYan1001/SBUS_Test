

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Y_SERVO_H
#define __Y_SERVO_H

#include "main.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* QSA */
#define SERVO0_PIN GPIO_Pin_7
#define SERVO0_GPIO_PORT GPIOE               /* GPIO�˿� */
#define SERVO0_GPIO_CLK RCC_AHB1Periph_GPIOE /* GPIO�˿�ʱ�� */

#define SERVO1_PIN GPIO_Pin_8
#define SERVO1_GPIO_PORT GPIOE               /* GPIO�˿� */
#define SERVO1_GPIO_CLK RCC_AHB1Periph_GPIOE /* GPIO�˿�ʱ�� */

#define SERVO2_PIN GPIO_Pin_0
#define SERVO2_GPIO_PORT GPIOB               /* GPIO�˿� */
#define SERVO2_GPIO_CLK RCC_AHB1Periph_GPIOB /* GPIO�˿�ʱ�� */

#define SERVO3_PIN GPIO_Pin_1
#define SERVO3_GPIO_PORT GPIOB               /* GPIO�˿� */
#define SERVO3_GPIO_CLK RCC_AHB1Periph_GPIOB /* GPIO�˿�ʱ�� */

#define SERVO4_PIN GPIO_Pin_8
#define SERVO4_GPIO_PORT GPIOC               /* GPIO�˿� */
#define SERVO4_GPIO_CLK RCC_AHB1Periph_GPIOC /* GPIO�˿�ʱ�� */

#define SERVO5_PIN GPIO_Pin_9
#define SERVO5_GPIO_PORT GPIOC               /* GPIO�˿� */
#define SERVO5_GPIO_CLK RCC_AHB1Periph_GPIOC /* GPIO�˿�ʱ�� */

/* ���ƶ����������ĺ� */
#define SERVO0_PIN_SET(level) GPIO_WriteBit(SERVO0_GPIO_PORT, SERVO0_PIN, level)
#define SERVO1_PIN_SET(level) GPIO_WriteBit(SERVO1_GPIO_PORT, SERVO1_PIN, level)
#define SERVO2_PIN_SET(level) GPIO_WriteBit(SERVO2_GPIO_PORT, SERVO2_PIN, level)
#define SERVO3_PIN_SET(level) GPIO_WriteBit(SERVO3_GPIO_PORT, SERVO3_PIN, level)
#define SERVO4_PIN_SET(level) GPIO_WriteBit(SERVO4_GPIO_PORT, SERVO4_PIN, level)
#define SERVO5_PIN_SET(level) GPIO_WriteBit(SERVO5_GPIO_PORT, SERVO5_PIN, level)

#define DJ_NUM 8 /* ���������Ϊ8����Ϊ��ʱ���жϼ���pwm������Ҫ */

typedef struct
{
    // uint8_t valid; // ��Ч TODO
    uint16_t aim;  // ִ��Ŀ��
    uint16_t time; // ִ��ʱ��
    float cur;     // ��ǰֵ
    float inc;     // ����
} servo_t;

extern servo_t duoji_doing[DJ_NUM];

// X-SOFT �ӿں���
void SERVO_Init(void); // ����ӿڳ�ʼ��
void servo_pin_set(u8 index, BitAction level);
void duoji_doing_set(u8 index, int aim, int time);

void TIM14_Int_Init(u16 arr, u16 psc);

#endif

/******************* (C) ��Ȩ 2022 XTARK **************************************/
