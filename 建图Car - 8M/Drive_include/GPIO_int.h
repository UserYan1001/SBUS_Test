

#ifndef _GPIO_int_H
#define _GPIO_int_H
#include "Sys.h"    


//----------------------------------����--------------------------------------------------
// �������Ŷ���
#define KEY1_PIN    GPIO_Pin_10
#define KEY1_PORT   GPIOE
#define KEY2_PIN    GPIO_Pin_15
#define KEY2_PORT   GPIOC
// ����״̬����
#define KEY1_PRESSED  1
#define KEY2_PRESSED  2
#define KEY_NONE      0
//������������
void KEY_Init(void);
uint8_t KEY_Scan(void);
//----------------------------------����--------------------------------------------------








//----------------------------------��·ѭ��ģ��----------------------------------------------


void TRACK_IR4_Init(void);
/* ѭ�����Ŷ�ȡ��������1-��4 */
#define TRTACK_IR4_X1_PIN GPIO_Pin_0
#define TRTACK_IR4_X1_PORT GPIOE               /* GPIO�˿� */
#define TRTACK_IR4_X1_CLK RCC_AHB1Periph_GPIOE /* GPIO�˿�ʱ�� */

#define TRTACK_IR4_X2_PIN GPIO_Pin_2
#define TRTACK_IR4_X2_PORT GPIOE               /* GPIO�˿� */
#define TRTACK_IR4_X2_CLK RCC_AHB1Periph_GPIOE /* GPIO�˿�ʱ�� */

#define TRTACK_IR4_X3_PIN GPIO_Pin_3
#define TRTACK_IR4_X3_PORT GPIOE               /* GPIO�˿� */
#define TRTACK_IR4_X3_CLK RCC_AHB1Periph_GPIOE /* GPIO�˿�ʱ�� */

#define TRTACK_IR4_X4_PIN GPIO_Pin_4
#define TRTACK_IR4_X4_PORT GPIOE               /* GPIO�˿� */
#define TRTACK_IR4_X4_CLK RCC_AHB1Periph_GPIOE /* GPIO�˿�ʱ�� */

#define TRTACK_IR4_X1_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X1_PORT,TRTACK_IR4_X1_PIN)
#define TRTACK_IR4_X2_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X2_PORT,TRTACK_IR4_X2_PIN)
#define TRTACK_IR4_X3_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X3_PORT,TRTACK_IR4_X3_PIN)
#define TRTACK_IR4_X4_READ() GPIO_ReadInputDataBit(TRTACK_IR4_X4_PORT,TRTACK_IR4_X4_PIN)

//----------------------------------��·ѭ��ģ��----------------------------------------------






 
//------------------------------------������ģ��--------------------------------------------
//���Ŷ����
//������һOK
#define TRIG_PORT       GPIOB
#define TRIG_PIN        GPIO_Pin_6
#define ECHO_PORT       GPIOB
#define ECHO_PIN        GPIO_Pin_7
#define TRIG_RCC				RCC_AHB1Periph_GPIOB


//��������OK
//#define TRIG_PORT       GPIOE
//#define TRIG_PIN        GPIO_Pin_1
//#define ECHO_PORT       GPIOE
//#define ECHO_PIN        GPIO_Pin_12
//#define TRIG_RCC				RCC_AHB1Periph_GPIOE
// �����궨��
#define TRIG_HIGH()     GPIO_SetBits(TRIG_PORT, TRIG_PIN)
#define TRIG_LOW()      GPIO_ResetBits(TRIG_PORT, TRIG_PIN)
#define ECHO_STATE()    GPIO_ReadInputDataBit(ECHO_PORT, ECHO_PIN)

// ��������
void ultrasonic_sensor_init(void);
float sensor_sr_ultrasonic_read(void);

//------------------------------------������ģ��--------------------------------------------









//------------------------------------������------------------------------------------------
// �ӿں���
void BEEP_Init(void);
// ���������������궨��
#define BEEP_Off()       GPIO_ResetBits(GPIOC, GPIO_Pin_12)       //����������
#define BEEP_On()		GPIO_SetBits(GPIOC, GPIO_Pin_12)     //����������
#define BEEP_Toggle()    GPIO_WriteBit(GPIOC, GPIO_Pin_12, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)))    //������״̬��ת
void beep_on_times(int times, int delay);

//------------------------------------������------------------------------------------------




//------------------------------------LED������---------------------------------------------
// �궨��LED����
#define LED1_PIN    GPIO_Pin_14
#define LED2_PIN    GPIO_Pin_15
#define LED_PORT    GPIOD

// ��������
void LED_Init(void);
void LED1_On(void);
void LED1_Off(void);
void LED1_Turn(void);
void LED2_On(void);
void LED2_Off(void);
void LED2_Turn(void);
//------------------------------------LED������---------------------------------------------

#endif   

