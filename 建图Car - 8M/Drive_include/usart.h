#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "global.h"

//-----------------------------UART1��ͨ����-------------------------------------
void UART1_Init(uint32_t baud); 
void uart1_send_str(u8 *s);
void uart1_send_byte(u8 dat);

//-----------------------------UART2��ͨ����-------------------------------------
void UART2_Init(uint32_t baud);
void uart2_send_str(u8 *s);

void Send_Number(uint32_t Number, uint8_t len);

//-----------------------------UART3���ߴ���-------------------------------------
#define readOnly(x)	x->CR1 |= 4;	x->CR1 &= 0xFFFFFFF7;		//����x����Ϊֻ����CR1->RE=1, CR1->TE=0
#define sendOnly(x)	x->CR1 |= 8;	x->CR1 &= 0xFFFFFFFB;		//����x����Ϊֻд��CR1->RE=0, CR1->TE=1
void USART3_Init(uint32_t baudrate);
void uart3_send_str(uint8_t *s);


//-----------------------------UART4��ͨ����-------------------------------------
void UART4_Init(uint32_t baud);
void uart4_send_str(u8 *s);


// SBUS֡����
#define SBUS_FRAME_LENGTH 25

// �ⲿ�����������ͱ�־λ
extern uint8_t uart_sbus_rx_buf[SBUS_FRAME_LENGTH];
extern uint8_t uart_sbus_rx_complete;
extern uint16_t sbus_buf[16];  // SBUSͨ����16��ͨ��
//-----------------------------UART5-SBUS-����-----------------------------------
void SBUS_Init(void);  //SBUS_Init���ڳ�ʼ��



//-----------------------------UART6���ߴ���-------------------------------------
void USART6_Init(uint32_t baudrate);
void uart6_send_str(uint8_t *s);

void analysis(char *a,double *number);

#endif 
