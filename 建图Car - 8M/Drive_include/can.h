
#ifndef __CAN_H
#define __CAN_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//X-SOFT �ӿں���
void CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);  //CAN��ʼ��
uint8_t CAN_SendMsg(uint8_t* msg,uint8_t num);    //CAN��������
uint8_t CAN_RecvMsg(uint8_t *msg);    //CAN��������

#endif

/******************* (C) ��Ȩ 2022 XTARK **************************************/
