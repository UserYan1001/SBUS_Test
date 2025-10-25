//############################################################
// FILE: Common_Math.h
// Created on: 2019��7��10��
// Author: lee
// summary:    Header file  and definition
//############################################################

#ifndef _Common_Math_H_
#define _Common_Math_H_

#define Abs(A)    ((A>=0)?A:-A)  // ����ֵ����
#define Min(A,B)  ((A<=B)?A:B)   // ����С����
#define Max(A,B)  ((A>=B)?A:B)   // �������

#include "stdint.h"

typedef struct
{
	uint16_t  table_Angle;
	float table_Sin;
	float table_Cos;
}Ang_SinCos, *p_Ang_SinCos;

#define  Ang_SinCos_DEFAULTS    {0,0.0,0.0}

typedef struct 	{
	        float    Alpha; 	//���ྲֹ����ϵ Alpha ��
	        float    Beta;		//���ྲֹ����ϵ Beta ��
	        float    IQTan;		//IQ��ʽ���� 45��������1��IQ�ĸ�ʽ��
  			  int16_t  IQAngle;	//IQ��ʽ�Ƕ�ֵ 0---65536 == 0---360��
         } IQAtan , *p_IQAtan;

#define IQAtan_DEFAULTS  {0.0,0.0,0.0,0}  // ��ʼ������

typedef struct
 {  //ָ�������б�ʴ���
  float  XieLv_X;   // ָ�����б���������x
	float  XieLv_Y;
	float  XieLv_Grad;
	uint16_t    Timer_Count;
	uint16_t    Grad_Timer;
 }GXieLv, *p_GXieLv;

 #define  GXieLv_DEFAULTS    {0.0,0.0,0.0,0,0}


void LookUp_CosSin(void);
float  Limit_Sat( float Uint,float U_max, float U_min); //���Ƹ�ֵ����
uint32_t   IQSqrt(uint32_t  M);
void Atan_Cale(p_IQAtan pV);  // ��ȡ�����Һ���
void SinCos_Table(p_Ang_SinCos PV);
void Grad_XieLv(p_GXieLv pV);

#endif /* SIN_COS_TABLE1_H_ */
