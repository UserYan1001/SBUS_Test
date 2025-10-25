

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KINEMATICS_H
#define __KINEMATICS_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"




//�������˶�ѧ��������
void ROBOT_Kinematics(void);

//�����˾�ֹ����
void ROBOT_Stop(void);

typedef struct {
	float L0;
	float L1;
	float L2;
	float L3;
	
	float servo_angle[6];	//0�ŵ�4�Ŷ���ĽǶ�
	float servo_range[6];		//����Ƕȷ�Χ
	int servo_pwm[6];		//0�ŵ�4�Ŷ���ĽǶ�
}kinematics_t;

extern kinematics_t kinematics;
extern u8 zhixing_flag;

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics);
int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics);
extern float odom_x,odom_y;
#endif

/******************* (C) ��Ȩ 2022 XTARK **************************************/
