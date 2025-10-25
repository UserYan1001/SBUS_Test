#ifndef __ROBOT_H
#define __ROBOT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

// C�⺯�������ͷ�ļ�
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// �������ͷ�ļ�
#include "sys.h"	  //ϵͳ����

#include "servo.h"   //�������
#include "motor.h"   //ֱ��������ٿ���

// �����������ٶ����ݽṹ��
typedef struct
{
	double RT; // ����ʵʱ�ٶȣ���λm/s
	float TG;  // ����Ŀ���ٶȣ���λm/s
	short PWM; // ����PWM�����ٶ�
} ROBOT_Wheel;

// �������ٶȽṹ��
typedef struct
{
	short RT_IX; // ʵʱX���ٶȣ�16λ������
	short RT_IY; // ʵʱY���ٶȣ�16λ������
	short RT_IW; // ʵʱYaw��ת���ٶȣ�16λ������

	short TG_IX; // Ŀ��X���ٶȣ�16λ������
	short TG_IY; // Ŀ��Y���ٶȣ�16λ������
	short TG_IW; // Ŀ��Yaw��ת���ٶȣ�16λ������

	float RT_FX; // ʵʱX���ٶȣ����㣩
	float RT_FY; // ʵʱY���ٶȣ����㣩
	float RT_FW; // ʵʱYaw��ת���ٶȣ����㣩

	float TG_FX; // Ŀ��X���ٶȣ����㣩
	float TG_FY; // Ŀ��Y���ٶȣ����㣩
	float TG_FW; // Ŀ��Yaw��ת���ٶȣ����㣩
	
	float speedX;
    float speedY;
    float speedYaw;
	

} ROBOT_Velocity;

// ������IMU����
typedef struct
{
	short ACC_X; // X��
	short ACC_Y; // Y��
	short ACC_Z; // Z��

	short GYRO_X; // X��
	short GYRO_Y; // Y��
	short GYRO_Z; // Z��

} ROBOT_Imu;

// RGB��Ч����
typedef struct
{
	uint8_t M; // ��Ч��ģʽ
	uint8_t S; // ��Ч��ģʽ
	uint8_t T; // ��Чʱ�����

	uint8_t R; // ��Ч��ɫ R
	uint8_t G; // ��Ч��ɫ G
	uint8_t B; // ��Ч��ɫ B

} ROBOT_Light;

// ������ת��ṹ��
typedef struct
{
	float Radius; // ת��뾶
	float Angle;  // ������ת��Ƕ�
	float RAngle; // ǰ����ת��Ƕ�
	float SAngle; // ����Ƕ�

} ROBOT_Steering;

// ��е�۲���

// ���
#define VBAT_40P 1065 // ���40%��ѹ
#define VBAT_20P 1012 // ���20%��ѹ
#define VBAT_10P 984  // ���10%��ѹ

#define PID_RATE 200 // PIDƵ��

// ����������
#define ROBOT_MEC 1
#define ROBOT_FWD 2
#define ROBOT_AKM 3
#define ROBOT_TWD 4

// ����������ֱ���
#define WHEEL_RESOLUTION 2464.0f // 1024�ߴű������ֱ���,1204x30x4=122880    ���ػ�����11*4*30�����ٱȣ�= 1320

// ���ֻ����˲���
#define MEC_WHEEL_BASE 0.322f													// �־࣬�����ֵľ���
#define MEC_ACLE_BASE 0.175f													// ��࣬ǰ���ֵľ���
#define MEC_WHEEL_DIAMETER 0.097f												// ����ֱ��
#define MEC_WHEEL_SCALE (PI * MEC_WHEEL_DIAMETER * PID_RATE / WHEEL_RESOLUTION) // �����ٶ�m/s�������ת��ϵ��

// ���ֲ��ٻ����˲���
#define FWD_WHEEL_BASE 0.200													// �־࣬�����ֵľ���
#define FWD_WB_SCALE 1.75														// �־�ϵ�����־�ϵ��������˵��ܸ��ء���̥���������Ħ��ϵ����ת��뾶������λ�ö����й�ϵ��һ���ǳ����ӵĲ��������Գ��õķ���������ʵ��
#define FWD_WHEEL_DIAMETER 0.100												// ����ֱ��
#define FWD_WHEEL_SCALE (PI * FWD_WHEEL_DIAMETER * PID_RATE / WHEEL_RESOLUTION) // �����ٶ�m/s�������ת��ϵ��

// ���ֻ����˲���
#define AKM_WHEEL_BASE 0.165													// �־࣬�����ֵľ���
#define AKM_ACLE_BASE 0.175f													// ��࣬ǰ���ֵľ���
#define AKM_WHEEL_DIAMETER 0.075												// ����ֱ��
#define AKM_TURN_R_MINI 0.35f													// ��Сת��뾶( L*cot30-W/2)
#define AKM_WHEEL_SCALE (PI * AKM_WHEEL_DIAMETER * PID_RATE / WHEEL_RESOLUTION) // �����ٶ�m/s�������ת��ϵ��

// ���ֲ��ٻ����˲���
#define TWD_WHEEL_DIAMETER 0.0724												// ����ֱ��
#define TWD_WHEEL_BASE 0.206													// �־࣬�����ֵľ���
#define TWD_WHEEL_SCALE (PI * TWD_WHEEL_DIAMETER * PID_RATE / WHEEL_RESOLUTION) // �����ٶ�m/s�������ת��ϵ��

// �������ٶ�����
#define R_VX_LIMIT 4200 // X���ٶ���ֵ m/s*1000
#define R_VY_LIMIT 4200 // Y���ٶ���ֵ m/s*1000
#define R_VW_LIMIT 6280 // W��ת���ٶ���ֵ rad/s*1000


typedef struct
{
	float gpitch;
	float groll;
	float gyaw;

	short gGyro1;
	short gGyro2;
	short gGyro3;

	float Angle_Old;

	short gAx;
	short gAy;
	short gAz;
	float Temp;

} blance_samp;




#define blance_samp_DEFAULTS                            \
	{                                                   \
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0.0 \
	}

extern blance_samp blance_sampPara;

// �����˹ؼ�ȫ�ֱ���
extern ROBOT_Velocity Vel;							   // �������ٶ�����
extern ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D; // ��������������
extern uint16_t Bat_Vol;							   // �����˵�ص�ѹ����

// ȫ�ֱ���
extern int16_t imu_acc_data[3];  
extern int16_t imu_gyro_data[3];
extern int16_t imu_gyro_offset[3];   
extern float motor_kp;
extern float motor_ki;
extern float motor_kd;

	
extern float motorB_kp;
extern float motorB_ki;
extern float motorB_kd;

extern float motorC_kp;
extern float motorC_ki;
extern float motorC_kd;

extern float motorD_kp;
extern float motorD_ki;
extern float motorD_kd;

extern int8_t model_flag;

// ������������
extern int16_t servo_offset;

void Robot_Task(void);
void Bat_Task(void);
void Key_Task(void);
void ROBOT_ArmControl(void);
#endif
