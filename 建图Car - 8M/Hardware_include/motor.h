/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Y_MOTOR_H
#define __Y_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"



// ������ÿת������
#define ENCODER_PPR 1555   // 26 * 2 * 30

//���PID�ջ��ٶȿ��ƺ���
//int16_t SPEED_PidCtlA(float spd_target, float spd_current);   //PID���ƺ��������A
//int16_t SPEED_PidCtlB(float spd_target, float spd_current);    //PID���ƺ��������B
//int16_t SPEED_PidCtlC(float spd_target, float spd_current);    //PID���ƺ��������C
//int16_t SPEED_PidCtlD(float spd_target, float spd_current);    //PID���ƺ��������D

void SPEED_PidResetA(void);
void SPEED_PidResetB(void);
void SPEED_PidResetC(void);
void SPEED_PidResetD(void);

void MOTOR_A_SetSpeed(int16_t speed);   //���A����
void MOTOR_B_SetSpeed(int16_t speed);   //���B����

void MOTOR_C_SetSpeed(int16_t speed);   //���C����
void MOTOR_D_SetSpeed(int16_t speed);   //���D����

typedef struct
{
	double	Target_val;//Ŀ��ֵ
	double	Actual_val;//ʵ��ֵ
	double 	err;//��ǰƫ��
	double	last_error;//�ϴ�ƫ��
	double  Prev_Error;//ǰ�������
	double	sum_error;//����ۼ�ֵ
	double	result;//����ۼ�ֵ
    double maximum;   //���ֵ
    double minimum;   //��Сֵ
    double  kp, ki, kd, dt;//����kp ����ʱ��ti ΢��ʱ��td ִ��ʱ��dt(ע��ͳһʱ�䵥λ)
} tpid;
extern tpid MotorA;
extern tpid MotorB;
extern tpid MotorC;
extern tpid MotorD;
extern tpid pidAngle;
//void MOTORS_AllRunCircle(int8_t cntA, int8_t cntB, int8_t cntC, int8_t cntD);//�ĸ����һ��
//void MOTOR_A_RunCircle(int8_t cnt);		//���A��Ȧ����
//void MOTOR_B_RunCircle(int8_t cnt);		//���B��Ȧ����
//void MOTOR_C_RunCircle(int8_t cnt);		//���C��Ȧ����
//void MOTOR_D_RunCircle(int8_t cnt);		//���D��Ȧ����

//float Calculate_Motor_A_RPM(void);			// ������A��ת��
//float Calculate_Motor_B_RPM(void);			// ������B��ת��
//float Calculate_Motor_C_RPM(void);			// ������C��ת��
//float Calculate_Motor_D_RPM(void);			// ������D��ת��

//float MOTOR_A_SpeedControl(float target_rpm);		//���A��PID����
//float MOTOR_B_SpeedControl(float target_rpm);		//���B��PID����
//float MOTOR_C_SpeedControl(float target_rpm);		//���C��PID����
//float MOTOR_D_SpeedControl(float target_rpm);		//���D��PID����


float PID_Calculate(tpid *pid,float Actual_val);
float PID_Realize(tpid *pid,float Actual_val);
float angle_pid(float angle,float gyro,float target);
int16_t motorA_pid(float encoder,float target);
int16_t motorB_pid(float encoder,float target);
int16_t motorC_pid(float encoder,float target);
int16_t motorD_pid(float encoder,float target);
void Pid_Init(void);
extern int Arrive_flag;
void I_limit(tpid * pid, float low, float high);
void motor_doing_set(u8 index, int vel, int time);

#endif

/******************* (C) ��Ȩ 2022 XTARK **************************************/
