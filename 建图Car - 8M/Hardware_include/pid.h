#ifndef __PID_H
#define __PID_H

#include "main.h"




// ����ٶ�PID����
#define MOTOR_KP  20.0f
#define MOTOR_KI  0.6f
#define MOTOR_KD  0.1f
#define MOTOR_MIN -4200   // �����С���ֵ
#define MOTOR_MAX 4200    // ���������ֵ

// PID�������ṹ��
typedef struct {
    // PID����
    float kp;               // ����ϵ��
    float ki;               // ����ϵ��
    float kd;               // ΢��ϵ��
    
    // �������
    int16_t output_min;     // ��С���
    int16_t output_max;     // ������
    
    // �ڲ�����
    float target;           // Ŀ��ֵ
    float current;          // ��ǰֵ
    float error;            // ��ǰ���
    float last_error;       // ��һ�����
    float integral;         // ������
    float derivative;       // ΢����
    int16_t output;         // ���ֵ
} PID_HandleTypeDef;



// ��������
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, 
             int16_t output_min, int16_t output_max);
void PID_SetTarget(PID_HandleTypeDef *pid, float target);
int16_t PID_Compute(PID_HandleTypeDef *pid, float current);
void PID_Reset(PID_HandleTypeDef *pid);

#endif /* __PID_H */
