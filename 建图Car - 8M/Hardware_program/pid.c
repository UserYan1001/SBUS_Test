#include "pid.h"
#include <math.h>

/**
 * @brief ��ʼ��PID������
 * @param pid: PID�ṹ��ָ��
 * @param kp: ����ϵ��
 * @param ki: ����ϵ��
 * @param kd: ΢��ϵ��
 * @param output_min: ��С�������
 * @param output_max: ����������
 */
void PID_Init(PID_HandleTypeDef *pid, float kp, float ki, float kd, 
             int16_t output_min, int16_t output_max) {
    if (pid == NULL) return;
    
    // ��ʼ��PID����
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // �����������
    pid->output_min = output_min;
    pid->output_max = output_max;
    
    // ��ʼ���ڲ�����
    PID_Reset(pid);
}

/**
 * @brief ����PID��������Ŀ��ֵ
 * @param pid: PID�ṹ��ָ��
 * @param target: Ŀ��ֵ
 */
void PID_SetTarget(PID_HandleTypeDef *pid, float target) {
    if (pid == NULL) return;
    pid->target = target;
}

/**
 * @brief ����PID���
 * @param pid: PID�ṹ��ָ��
 * @param current: ��ǰ����ֵ
 * @return PID���ֵ
 */
int16_t PID_Compute(PID_HandleTypeDef *pid, float current) {
    if (pid == NULL) return 0;
    
    // ���浱ǰֵ
    pid->current = current;
    
    // ���㵱ǰ���
    pid->error = pid->target - current;
    
    // ��������� (�������ֱ���)
    pid->integral += pid->error * pid->ki;
    
    // �����޷�����ֹ���ֱ���
    if (pid->integral > pid->output_max) {
        pid->integral = pid->output_max;
    } else if (pid->integral < pid->output_min) {
        pid->integral = pid->output_min;
    }
    
    // ����΢���� (��һ�׵�ͨ�˲�˼�룬��������Ӱ��)
    pid->derivative = pid->kd * (pid->error - pid->last_error);
    
    // ����PID���
    pid->output = pid->kp * pid->error + pid->integral + pid->derivative;
    
    // ����޷�
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // ���浱ǰ��������´μ���΢��
    pid->last_error = pid->error;
    if(pid->output < 100 && pid->output > -100)
			pid->output = 0;
    return (int16_t)pid->output;
}

/**
 * @brief ����PID�������ڲ�״̬
 * @param pid: PID�ṹ��ָ��
 */
void PID_Reset(PID_HandleTypeDef *pid) {
    if (pid == NULL) return;
    
    pid->target = 0;
    pid->current = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}
