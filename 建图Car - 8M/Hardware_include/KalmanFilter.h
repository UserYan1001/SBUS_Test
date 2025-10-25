#ifndef __KALMANFILTER_H
#define __KALMANFILTER_H

#include <stm32f4xx.h>

// �������˲����ṹ��
typedef struct {
    float Q; // ������������
    float R; // ������������
    float P; // �������Э����
    float K; // ����������
    float estimate; // ����ֵ
} KalmanFilter;

// Ϊ�����ᴴ���˲���ʵ��
extern KalmanFilter kf_x;
extern KalmanFilter kf_y;
extern KalmanFilter kf_z;


extern float gyro_offset_gx;
extern float gyro_offset_gy;
extern float gyro_offset_gz;
extern float z_angle;




// �������˲����º���
float kalman_update(KalmanFilter *kf, float measurement);
void Reduce_error(void);
void Update_Angle_Z(float gyro_z);


#endif

