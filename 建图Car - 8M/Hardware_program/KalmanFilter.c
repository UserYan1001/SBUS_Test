#include "KalmanFilter.h"
#include "qmi8658a.h"


float gyro_offset_gx = 0.0f;
float gyro_offset_gy = 0.0f;
float gyro_offset_gz = 0.0f;
float z_angle = 0.0f;


KalmanFilter kf_x = {0.1f, 1.0f, 1.0f, 0.0f, 0.0f};
KalmanFilter kf_y = {0.1f, 1.0f, 1.0f, 0.0f, 0.0f};
KalmanFilter kf_z = {0.1f, 1.0f, 1.0f, 0.0f, 0.0f};

// �������˲����º���
float kalman_update(KalmanFilter *kf, float measurement) {
    // Ԥ�ⲽ��
    kf->P = kf->P + kf->Q;
    
    // ���²���
    kf->K = kf->P / (kf->P + kf->R);
    kf->estimate = kf->estimate + kf->K * (measurement - kf->estimate);
    kf->P = (1 - kf->K) * kf->P;
    
    return kf->estimate;
}

void Reduce_error(void) {
	
    QMI8658A_Data qdata_t;
    float sum_gx = 0.0f;
		float sum_gy = 0.0f;
		float sum_gz = 0.0f;
    int count = 5;
    
    // �ȴ��������ȶ�
    Delay_ms(3000);
    
    // �ۼ�1000������
    for (int i = 0; i < count; i++) {
        QMI8658A_GetAllData(&qdata_t);
        sum_gx += qdata_t.gx;
				sum_gy += qdata_t.gy;
				sum_gz += qdata_t.gz;
        Delay_ms(20); 
    }
    
    // ֱ�����ֵ��Ϊ��ƫ
    gyro_offset_gx = sum_gx / count;
		gyro_offset_gy = sum_gy / count;
		gyro_offset_gz = sum_gz / count;
}


// ����Z��Ƕ�
// ������gyro_z - Z������������
void Update_Angle_Z(float gyro_z)
{
    // ����΢С�仯���쳣��ֵ�������Ǵ���������
    if ((gyro_z >= -15.0f && gyro_z <= 15.0f))
    {
        return;
    }
    
    // ����������ת���������Ƕ�/�룩
    gyro_z /= 15.7f;
    
    unsigned long current_time = millis();
    static unsigned long last_time = 0; 
    float dt = 0.0f;
    
    // �״ε��ó�ʼ��ʱ��
    if (last_time == 0)
    {
        last_time = current_time;
        return;
    }
    
    // ����ʱ�������룩
    dt = (current_time - last_time) / 1000.0f;
    
    // ��ֹʱ���������µĽǶ�����
    if (dt > 0.5f)  // ����0.5������ݲ��ɿ�
    {
        last_time = current_time;
        return;
    }
    
    // ����Ƕȱ仯��
    float angle_delta = gyro_z * dt;
    
    // ���˹���ĽǶȱ仯
//    if (angle_delta >= 50.0f || angle_delta <= -50.0f)
//    {
//        last_time = current_time;
//        return;
//    }
    
    // ���½ǶȲ���һ����[-360, 360]��Χ
    z_angle += angle_delta;
    
    // ��һ���Ƕȣ�����ԭ�߼����󣬱��ֽǶ������ԣ�
    while (z_angle > 360.0f)  z_angle -= 360.0f;
    while (z_angle < -360.0f) z_angle += 360.0f;
    
    last_time = current_time;
}
