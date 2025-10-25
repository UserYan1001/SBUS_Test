/****************************************************************************
 *	@����	��	YJH
 *	@����	��	2025-09-09
 *	@����	��	�������¿Ƽ����޹�˾
 *****************************************************************************/
#include "qmi8658a.h"
//#include <stdio.h>
// �Ĵ�����ַ
#define QMI8658A_WHO_AM_I 0x00
#define QMI8658A_CTRL1    0x02
#define QMI8658A_CTRL2    0x03
#define QMI8658A_CTRL3    0x04
#define QMI8658A_CTRL5    0x06
#define QMI8658A_CTRL7    0x08
#define AccX_L 0x35
#define AccX_H 0x36
#define AccY_L 0x37
#define AccY_H 0x38
#define AccZ_L 0x39
#define AccZ_H 0x3A


#define GyrX_L 0x3B
#define GyrX_H 0x3C
#define GyrY_L 0x3D
#define GyrY_H 0x3E
#define GyrZ_L 0x3F
#define GyrZ_H 0x40

// �豸ID
#define QMI8658A_ID 0x05

#include "stdio.h"
// ת������
static const float acc_factor = 16384.0f;  // ��2g����
static const float gyr_factor = 1.025f;   // ��2048��/s����

/**
 * @brief  ��ʼ��QMI8658A������
 * @retval 0: �ɹ�, 1: ʧ��
 */
uint8_t QMI8658A_Init(void) {
    uint8_t id;
    
    // ��ʼ��IIC
    IIC_Init();
    
    // ��ȡ�豸ID
    id = QMI8658A_ReadReg(QMI8658A_WHO_AM_I);
		printf(" Id : %2d\r\n",id);
    if (id != QMI8658A_ID) {
        return 1; // �豸ID����ȷ
    }
    
		
    // �����λ
    QMI8658A_WriteReg(QMI8658A_CTRL1, 0x80);
    Delay_ms(10);
    
    // ���ô�����: ʹ�ܼ��ٶȺ������ǣ�Ĭ�����̺��������
    QMI8658A_WriteReg(QMI8658A_CTRL1, 0x40);  
    QMI8658A_WriteReg(QMI8658A_CTRL2, 0x06);  // <��2g  125Hz>
		QMI8658A_WriteReg(QMI8658A_CTRL3, 0x76);  // < ��2048dps 125Hz>
    QMI8658A_WriteReg(QMI8658A_CTRL5, 0x11);  // <Enable Gyroscope Accelerometer ��ͨ�˲�>
		QMI8658A_WriteReg(QMI8658A_CTRL7, 0x03);  // ʹ�ܼ��ٶȺ�������
		

    return 0;
}

// �����������
#define SAMPLE_COUNT 5
/**
 * @brief  ֱ�Ӷ�ȡ���д���������
 * @param  data: �洢���ݵĽṹ��ָ��
 * @retval 0: �ɹ�, 1: ʧ��
 */
uint8_t QMI8658A_GetAllData(QMI8658A_Data *data) {
    
		uint8_t buf[14];
		

		// �������ٶ�X�����ݣ�10�β�����ƽ����
		memset(buf, 0, sizeof(buf));
		int16_t ax_samples[SAMPLE_COUNT];
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				QMI8658A_ReadRegs(AccX_L, buf, 2);
				ax_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
		}
		// ����ƽ��ֵ
		int32_t ax_sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				ax_sum += ax_samples[i];
		}
		data->ax = (float)(ax_sum / SAMPLE_COUNT) / acc_factor;

		// �������ٶ�Y�����ݣ�10�β�����ƽ����
		memset(buf, 0, sizeof(buf));
		int16_t ay_samples[SAMPLE_COUNT];
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				QMI8658A_ReadRegs(AccY_L, buf, 2);
				ay_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
		}
		// ����ƽ��ֵ
		int32_t ay_sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				ay_sum += ay_samples[i];
		}
		data->ay = (float)(ay_sum / SAMPLE_COUNT) / acc_factor;

		// �������ٶ�Z�����ݣ�10�β�����ƽ����
		memset(buf, 0, sizeof(buf));
		int16_t az_samples[SAMPLE_COUNT];
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				QMI8658A_ReadRegs(AccZ_L, buf, 2);
				az_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
		}
		// ����ƽ��ֵ
		int32_t az_sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				az_sum += az_samples[i];
		}
		data->az = (float)(az_sum / SAMPLE_COUNT) / acc_factor;

		// ����������X�����ݣ�10�β�����ƽ����
		memset(buf, 0, sizeof(buf));
		int16_t gx_samples[SAMPLE_COUNT];
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				QMI8658A_ReadRegs(GyrX_L, buf, 2);
				gx_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
		}
		// ����ƽ��ֵ
		int32_t gx_sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				gx_sum += gx_samples[i];
		}
		data->gx = (float)(gx_sum / SAMPLE_COUNT) / gyr_factor;

		// ����������Y�����ݣ�10�β�����ƽ����
		memset(buf, 0, sizeof(buf));
		int16_t gy_samples[SAMPLE_COUNT];
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				QMI8658A_ReadRegs(GyrY_L, buf, 2);
				gy_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
		}
		// ����ƽ��ֵ
		int32_t gy_sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				gy_sum += gy_samples[i];
		}
		data->gy = (float)(gy_sum / SAMPLE_COUNT) / gyr_factor;

		// ����������Z�����ݣ�10�β�����ƽ����
		memset(buf, 0, sizeof(buf));
		int16_t gz_samples[SAMPLE_COUNT];
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				QMI8658A_ReadRegs(GyrZ_L, buf, 2);
				gz_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
		}
		// ����ƽ��ֵ
		int32_t gz_sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
				gz_sum += gz_samples[i];
		}
		data->gz = (float)(gz_sum / SAMPLE_COUNT) / gyr_factor;

    
    return 0;
}

void Get_Gyr_Z(QMI8658A_Data *data)
{
	uint8_t buf[14];
		memset(buf, 0, sizeof(buf));
	int16_t gz_samples[SAMPLE_COUNT];
	for (int i = 0; i < SAMPLE_COUNT; i++) {
			QMI8658A_ReadRegs(GyrZ_L, buf, 2);
			gz_samples[i] = (int16_t)((buf[1] << 8) | buf[0]);
	}
	// ����ƽ��ֵ
	int32_t gz_sum = 0;
	for (int i = 0; i < SAMPLE_COUNT; i++) {
			gz_sum += gz_samples[i];
	}
	data->gz = (float)(gz_sum / SAMPLE_COUNT) / gyr_factor;
}


/**
 * @brief  ��QMI8658Aд��һ���Ĵ���
 * @param  reg: �Ĵ�����ַ
 * @param  data: Ҫд�������
 * @retval 0: �ɹ�, 1: ʧ��
 */
static uint8_t QMI8658A_WriteReg(uint8_t reg, uint8_t data) {
    IIC_Start();
    IIC_Send_Byte(QMI8658A_ADDR);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    
    IIC_Send_Byte(data);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    
    IIC_Stop();
    return 0;
}

/**
 * @brief  ��QMI8658A��ȡһ���Ĵ���
 * @param  reg: �Ĵ�����ַ
 * @retval ��ȡ��������
 */
static uint8_t QMI8658A_ReadReg(uint8_t reg) {
    uint8_t data;
    
    IIC_Start();
    IIC_Send_Byte(QMI8658A_ADDR);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0xFF; }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0xFF; }
    IIC_Start();
    IIC_Send_Byte(QMI8658A_ADDR | 0x01);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0xFF; }
    
    data = IIC_Read_Byte(0);
    IIC_Stop();
    
    return data;
}

/**
 * @brief  ��QMI8658A��ȡ����Ĵ���
 * @param  reg: ��ʼ�Ĵ�����ַ
 * @param  buf: �洢��ȡ���ݵĻ�����
 * @param  len: Ҫ��ȡ���ֽ���
 * @retval 0: �ɹ�, 1: ʧ��
 */
static uint8_t QMI8658A_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t i;
    
    IIC_Start();
    IIC_Send_Byte(QMI8658A_ADDR);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    
    IIC_Start();
    IIC_Send_Byte(QMI8658A_ADDR | 0x01);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    
    for (i = 0; i < len; i++) {
        buf[i] = IIC_Read_Byte(i == len - 1 ? 0 : 1);
    }
    
    IIC_Stop();
    return 0;
}


