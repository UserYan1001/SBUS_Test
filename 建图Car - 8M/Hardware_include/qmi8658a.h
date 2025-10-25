#ifndef __QMI8658A_H
#define __QMI8658A_H

#include "stm32f4xx.h"
#include "myiic.h"

// QMI8658A�豸��ַ
#define QMI8658A_ADDR 0x6A << 1  // ����SA0��GND

// ���������ݽṹ��
typedef struct {
    float ax;  // ���ٶ�X�� (g)
    float ay;  // ���ٶ�Y�� (g)
    float az;  // ���ٶ�Z�� (g)
    float gx;  // ������X�� (��/s)
    float gy;  // ������Y�� (��/s)
    float gz;  // ������Z�� (��/s)
    float temp;// �¶� (��C)
} QMI8658A_Data;

uint8_t QMI8658A_Init(void) ;
uint8_t QMI8658A_GetAllData(QMI8658A_Data *data) ;
static uint8_t QMI8658A_WriteReg(uint8_t reg, uint8_t data) ;
static uint8_t QMI8658A_ReadReg(uint8_t reg) ;
static uint8_t QMI8658A_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) ;
void Get_Gyr_Z(QMI8658A_Data *data);
#endif



