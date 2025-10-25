#ifndef __WS2812B_H
#define __WS2812B_H

#include "main.h"

// ����WS2812B���ӵ�GPIO���źͶ˿�
#define WS2812B_GPIO_PORT    GPIOA
#define WS2812B_GPIO_PIN     GPIO_Pin_8
#define WS2812B_GPIO_CLK     RCC_AHB1Periph_GPIOA

// ����LED����
#define LED_COUNT            20   // ����ʵ��LED�����޸�

// RGB��ɫ�ṹ
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} WS2812B_ColorTypeDef;

// ��ʼ������
void WS2812B_Init(void);

// ���õ���LED��ɫ
void WS2812B_SetLEDColor(uint16_t ledIndex, WS2812B_ColorTypeDef color);

// ��������LED��ɫ
void WS2812B_SetAllLEDColor(WS2812B_ColorTypeDef color);

// ������ɫ���ݵ�LED
void WS2812B_Update(void);

// ������ɫ
WS2812B_ColorTypeDef WS2812B_CreateColor(uint8_t red, uint8_t green, uint8_t blue);

#endif /* __WS2812B_H */
