/****************************************************************************
 *	@����	��	YJH
 *	@����	��	2025-09-09
 *	@����	��	�������¿Ƽ����޹�˾
 *****************************************************************************/

#include "ws2812b.h"

// �洢ÿ��LED����ɫ����
static WS2812B_ColorTypeDef ledColors[LED_COUNT];

// ���ڷ������ݵĻ�����(ÿ��LED��Ҫ24λ�����ϸ�λ��)
static uint8_t dataBuffer[(LED_COUNT * 24) + 1];

// ��ʼ��WS2812B��������
void WS2812B_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ʹ��GPIOʱ��
    RCC_AHB1PeriphClockCmd(WS2812B_GPIO_CLK, ENABLE);
    
    // ����GPIOΪ�������
    GPIO_InitStructure.GPIO_Pin = WS2812B_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(WS2812B_GPIO_PORT, &GPIO_InitStructure);
    
    // ��ʼ��Ϊ�͵�ƽ
    GPIO_ResetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
    
    // ��ʼ������LEDΪ��ɫ
    WS2812B_SetAllLEDColor(WS2812B_CreateColor(0, 0, 0));
}

// ������ɫ
WS2812B_ColorTypeDef WS2812B_CreateColor(uint8_t red, uint8_t green, uint8_t blue) {
    WS2812B_ColorTypeDef color;
    color.red = red;
    color.green = green;  // WS2812B��GRB��ʽ
    color.blue = blue;
    return color;
}

// ���õ���LED��ɫ
void WS2812B_SetLEDColor(uint16_t ledIndex, WS2812B_ColorTypeDef color) {
    if (ledIndex < LED_COUNT) {
        ledColors[ledIndex] = color;
    }
}

// ��������LED��ɫ
void WS2812B_SetAllLEDColor(WS2812B_ColorTypeDef color) {
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        ledColors[i] = color;
    }
}

// �������ݵ�WS2812B
static void WS2812B_SendData(uint8_t *data, uint32_t length) {
    // �ر������жϣ�ȷ��ʱ��׼ȷ
    __disable_irq();
    
    for (uint32_t i = 0; i < length; i++) {
        // ����8λ���ݣ���λ��ǰ
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (data[i] & (1 << (7 - bit))) {
                // ����1�룺�ߵ�ƽԼ0.8us���͵�ƽԼ0.45us
                GPIO_SetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // ��ʱԼ0.8us (����ʵ��ϵͳʱ�ӵ���)
                for (uint8_t n = 0; n < 32; n++) __NOP();
                GPIO_ResetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // ��ʱԼ0.45us
                for (uint8_t n = 0; n < 18; n++) __NOP();
            } else {
                // ����0�룺�ߵ�ƽԼ0.4us���͵�ƽԼ0.85us
                GPIO_SetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // ��ʱԼ0.4us
                for (uint8_t n = 0; n < 16; n++) __NOP();
                GPIO_ResetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // ��ʱԼ0.85us
                for (uint8_t n = 0; n < 34; n++) __NOP();
            }
        }
    }
    
    // �ָ��ж�
    __enable_irq();
}

// ����LED��ʾ
void WS2812B_Update(void) {
    uint32_t index = 0;
    
    // ������ݻ�����
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        // WS2812B����GRB��ʽ
        dataBuffer[index++] = ledColors[i].green;
        dataBuffer[index++] = ledColors[i].red;
        dataBuffer[index++] = ledColors[i].blue;
    }
    
    // ��������
    WS2812B_SendData(dataBuffer, LED_COUNT * 3);
    
    // ���͸�λ�ź�(����50us�͵�ƽ)
    Delay_us(50);
}
