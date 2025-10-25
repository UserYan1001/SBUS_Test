#ifndef __WS2812B_H
#define __WS2812B_H

#include "main.h"

// 定义WS2812B连接的GPIO引脚和端口
#define WS2812B_GPIO_PORT    GPIOA
#define WS2812B_GPIO_PIN     GPIO_Pin_8
#define WS2812B_GPIO_CLK     RCC_AHB1Periph_GPIOA

// 定义LED数量
#define LED_COUNT            20   // 根据实际LED数量修改

// RGB颜色结构
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} WS2812B_ColorTypeDef;

// 初始化函数
void WS2812B_Init(void);

// 设置单个LED颜色
void WS2812B_SetLEDColor(uint16_t ledIndex, WS2812B_ColorTypeDef color);

// 设置所有LED颜色
void WS2812B_SetAllLEDColor(WS2812B_ColorTypeDef color);

// 发送颜色数据到LED
void WS2812B_Update(void);

// 创建颜色
WS2812B_ColorTypeDef WS2812B_CreateColor(uint8_t red, uint8_t green, uint8_t blue);

#endif /* __WS2812B_H */
