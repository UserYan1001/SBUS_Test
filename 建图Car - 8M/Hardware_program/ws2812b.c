/****************************************************************************
 *	@笔者	：	YJH
 *	@日期	：	2025-09-09
 *	@所属	：	杭州星呗科技有限公司
 *****************************************************************************/

#include "ws2812b.h"

// 存储每个LED的颜色数据
static WS2812B_ColorTypeDef ledColors[LED_COUNT];

// 用于发送数据的缓冲区(每个LED需要24位，加上复位码)
static uint8_t dataBuffer[(LED_COUNT * 24) + 1];

// 初始化WS2812B控制引脚
void WS2812B_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIO时钟
    RCC_AHB1PeriphClockCmd(WS2812B_GPIO_CLK, ENABLE);
    
    // 配置GPIO为推挽输出
    GPIO_InitStructure.GPIO_Pin = WS2812B_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(WS2812B_GPIO_PORT, &GPIO_InitStructure);
    
    // 初始化为低电平
    GPIO_ResetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
    
    // 初始化所有LED为黑色
    WS2812B_SetAllLEDColor(WS2812B_CreateColor(0, 0, 0));
}

// 创建颜色
WS2812B_ColorTypeDef WS2812B_CreateColor(uint8_t red, uint8_t green, uint8_t blue) {
    WS2812B_ColorTypeDef color;
    color.red = red;
    color.green = green;  // WS2812B是GRB格式
    color.blue = blue;
    return color;
}

// 设置单个LED颜色
void WS2812B_SetLEDColor(uint16_t ledIndex, WS2812B_ColorTypeDef color) {
    if (ledIndex < LED_COUNT) {
        ledColors[ledIndex] = color;
    }
}

// 设置所有LED颜色
void WS2812B_SetAllLEDColor(WS2812B_ColorTypeDef color) {
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        ledColors[i] = color;
    }
}

// 发送数据到WS2812B
static void WS2812B_SendData(uint8_t *data, uint32_t length) {
    // 关闭所有中断，确保时序准确
    __disable_irq();
    
    for (uint32_t i = 0; i < length; i++) {
        // 发送8位数据，高位在前
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (data[i] & (1 << (7 - bit))) {
                // 发送1码：高电平约0.8us，低电平约0.45us
                GPIO_SetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // 延时约0.8us (根据实际系统时钟调整)
                for (uint8_t n = 0; n < 32; n++) __NOP();
                GPIO_ResetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // 延时约0.45us
                for (uint8_t n = 0; n < 18; n++) __NOP();
            } else {
                // 发送0码：高电平约0.4us，低电平约0.85us
                GPIO_SetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // 延时约0.4us
                for (uint8_t n = 0; n < 16; n++) __NOP();
                GPIO_ResetBits(WS2812B_GPIO_PORT, WS2812B_GPIO_PIN);
                // 延时约0.85us
                for (uint8_t n = 0; n < 34; n++) __NOP();
            }
        }
    }
    
    // 恢复中断
    __enable_irq();
}

// 更新LED显示
void WS2812B_Update(void) {
    uint32_t index = 0;
    
    // 填充数据缓冲区
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        // WS2812B采用GRB格式
        dataBuffer[index++] = ledColors[i].green;
        dataBuffer[index++] = ledColors[i].red;
        dataBuffer[index++] = ledColors[i].blue;
    }
    
    // 发送数据
    WS2812B_SendData(dataBuffer, LED_COUNT * 3);
    
    // 发送复位信号(至少50us低电平)
    Delay_us(50);
}
