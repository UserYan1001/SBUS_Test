#ifndef __OLED_H_
#define __OLED_H_

#include "main.h"

/*******OLED��������궨��*******/
#define OLED_ADDRESS 0x78 // ͨ������0R����,������0x78��0x7A������ַ -- Ĭ��0x78

#define Brightness 0xCF
#define X_WIDTH 128
#define Y_WIDTH 64

/*******OLED��غ�������*******/
void OLED_Write_Dat(unsigned char I2C_Data);
void OLED_Write_Cmd(unsigned char I2C_Command);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);

void OLED_Init(void);
void OLED_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[]);
void OLED_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[]);
void OLED_P16x16Ch(unsigned char x, unsigned char y, unsigned char N);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[], uint16_t len);
void OLED_TEST(void);
void OLED_ShowColorFont(unsigned char x, unsigned char y, unsigned char N);




#endif
