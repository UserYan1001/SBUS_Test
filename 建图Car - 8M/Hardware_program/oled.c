/****************************************************************************
 *	@����	��	
 *	@����	��	
 *	@����	��	
 *	@����	��	���OLED��صĺ���
 *	@�����б�
 *	1.	void I2C_init() -- I2C��ʼ��
 *	2.	void I2C_WriteByte(unsigned char I2C_Byte) -- ͨ��I2C����дһ��byte������
 *	3.	void OLED_Write_Dat(unsigned char I2C_Data) -- ��OLED��д����
 *	4.	void OLED_Write_Cmd(unsigned char I2C_Command) -- ��OLED��д����
 *	5.	void OLED_ON(void) -- OLED����
 *	6.	void OLED_OFF(void) -- OLED�ر�
 *	7.	void OLED_Set_Pos(unsigned char x, unsigned char y) -- ������ʾ����
 *	8.	void OLED_Fill(unsigned char fill_Data) -- ȫ����ʾ(��ʾBMPͼƬʱ�Ż��õ��˹���)
 *	9.	void OLED_CLS(void) -- ��λ/����
 *	10.	void OLED_Init(void) -- OLED����ʼ�����򣬴˺���Ӧ�ڲ�����Ļ֮ǰ���ȵ���
 *	11.	void OLED_P6x8Str(unsigned char x, y,unsigned char ch[]) -- 6x8������������ʾASCII�����С���У���̫����
 *	12.	void OLED_P8x16Str(unsigned char x, y,unsigned char ch[]) -- 8x16������������ʾASCII�룬�ǳ�����
 *	13.	void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) -- ������ʾASCII�룬�ǳ�����
 *	14.	void OLED_P16x16Ch(unsigned char x, y, N) -- 16x16������������ʾ���ֵ���С���У������ø������塢�Ӵ֡���б���»��ߵ�
 *	15.	void OLED_DrawBMP(unsigned char x0, y0,x1, y1,unsigned char BMP[]) -- ��128x64���ص�BMPλͼ��ȡ�����������ֱ�Ȼ���Ƶ�codetab�У��˺������ü���
 *	16.	void OLED_TEST(void) -- OLED���Գ���
 *****************************************************************************/

#include "oled.h"
#include "resource.h"

#define I2C_TIMEOUT_TIMES 100 // ��ʱ����

static void soft_i2c_gpio_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��
	//GPIOB8,B9��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
}

// ��ʱ ���ڵȴ�Ӧ��ʱ�ĳ�ʱ�ж� ��ֲʱ���޸�
static void i2c_timeout_delay(void)
{
    Delay_us(1);
}
static void i2c_delay() // ÿ���ļ�� ���ڵȴ���ƽ�ȶ��Ϳ���ͨѶ����
{
    unsigned char i;
    i = 2;
    while (--i)
    {
        i2c_timeout_delay();
    }
}

// SCL���� ��ֲʱ���޸�
static void I2C_SCL_H(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
}

// SCL���� ��ֲʱ���޸�
static void I2C_SCL_L(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}

// SDA���� ��ֲʱ���޸�
static void I2C_SDA_H(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_9);
}

// SDA���� ��ֲʱ���޸�
static void I2C_SDA_L(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
}

// ��ȡSDA ��ֲʱ���޸�
static uint8_t I2C_SDA_Read(void)
{
    int t;

    t = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);

    return t;
}

/*******************************************************************************
 * �� �� ��       : i2c_start
 * ��������		 : ����I2C��ʼ�ź�
 * ��    ��       : ��
 * ��    ��    	 : ��
 *******************************************************************************/
static void i2c_start(void)
{
    I2C_SDA_H();
    I2C_SCL_H();
    i2c_delay();

    I2C_SDA_L(); // ��SCLΪ�ߵ�ƽʱ��SDA�ɸ߱�Ϊ��
    i2c_delay();
    I2C_SCL_L(); // ǯסI2C���ߣ�׼�����ͻ��������
}

/*******************************************************************************
 * �� �� ��         : i2c_stop
 * ��������		   : ����I2Cֹͣ�ź�
 * ��    ��         : ��
 * ��    ��         : ��
 *******************************************************************************/
static void i2c_stop(void)
{
    I2C_SDA_L();
    I2C_SCL_H();
    i2c_delay();

    I2C_SDA_H(); // ��SCLΪ�ߵ�ƽʱ��SDA�ɵͱ�Ϊ��
    i2c_delay();
}

/*******************************************************************************
* �� �� ��         : i2c_wait_ack
* ��������		   : �ȴ�Ӧ���źŵ���
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
                     0������Ӧ��ɹ�
*******************************************************************************/
static uint8_t i2c_wait_ack(void)
{
    uint16_t time_temp = 0;

    I2C_SCL_H();
    I2C_SDA_H();
    i2c_delay();
    while (I2C_SDA_Read()) // �ȴ�SDAΪ�͵�ƽ
    {
        time_temp++;
        i2c_timeout_delay();
        if (time_temp > I2C_TIMEOUT_TIMES) // ��ʱ��ǿ�ƽ���I2Cͨ��
        {
            i2c_stop();
            return 1;
        }
    }
    I2C_SCL_L();
    return 0;
}

/*******************************************************************************
 * �� �� ��         : i2c_write_byte
 * ��������		   : I2C����һ���ֽ�
 * ��    ��         : dat������һ���ֽ�
 * ��    ��         : ��
 *******************************************************************************/
static void i2c_write_byte(uint8_t dat)
{
    uint8_t i = 0;

    I2C_SCL_L();
    for (i = 0; i < 8; i++) // ѭ��8�ν�һ���ֽڴ������ȴ����ٴ���λ
    {
        if ((dat & 0x80) > 0)
            I2C_SDA_H();
        else
            I2C_SDA_L();
        dat <<= 1;
        i2c_delay();
        I2C_SCL_H();
        i2c_delay();
        I2C_SCL_L();
        i2c_delay();
    }
}

/***********************************************
    �������ƣ�I2C_WriteByte(I2C_Byte)
    ���ܽ��ܣ�I2Cдһ���ֽ�
    ����������I2C_Byte д����ֽ�
    ����ֵ��	��
 ***********************************************/
void I2C_WriteByte(uint8_t addr, uint8_t data)
{
    i2c_start();
    i2c_write_byte(OLED_ADDRESS); // ����������ַ+д����
    if (i2c_wait_ack())          // �ȴ�Ӧ��
    {
        i2c_stop();
        return;
    }
    i2c_write_byte(addr); // д�Ĵ�����ַ
    i2c_wait_ack();      // �ȴ�Ӧ��
    i2c_write_byte(data); // ��������
    if (i2c_wait_ack())  // �ȴ�ACK
    {
        i2c_stop();
        return;
    }
    i2c_stop();
	printf("oled ok\n");
    return;
}

/***********************************************
    �������ƣ�OLED_Write_Dat(I2C_Data)
    ���ܽ��ܣ�OLEDдһ�������ֽ�
    ����������I2C_Data д��������ֽ�
    ����ֵ��	��
 ***********************************************/
void OLED_Write_Dat(unsigned char I2C_Data)
{
    I2C_WriteByte(0x40, I2C_Data);
}

/***********************************************
    �������ƣ�OLED_Write_Cmd(I2C_Command)
    ���ܽ��ܣ�OLEDдһ�������ֽ�
    ����������I2C_Command д��������ֽ�
    ����ֵ��	��
 ***********************************************/
void OLED_Write_Cmd(unsigned char I2C_Command)
{
    I2C_WriteByte(0x00, I2C_Command);
}

/***********************************************
    �������ƣ�OLED_ON()
    ���ܽ��ܣ�OLED����
    ������������
    ����ֵ��	��
 ***********************************************/
void OLED_ON(void)
{
    OLED_Write_Cmd(0X8D);
    OLED_Write_Cmd(0X14);
    OLED_Write_Cmd(0XAF);
}

/***********************************************
    �������ƣ�OLED_OFF()
    ���ܽ��ܣ�OLED�ر�
    ������������
    ����ֵ��	��
 ***********************************************/
void OLED_OFF(void)
{
    OLED_Write_Cmd(0X8D);
    OLED_Write_Cmd(0X10);
    OLED_Write_Cmd(0XAE);
}

/***********************************************
    �������ƣ�OLED_Set_Pos(x,y)
    ���ܽ��ܣ�����OLEDд������
    ����������x ������ y ������
    ����ֵ��	��
 ***********************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_Write_Cmd(0xb0 + y);
    OLED_Write_Cmd(((x & 0xf0) >> 4) | 0x10);
    OLED_Write_Cmd((x & 0x0f) | 0x01);
}

/***********************************************
    �������ƣ�OLED_Fill(fill_Data)
    ���ܽ��ܣ�OLEDȫ����ʾ
    ����������fill_Data ȫ��ָ��
    ����ֵ��	��
 ***********************************************/
void OLED_Fill(unsigned char fill_Data)
{
    unsigned char y, x;
    for (y = 0; y < 8; y++)
    {
        OLED_Write_Cmd(0xb0 + y);
        OLED_Write_Cmd(0x01);
        OLED_Write_Cmd(0x10);
        for (x = 0; x < X_WIDTH; x++)
            OLED_Write_Dat(fill_Data);
    }
}

/***********************************************
    �������ƣ�OLED_CLS()
    ���ܽ��ܣ�OLED��λ ȫ�����
    ������������
    ����ֵ��	��
 ***********************************************/
void OLED_CLS(void)
{
    OLED_Fill(0x00);
}

/***********************************************
    �������ƣ�OLED_Init()
    ���ܽ��ܣ�OLED��ʼ��
    ������������
    ����ֵ��	��
 ***********************************************/
void OLED_Init(void)
{
	soft_i2c_gpio_init();
	
    OLED_Write_Cmd(0xae);       // �ر�oled���
    OLED_Write_Cmd(0x00);       // ���õ��е�ַ
    OLED_Write_Cmd(0x10);       // ���ø��е�ַ
    OLED_Write_Cmd(0x40);       // ������ʼ�е�ַ  ����ӳ��RAM��ʾ��ʼ��(0x00~0x3F)
    OLED_Write_Cmd(0x81);       // ���öԱȶȿ��ƼĴ���
    OLED_Write_Cmd(Brightness); // ����SEG�����������
    OLED_Write_Cmd(0xa1);       // ����SEG/��ӳ��     0xa0���ҷ��� 0xa1����
    OLED_Write_Cmd(0xc8);       // ����COM/��ɨ�跽��   0xc0���·��� 0xc8����
    OLED_Write_Cmd(0xa6);       // ����������ʾ
    OLED_Write_Cmd(0xa8);       // ���ö�·���ñ�(1~64)
    OLED_Write_Cmd(0x3f);       // 1/64 duty
    OLED_Write_Cmd(0xd3);       // ������ʾƫ��λ��ӳ��RAM������(0x00~0x3F)
    OLED_Write_Cmd(0x00);       // ��ƫ��
    OLED_Write_Cmd(0xd5);       // ������ʾʱ�ӷ�Ƶ��/��Ƶ��
    OLED_Write_Cmd(0x80);       // ���÷�Ƶ�ȣ�����ʱ��Ϊ100֡/��
    OLED_Write_Cmd(0xd9);       // ����Ԥ�������
    OLED_Write_Cmd(0xf1);       // �趨Ԥ���Ϊ15��ʱ�ӣ��ŵ�Ϊ1��ʱ��
    OLED_Write_Cmd(0xda);       // ����com����Ӳ������
    OLED_Write_Cmd(0x12);
    OLED_Write_Cmd(0xdb); // ����VCOM��
    OLED_Write_Cmd(0x40); // ����VCOMȡ��ѡ���ƽ
    OLED_Write_Cmd(0x20); // ����ҳ��Ѱַģʽ(0x00/0x01/0x02)
    OLED_Write_Cmd(0x02);
    OLED_Write_Cmd(0x8d); // ���ó�������/����
    OLED_Write_Cmd(0x14); // ����(0x10)����
    OLED_Write_Cmd(0xa4); // ����������ʾ(0xa4/0xa5)
    OLED_Write_Cmd(0xa6); // ���÷�����ʾ(0xa6/0xa7)
    OLED_Write_Cmd(0xaf); // ��oled���
    OLED_Fill(0x00);      // ��ʼ����
    OLED_Set_Pos(0, 0);

    OLED_CLS();
}

/***********************************************
    �������ƣ�OLED_P6x8Str(x,y,ch[])
    ���ܽ��ܣ���ʾ6*8һ���׼ASCII�ַ���
    ����������x ��ʼ�������(0~127) y ��ʼ��������(0~7) ch[] Ҫ��ʾ���ַ���
    ����ֵ��	��
 ***********************************************/
void OLED_P6x8Str(unsigned char x, unsigned char y, unsigned char ch[])
{
    unsigned char c = 0, i = 0, j = 0;
    while (ch[j] != '\0')
    {
        c = ch[j] - 32;
        if (x > 126)
        {
            x = 0;
            y++;
        }
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_Write_Dat(F6x8[c][i]);
        x += 6;
        j++;
    }
}

/***********************************************
    �������ƣ�OLED_P8x16Str(x,y,ch[])
    ���ܽ��ܣ���ʾ8*16һ���׼ASCII�ַ���
    ����������x ��ʼ�������(0~127) y ��ʼ��������(0~7) ch[] Ҫ��ʾ���ַ���
    ����ֵ��	��
 ***********************************************/
void OLED_P8x16Str(unsigned char x, unsigned char y, unsigned char ch[])
{
    unsigned char c = 0, i = 0, j = 0;
    while (ch[j] != '\0')
    {
        c = ch[j] - 32;
        if (x > 120)
        {
            x = 0;
            y++;
        }
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_Write_Dat(F8X16[c * 16 + i]);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_Write_Dat(F8X16[c * 16 + i + 8]);
        x += 8;
        j++;
    }
}

/***********************************************
    �������ƣ�OLED_ShowStr(x,y,ch[],TextSize)
    ���ܽ��ܣ���ʾcodetab.h�е�ASCII�ַ�
    ����������x ��ʼ�������(0~127) y ��ʼ��������(0~7) ch[] Ҫ��ʾ���ַ��� TextSize �ַ���С(1:6*8 ; 2:8*16)
    ����ֵ��	��
 ***********************************************/
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
    unsigned char c = 0, i = 0, j = 0;
    switch (TextSize)
    {
    case 1:
    {
        while (ch[j] != '\0')
        {
            c = ch[j] - 32;
            if (x > 126)
            {
                x = 0;
                y++;
            }
            OLED_Set_Pos(x, y);
            for (i = 0; i < 6; i++)
                OLED_Write_Dat(F6x8[c][i]);
            x += 6;
            j++;
        }
    }
    break;
    case 2:
    {
        while (ch[j] != '\0')
        {
            c = ch[j] - 32;
            if (x > 120)
            {
                x = 0;
                y++;
            }
            OLED_Set_Pos(x, y);
            for (i = 0; i < 8; i++)
                OLED_Write_Dat(F8X16[c * 16 + i]);
            OLED_Set_Pos(x, y + 1);
            for (i = 0; i < 8; i++)
                OLED_Write_Dat(F8X16[c * 16 + i + 8]);
            x += 8;
            j++;
        }
    }
    break;
    }
}

/***********************************************
    �������ƣ�OLED_P16x16Ch(x,y,N)
    ���ܽ��ܣ���ʾ16*16����
    ����������x ��ʼ�������(0~127) y ��ʼ��������(0~7) N ��ʾ����
    ����ֵ��	��
 ***********************************************/
void OLED_P16x16Ch(unsigned char x, unsigned char y, unsigned char N)
{
    unsigned char wm = 0;
    unsigned int adder = 32 * N;
    OLED_Set_Pos(x, y);
    for (wm = 0; wm < 16; wm++)
    {
        OLED_Write_Dat(F16x16[adder]);

        adder += 1;
    }
    OLED_Set_Pos(x, y + 1);
    for (wm = 0; wm < 16; wm++)
    {
        OLED_Write_Dat(F16x16[adder]);
        adder += 1;
    }
}

/***********************************************
    ���ܽ��ܣ���ʾ��ɫ���� ��0 ��1 ��2 ɫ3
    ����������x ��ʼ�������(0~127) y ��ʼ��������(0~7) N ��ʾ����
    ����ֵ��	��
 ***********************************************/
void OLED_ShowColorFont(unsigned char x, unsigned char y, unsigned char N)
{
    unsigned char wm = 0;
    unsigned int adder = 2 * N;
    OLED_Set_Pos(x, y);
    for (wm = 0; wm < 16; wm++)
    {
        OLED_Write_Dat(FONT_COLOR[adder][wm]);

    }
	adder += 1;
    OLED_Set_Pos(x, y + 1);
    for (wm = 0; wm < 16; wm++)
    {
        OLED_Write_Dat(FONT_COLOR[adder][wm]);
        
    }
}

/***********************************************
    �������ƣ�OLED_DrawBMP(x0,y0,x1,y1,BMP[])
    ���ܽ��ܣ���ʾ��ʾBMPͼƬ128��64
    ����������x ��ʼ�������(0~127) y ��ʼ��������(0~7) BMP[] Ҫ��ʾ��ͼƬ�� len ͼƬ�Ĵ�С
    ����ֵ��	��
 ***********************************************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[], uint16_t len)
{
    unsigned int j = 0;
    unsigned char x, y;

    for (y = y0; y <= y1; y++)
    {

        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            if (len < j)
                return;
            OLED_Write_Dat(BMP[j++]);
        }
    }
}

/***********************************************
    �������ƣ�OLED_TEST()
    ���ܽ��ܣ�OLED���Գ���
    ������������
    ����ֵ��	��
 ***********************************************/
void OLED_TEST(void)
{
    OLED_DrawBMP(0, 2, 128, 7, BMP1, sizeof(BMP1));
}
