/****************************************************************************
 *	@����	��	YJH
 *	@����	��	2025-09-09
 *	@����	��	�������¿Ƽ����޹�˾
 *****************************************************************************/
#include "stm32f4xx.h"
#include "sys.h"
#include "myiic.h"
//// ���ŷ�������
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PC3����ģʽ
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PC3���ģʽ

//// IO��������	 
//#define IIC_SCL    PBout(8) //SCL - PC5
//#define IIC_SDA    PBout(9) //SDA - PC3
//#define READ_SDA   PBin(9)  //����SDA - PC3

//// ��ʼ��IIC
//void IIC_Init(void)
//{			
//  GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // ʹ��GPIOCʱ��

//  // GPIOC5(SCL), GPIOC3(SDA)��ʼ������
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         // ��ͨ���ģʽ
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        // ��©���
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // ����
//  GPIO_Init(GPIOB, &GPIO_InitStructure);                // ��ʼ��GPIOC
//  // ��ʼ��IIC���ߵ�ƽ
//  IIC_SCL = 1;
//  IIC_SDA = 1;
//}

//// ���ŷ�������
#define SDA_IN()  {GPIOC->MODER&=~(3<<(3*2));GPIOC->MODER|=0<<3*2;}	//PC3����ģʽ
#define SDA_OUT() {GPIOC->MODER&=~(3<<(3*2));GPIOC->MODER|=1<<3*2;} //PC3���ģʽ

// IO��������	 
#define IIC_SCL    PCout(5) //SCL - PC5
#define IIC_SDA    PCout(3) //SDA - PC3
#define READ_SDA   PCin(3)  //����SDA - PC3

// ��ʼ��IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ʹ��GPIOCʱ��

  // GPIOC5(SCL), GPIOC3(SDA)��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         // ��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;        // ��©���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    // 100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          // ����
  GPIO_Init(GPIOC, &GPIO_InitStructure);                // ��ʼ��GPIOC
  // ��ʼ��IIC���ߵ�ƽ
  IIC_SCL = 1;
  IIC_SDA = 1;
}

// ����IIC��ʼ�ź�
void IIC_Start(void)
{
  SDA_OUT();     // sda�����
  IIC_SDA = 1;	  
  IIC_SCL = 1;
  Delay_us(5);
  IIC_SDA = 0;   // START: ��CLKΪ��ʱ��DATA�Ӹ߱��
  Delay_us(5);
  IIC_SCL = 0;   // ǯסI2C���ߣ�׼�����ͻ�������� 
}	  

// ����IICֹͣ�ź�
void IIC_Stop(void)
{
  SDA_OUT();     // sda�����
  IIC_SCL = 0;
  IIC_SDA = 0;   // STOP: ��CLKΪ��ʱ��DATA�ӵͱ��
  Delay_us(5);
  IIC_SCL = 1; 
  IIC_SDA = 1;   // ����I2C���߽����ź�
  Delay_us(5);							   	
}

// �ȴ�Ӧ���źŵ���
// ����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
  u8 ucErrTime = 0;
  SDA_IN();      // SDA����Ϊ����  
  IIC_SDA = 1;
  Delay_us(5);	   
  IIC_SCL = 1;
  Delay_us(5);	 
  while(READ_SDA)
  {
    ucErrTime++;
    if(ucErrTime > 250)
    {
      IIC_Stop();
      return 1;
    }
  }
  IIC_SCL = 0;   // ʱ�����0 	   
  return 0;  
} 

// ����ACKӦ��
void IIC_Ack(void)
{
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 0;
  Delay_us(5);
  IIC_SCL = 1;
  Delay_us(5);
  IIC_SCL = 0;
}

// ������ACKӦ��		    
void IIC_NAck(void)
{
  IIC_SCL = 0;
  SDA_OUT();
  IIC_SDA = 1;
  Delay_us(5);
  IIC_SCL = 1;
  Delay_us(5);
  IIC_SCL = 0;
}					 				     

// IIC����һ���ֽ�
void IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
  SDA_OUT(); 	    
  IIC_SCL = 0;   // ����ʱ�ӿ�ʼ���ݴ���
  for(t = 0; t < 8; t++)
  {              
    IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1; 	  
    Delay_us(5);
    IIC_SCL = 1;
    Delay_us(5); 
    IIC_SCL = 0;	
  }	 
} 	    

// ��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
  unsigned char i, receive = 0;
  SDA_IN();      // SDA����Ϊ����
  for(i = 0; i < 8; i++)
  {
    IIC_SCL = 0; 
    Delay_us(5);
    IIC_SCL = 1;
    receive <<= 1;
    if(READ_SDA) receive++;   
    Delay_us(5); 
  }					 
  if (!ack)
    IIC_NAck();  // ����nACK
  else
    IIC_Ack();   // ����ACK   
  return receive;
}

