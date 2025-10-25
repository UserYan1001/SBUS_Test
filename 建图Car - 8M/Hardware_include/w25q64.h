#ifndef __W25Q64_H__
#define __W25Q64_H__

#include "main.h"
#include "stm32f4xx.h"


/*******SPI FlashƬѡ���Ŷ���*******/
// ����SPI Flash��Ƭѡ���ţ�GPIOE����15����xΪBitAction���ͣ�ENABLE/DISABLE��
#define	SPI_FLASH_CS(x)			GPIO_WriteBit(GPIOE, GPIO_Pin_15, (BitAction)x)

/*******W25Qϵ��FlashоƬID����*******/
#define W25Q80 	0XEF13 	// W25Q80оƬID������8Mbit��
#define W25Q16 	0XEF14 	// W25Q16оƬID������16Mbit��
#define W25Q32 	0XEF15 	// W25Q32оƬID������32Mbit��
#define W25Q64 	0XEF16 	// W25Q64оƬID������64Mbit��������Ĭ��оƬ��

/*******W25Q64оƬ�������ö���*******/
#define W25Q64_SECTOR_SIZE	4096		// ����������С��4KB��
#define W25Q64_SECTOR_NUM		2048		// ����������64Mbit = 8MB��8MB/4KB = 2048��������

/*******W25Q64оƬ��ַ�洢����*******/
#define FLASH_ASC16_ADDRESS                 0               // ASC16��ģ���ݴ洢��ʼ��ַ
#define FLASH_HZK16_ADDRESS                 0x1000          // HZK16���ֿ����ݴ洢��ʼ��ַ��4KBƫ�ƣ�

#define FLASH_SYSTEM_CONFIG_ADDRESS         0x43000         // ϵͳ���ò����洢��ʼ��ַ��276KBƫ�ƣ�

// 5��λͼ��Bitmap���Ĵ�С��Ϣ�洢��ַ��ÿ�����160KB��
#define FLASH_BITMAP1_SIZE_ADDRESS	        0x50000         // λͼ1��С��Ϣ��ַ��320KBƫ�ƣ�
#define FLASH_BITMAP2_SIZE_ADDRESS	        FLASH_BITMAP1_SIZE_ADDRESS + 0x28000  // λͼ2��С��Ϣ��ַ��+160KB��
#define FLASH_BITMAP3_SIZE_ADDRESS	        FLASH_BITMAP2_SIZE_ADDRESS + 0x28000  // λͼ3��С��Ϣ��ַ��+160KB��
#define FLASH_BITMAP4_SIZE_ADDRESS	        FLASH_BITMAP3_SIZE_ADDRESS + 0x28000  // λͼ4��С��Ϣ��ַ��+160KB��
#define FLASH_BITMAP5_SIZE_ADDRESS	        FLASH_BITMAP4_SIZE_ADDRESS + 0x28000  // λͼ5��С��Ϣ��ַ��+160KB��	
#define FLASH_BITMAP6_SIZE_ADDRESS	        FLASH_BITMAP5_SIZE_ADDRESS + 0x28000  // λͼ6��С��Ϣ��ַ��+160KB��

// ����λͼ��С��Ϣ�洢��ַ��ÿ�����160KB��
#define FLASH_BITMAPMAIN_SIZE_ADDRESS       FLASH_BITMAP6_SIZE_ADDRESS + 0x28000  // ��λͼ��С��Ϣ��ַ��+160KB��
#define FLASH_BITMAPDS1302_SIZE_ADDRESS     FLASH_BITMAPMAIN_SIZE_ADDRESS + 0x28000  // DS1302���λͼ��С��ַ��+160KB��
#define FLASH_BITMAPDS18B20_SIZE_ADDRESS    FLASH_BITMAPDS1302_SIZE_ADDRESS + 0x28000  // DS18B20���λͼ��С��ַ��+160KB��
#define FLASH_BITMAPBLUETOOTH_SIZE_ADDRESS  FLASH_BITMAPDS18B20_SIZE_ADDRESS + 0x28000  // �������λͼ��С��ַ��+160KB��

/*******FLASH��ؿ���ָ��壨��ѭW25Q64�����ֲᣩ*******/
#define W25X_WriteEnable			0x06 	// дʹ��ָ��������д/����������
#define W25X_WriteDisable			0x04 	// д��ָֹ���ֹд/�����������������ݣ�
#define W25X_ReadStatusReg		0x05 	// ��״̬�Ĵ���ָ���ȡоƬæ״̬��д����״̬�ȣ�
#define W25X_WriteStatusReg		0x01 	// д״̬�Ĵ���ָ�����д�������鱣���ȣ�
#define W25X_ReadData					0x03 	// ��ͨ������ָ����٣���ַ��ֱ�Ӷ����ݣ�
#define W25X_FastReadData			0x0B 	// ���ٶ�����ָ����٣���ַ���1�������ڣ�
#define W25X_FastReadDual			0x3B 	// ˫������ٶ�ָ�SPI˫·����������ٶȣ�
#define W25X_PageProgram			0x02 	// ҳ���ָ��������д256�ֽڣ����Ȳ���������
#define W25X_SectorErase			0x20 	// ��������ָ���������4KB����������дʹ�ܣ�
#define W25X_BlockErase				0xD8 	// �����ָ���������64KB�飬����дʹ�ܣ�
#define W25X_ChipErase				0xC7 	// ��Ƭ����ָ���������оƬ����ʱ�ϳ�������дʹ�ܣ�
#define W25X_PowerDown				0xB9 	// ����ģʽָ����͹��ģ����ϵ�ָ��ɻ��ѣ�
#define W25X_ReleasePowerDown	0xAB 	// �ͷŵ���ģʽָ�����оƬ���ָ�����������
#define W25X_DeviceID					0xAB 	// ���豸IDָ���ȡоƬΨһ��ʶ��
#define W25X_ManufactDeviceID	0x90 	// ������+�豸IDָ���ȡ���̴�����豸���룩
#define W25X_JedecDeviceID		0x9F 	// ��JEDEC��׼IDָ���ȡ����+�豸+������Ϣ�����ã�

/*******FLASH��غ�������*******/
void	SpiFlashInit(void);                          // SPI Flash��ʼ��������SPI���š�ʱ�ӡ�ģʽ��
void    spi_set_speed(uint16_t SpeedSet);            // ����SPIͨ�����ʣ�����������������ٶȣ�
u8		SpiWriteRead(u8 d);                          // SPI���ֽڶ�д������1�ֽ����ݣ�ͬʱ����1�ֽ����ݣ�
u16		SpiFlashReadID(void);                        // ��ȡFlashоƬID���ж�оƬ�ͺ��Ƿ�ΪW25Q64��
u8		SpiFlashReadSR(void);                        // ��ȡFlash״̬�Ĵ�������ȡæ״̬��д��������Ϣ��
void 	SpiFlashWriteSR(u8 sr);                      // д��Flash״̬�Ĵ���������д��������ȣ�
void 	SpiFlashWriteEnable(void);                   // ʹ��Flashд������ִ��д/����ǰ������ã�
void 	SpiFlashWriteDisable(void);                  // ��ֹFlashд�������������ݣ���ֹ��д��
char 	SpiFlashReadChar(u32 ReadAddr);              // ��ָ����ַ��ȡ1���ֽ�����
void 	SpiFlashRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);  // ������ȡ���ݣ���ָ����ַ��N�ֽڵ���������
void 	SpiFlashWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);  // ����д�����ݣ�������������飬��ȫд��
void 	SpiFlashWriteS(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);  // ��ǿ������д�루�����߼��迴ʵ�֣�ͨ��������У�飩
void    SpiFlashWriteChar(char tmp, u32 WriteAddr);  // ���ֽ�д�루��ָ����ַд1���ֽڣ����Ȳ���������
void 	SpiFlashWritePage(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);  // ҳд�루�������256�ֽڣ���ȷ����ַ��ͬһҳ��
void 	SpiFlashWriteSector(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);  // ����д�루�Ȳ���ָ����������д�����ݣ�
void 	SpiFlashWriteNoCheck(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);  // �޼��д�루��������������ȷ����ַ�Ѳ���������д��
void 	SpiFlashEraseSector(u32 Dst_Addr);           // ��������������ָ����ַ���ڵ�4KB����������дʹ�ܣ�
void 	SpiFlashEraseChip(void);                     // ��Ƭ��������������оƬ�������ݣ���ʱ�ϳ��������ʹ�ã�
void 	SpiFlashWaitBusy(void);                      // �ȴ�оƬ���У�ѭ����ȡ״̬�Ĵ�����ֱ��оƬ��æ��
void 	SpiFlashPowerDown(void);                     // �������ģʽ�����͹��ģ�������ָ��ɻָ���
void	SpiFlashWakeUp(void);                        // ����оƬ���ӵ���ģʽ�ָ������Ѻ���ȴ��ȶ���
void    setup_w25q64(void);                          // W25Q64�洢оƬ��ʼ�����߽׳�ʼ�������ܺ�IDУ�顢�������ã�

/*******�����궨��*******/
#define PRE_CMD_SIZE 256  // Ԥ������С��128�ֽڣ�������;����ҵ���߼���

/*******Flash�����򻯺궨�壨��װ���ĺ��������ڵ��ã�*******/
#define w25x_init()          SpiFlashInit()                  // Flash��ʼ���򻯺�
#define w25x_readId()        SpiFlashReadID()                // ��Flash ID�򻯺�
#define w25x_read(buf, addr, len) SpiFlashRead(buf, addr, len)  // �������򻯺�
#define w25x_write(buf, addr, len) SpiFlashWriteNoCheck(buf, addr, len)  // �޼��д�򻯺�
#define w25x_writeS(buf, addr, len) SpiFlashWriteS(buf, addr, len)  // ��ǿ��д�򻯺�
#define w25x_erase_sector(addr) SpiFlashEraseSector(addr)    // ���������򻯺�
#define w25x_wait_busy()     SpiFlashWaitBusy()              // �ȴ����м򻯺�

#endif
