#include "usbh_hid_gamepad.h"

uint8_t ps2_buf[12];
uint8_t ps2_do_ok = 0;
static void GAMEPAD_Init(void);
static void GAMEPAD_Decode(u8 *data);

HID_cb_TypeDef HID_GAMEPAD_cb =
	{
		GAMEPAD_Init,
		GAMEPAD_Decode,
};

// game pad ��ʼ��
static void GAMEPAD_Init(void)
{
	USR_GAMEPAD_Init();
}

// game pad���ݽ���
static void GAMEPAD_Decode(uint8_t *data)
{
	u8 i;

	if (data[0] == 0X07) /* ��1��2���ֽڿ�ʼ */
	{
		for (i = 0; i < 12; i++)
		{
			//printf("%02x ", data[i+1]);
			ps2_buf[i] = data[i + 1];
		}
		ps2_do_ok = 1;
		//printf("\r\n");
	}
}
