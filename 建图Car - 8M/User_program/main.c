#include "main.h"

#include "usb_bsp.h"
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_hid_core.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;
extern HID_Machine_TypeDef HID_Machine;


int main(void)
{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		Delay_ms(5);

		SysTickConfig(); // 1ms��ʱ����ʼ��
		UART1_Init(115200);
		UART2_Init(115200);
		USART3_Init(115200);
		//	USART6_Init(115200);
		//��������ʼ��
		LED_Init();
		BEEP_Init();
		SBUS_Init();
		
		//FLASH��ʼ��
		setup_w25q64();
		//��ȡƫ��
		others_init();
	//���ר�ö�ʱ��
		TIM14_Int_Init(5000 - 1, 84 - 1);
		//�����ʼ��
		SERVO_Init();
		//������ʼ��
		KEY_Init();
		//------------�����ʼ��---------
		Pid_Init();
		MOTOR_AB_Init();
		MOTOR_CD_Init();
		//------------��������ʼ��---------
		ENCODER_A_Init();
		ENCODER_B_Init();
		ENCODER_C_Init();
		ENCODER_D_Init();

		beep_on_times(3,100);  // ��ʼ�������ʾ
		QMI8658A_Init();// �����ǳ�ʼ��
		Reduce_error();  //  ȥ����Ư
		beep_on_times(1,100);

//		//USB��ʼ��
		USBH_Init(&USB_OTG_Core_dev,
		USB_OTG_FS_CORE_ID,
		&USB_Host, &HID_cb, &USR_Callbacks);
//		//�����������ʼ��
		Task_Manage_List_Init();
	
		while(1)
		{
			//��������
			Execute_Task_List_RUN();

		}
}
