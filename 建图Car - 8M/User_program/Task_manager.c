// ############################################################
//  FILE:  Task_manager.c
//  Created on: 2021年8月14日
//  Author: lee
//  summary: Task_manager
// ############################################################

#include "main.h"
#include "stdio.h"
#include "usbh_usr.h"

#define Task_Num 6

#define HFPeriod_COUNT 1	// 5ms
#define FaulPeriod_COUNT 10 // 10ms
#define Robot_COUNT 5		// 10ms
#define KEY_COUNT 80		// 10ms
#define LEDPeriod_COUNT 600 // 500ms
#define filter_N 12
#define IMU_COUNT 8	

extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
extern __ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;

int time_cnt,carmove_time,carmove_flag;
float Angle_Target;
float gz_filtered;

int tick_10ms;

TaskTime TasksPare[Task_Num];
float dir_anglecmd = 135.0;

void Timer_Task_Count(void) // 1ms的中断计数
{
	u16 Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++) // TASK_NUM=5
	{
		if ((TasksPare[Task_Count].Task_Count < TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Count++; // 计数 事件任务计数
		}
	}
}

void Execute_Task_List_RUN(void)
{
		
	uint16_t Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++)
	{
		if ((TasksPare[Task_Count].Task_Count >= TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Function(); // 运行计数的时间任务函数
			TasksPare[Task_Count].Task_Count = 0;
		}
	}
}

double Serial_Data[100];
//5ms进入一次（传感器应用（已注释）、USB获取数据、手柄数据处理、串口指令解析）
void HFPeriod_1msTask(void) // 备用
{
	//app_sensor_run();
	
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);//USB获取数据
	app_ps2();//手柄数据处理
	ProcessSBUSData();//
	if (uart1_get_ok) /* 接收指令 */
	{
		if (uart1_mode == 4)
		{
			// 存储模式
			save_action(uart_receive_buf);
		}
		else if (uart1_mode == 1)
		{
			uart3_send_str((u8*)"cmdOk");
			// 命令模式
			parse_cmd(uart_receive_buf);
			uart1_mode = 0;
			uart1_get_ok = 0;
		}
		else if (uart1_mode == 5)//接收树莓派发来的运动指令
		{
			// PID调试模式
			analysis(uart_receive_buf,Serial_Data);
			Vel.TG_IX=(int)Serial_Data[0]*128;
			Vel.TG_IY=(int)Serial_Data[1]*128;
			if(Serial_Data[2]==0){zhixing_flag=0;}
			else
			{
				zhixing_flag=1;
				Angle_Target -= Serial_Data[2] * 0.1;
//				Vel.TG_IW=(int)Serial_Data[2]*128;
			}
			
		}
		else if (uart1_mode == 2)
		{
			// 单个舵机调试
			parse_action(uart_receive_buf);

		}
		else if (uart1_mode == 6)
		{
			//软件复位
			soft_reset();
		}
		else if (uart1_mode == 7)//里程计和陀螺仪数据清零
		{
			odom_x=0;
			odom_y=0;
			z_angle=0;
			odom_x=0;
			odom_y=0;
		}
		uart1_mode = 0;
		uart1_get_ok = 0;
	}
	//小车运行指定时间
	if(carmove_flag==1)
	{
		time_cnt++;//printf("time_cnt=%d\r\n",time_cnt);
		if(time_cnt==carmove_time)
		{
			Vel.TG_IX=Vel.TG_IY=Vel.TG_IW=0;
			time_cnt=0;
			carmove_time=0;
			carmove_flag=0;
		}
	}
	
}


//IMU获取数据
QMI8658A_Data qdata;
void IMU_Get_Data()
{
	static uint32_t previous_millis = 0;
	if(QMI8658A_GetAllData(&qdata) != 0)
		return ;
	// 对陀螺仪数据进行卡尔曼滤波
	float gx_filtered = kalman_update(&kf_x, qdata.gx);
	float gy_filtered = kalman_update(&kf_y, qdata.gy);
	gz_filtered = kalman_update(&kf_z, qdata.gz);
	
	//解析角度只需要用Z轴角速度

	Update_Angle_Z(gz_filtered - gyro_offset_gz);//角速度转化为角度
	//printf("(%.2f,%.2f,%.1f)\r\n",odom_x,odom_y,z_angle);
	uint32_t current_millis = millis();
	if (current_millis - previous_millis >= 100)
	{
			previous_millis = current_millis;  // 更新上一次打印时间
	}

	
}



void task_send_Rece(void) 
{
	//动作组任务
	loop_action();
}

void Robot_Control(void) // 机器人控制任务，含电量管理
{
	Robot_Task();
}

void KEY_RUN(void) // 外部按键处理任务
{
	
	//软件复位，按键二
	if(KEY_Scan() == 2)
	{
		while(KEY_Scan() == 2){}
		soft_reset();
	}
}

void Task_LED(void) // 最小系统500ms的LED的闪烁
{
	GPIO_ToggleBits(GPIOD, GPIO_Pin_14); //  500ms的LED的闪烁
	 
}

void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period = HFPeriod_COUNT; // PERIOD_COUNT=5    5ms
	TasksPare[0].Task_Count = 1;			   // Task_Count的初值不一样，避免500ms到时，所有任务都执行一遍。
	TasksPare[0].Task_Function = HFPeriod_1msTask;

		
	
	TasksPare[1].Task_Period = FaulPeriod_COUNT; // 10ms
	TasksPare[1].Task_Count = 8;
	TasksPare[1].Task_Function = task_send_Rece; //

	TasksPare[2].Task_Period = Robot_COUNT; // 20ms
	TasksPare[2].Task_Count = 15;
	TasksPare[2].Task_Function = Robot_Control; //

	TasksPare[3].Task_Period = KEY_COUNT; // 100ms
	TasksPare[3].Task_Count = 80;
	TasksPare[3].Task_Function = KEY_RUN; //

	TasksPare[4].Task_Period = LEDPeriod_COUNT; // 500ms
	TasksPare[4].Task_Count = 300;
	TasksPare[4].Task_Function = Task_LED; // 500ms的LED的闪烁
	
	TasksPare[5].Task_Period = IMU_COUNT; // PERIOD_COUNT=5    5ms
	TasksPare[5].Task_Count = 1;			   // Task_Count的初值不一样，避免500ms到时，所有任务都执行一遍。
	TasksPare[5].Task_Function = IMU_Get_Data;
}
// USER CODE END
