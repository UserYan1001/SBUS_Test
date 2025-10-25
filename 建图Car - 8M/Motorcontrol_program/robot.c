#include "main.h"

// �������ٶ�����
ROBOT_Velocity Vel;

// ��������������
ROBOT_Wheel Wheel_A, Wheel_B, Wheel_C, Wheel_D;

// ������RGB����
ROBOT_Light Light;

// �����˵�ص�ѹ����
uint16_t Bat_Vol;

// IMU����
int16_t imu_acc_data[3];
int16_t imu_gyro_data[3];
int16_t imu_gyro_offset[3];

// ���PID���Ʋ���
float motor_kp = 15;
float motor_ki = 0.05;
float motor_kd = 8;

float motorB_kp = 80;
float motorB_ki = 1;
float motorB_kd = 20;

float motorC_kp = 80;
float motorC_ki = 1;
float motorC_kd = 20;

float motorD_kp = 80;
float motorD_ki = 1;
float motorD_kd = 20;


//int16_t motor_kp = 8;
//int16_t motor_kd = 4;


// ������������ר�ã�ת������
ROBOT_Steering RobotStr;

// ������������ר�ã�ת������
int16_t servo_offset = 0;

blance_samp blance_sampPara = blance_samp_DEFAULTS;

/**
 * @��  ��  �����˹�������
 * @��  ��  ��
 * @����ֵ  ��
 */
void Robot_Task(void)
{
	// �������˶�ѧ����
	ROBOT_Kinematics();
}

/**
 * @��  ��  ������������
 * @��  ��  ��
 * @����ֵ  ��
 */
void Bat_Task()
{
	// ��������
	static uint16_t bat_vol_cnt = 0;

	while (1)
	{
		// �ɼ���ص�ѹ
		Bat_Vol = BatteryVoltage_GetVol_X100();

		// ���������ص�ѹ����
		// printf("@ %d  \r\n",R_Bat_Vol);

		// ��������40%
		if (Bat_Vol < VBAT_40P)
		{
			// ��������20%
			if (Bat_Vol < VBAT_20P)
			{

				// ��������10%���ر�ϵͳ���뱣��״̬
				if (Bat_Vol < VBAT_10P) // 990
				{
					// ��ѹʱ�����
					bat_vol_cnt++;

					// ����10�Σ�����ر�״̬
					if (bat_vol_cnt > 10)
					{
						// ����ٶ�����Ϊ0
						MOTOR_A_SetSpeed(0);
						MOTOR_B_SetSpeed(0);
						MOTOR_C_SetSpeed(0);
						MOTOR_D_SetSpeed(0);

						// ���������б���
						while (1)
						{
							BEEP_On();
							BEEP_Off();
						}
					}
				}
				else
				{
					bat_vol_cnt = 0;
				}
			}
		}
		else
		{
		}
	}
}

/**
 * @��  ��  ������������
 * @��  ��  ��
 * @����ֵ  ��
 */
void Key_Task()
{
	while (1)
	{
		// ����ɨ��
		

	}
}

