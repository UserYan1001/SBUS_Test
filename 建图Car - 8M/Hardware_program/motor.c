/****************************************************************************
 *	@笔者	：	YJH、Alien
 *	@日期	：	2025-10-09
 *	@所属	：	杭州星呗科技有限公司
 *****************************************************************************/
#include "motor.h"

extern PID_HandleTypeDef motor_A_pid;
extern PID_HandleTypeDef motor_B_pid;
extern PID_HandleTypeDef motor_C_pid;
extern PID_HandleTypeDef motor_D_pid;
extern int reset_flag_A;
extern int reset_flag_B;
extern int reset_flag_C;
extern int reset_flag_D;

tpid MotorA;
tpid MotorB;
tpid MotorC;
tpid MotorD;
tpid pidAngle;

/**
 * @简  述  PID控制函数初始化
 * @参  数  无
 * @返回值  无
 */
void Pid_Init(void)
{
//	MotorA.kp=2800;
//	MotorA.ki=3;
//	MotorA.kd=10;

//	MotorB.kp=2800;
//	MotorB.ki=3;
//	MotorB.kd=10;
//	

//	MotorC.kp=2800;
//	MotorC.ki=3;
//	MotorC.kd=10;
//	

//	MotorD.kp=2800;
//	MotorD.ki=3;
//	MotorD.kd=10;
	
	pidAngle.Actual_val=0.0;
	pidAngle.Target_val=0.0;
	pidAngle.err=0.0;
	pidAngle.last_error=0.0;
	pidAngle.sum_error=0.0;
	pidAngle.kp=-100;//1.1
	pidAngle.ki=0.0;
	pidAngle.kd=-0.0;//

	MotorA.kp=3000;
	MotorA.ki=5;
	MotorA.kd=20;

	MotorB.kp=3000;
	MotorB.ki=5;
	MotorB.kd=20;
	

	MotorC.kp=3000;
	MotorC.ki=5;
	MotorC.kd=20;
	

	MotorD.kp=3000;
	MotorD.ki=5;
	MotorD.kd=20;
}

/**
 * @简  述 积分限幅函数
 * @参  数  tpid：pid的类型
 *			low ：限幅下限
 *			high：限幅上限
 * @返回值  无
 */
void I_limit(tpid * pid, float low, float high)
{
    if(pid->sum_error<low)pid->sum_error=low;
    if(pid->sum_error>high)pid->sum_error=high;
}
/**
 * @简  述  速度PID计算函数
 * @参  数  tpid：pid的类型
 *			Actual_val：编码器速度当前值
 * @返回值  编码器的理论速度
 */
float PID_Realize(tpid *pid,float Actual_val)
{
	pid->Actual_val = Actual_val;//传递真实值
	pid->err = pid->Target_val - pid->Actual_val;//目标值减去实际值等于误差值
	pid->sum_error += pid->err;//误差累计求和
	//使用PID控制
	pid->Actual_val = pid->kp*pid->err + pid->ki*pid->sum_error + pid->kd*(pid->err - pid->last_error);
	//保存上次误差:最近一次 赋值给上次
	pid->last_error = pid->err;
	return pid->Actual_val;
}
/**
 * @简  述  角度PID计算函数
 * @参  数  tpid：pid的类型
 *			Actual_val：当前角度
 * @返回值  计算出来的理论角度
 */
float P_Angle_Realize(tpid *pid,float Actual_val)
{
	pid->Actual_val=Actual_val;//传递真实值
	pid->err=pid->Target_val - pid->Actual_val;//当前误差=目标值-真实值
	if(pid->err>=180 ){pid->err=-(360-pid->err);}
	else if( pid->err<=-180){pid->err=(pid->err+360);}
	pid->Actual_val=pid->kp*pid->err;//比例控制调节 输出=Kp*当前误差
	return pid->Actual_val;
}
/**
 * @简  述  角度PID调用函数
 * @参  数  angle:当前角度 ,范围（±260）
 *          gyro: 当前角加速度
 *          target: 目标角度
 * @返回值  旋转速度
 */
float angle_pid(float angle,float gyro,float target)
{
	float output;
	pidAngle.Target_val=target;
	output=P_Angle_Realize(&pidAngle,angle)+pidAngle.kd*gyro;
	return output;
}

/**
 * @简  述  电机PID控制函数
 * @参  数  target:编码器速度目标值 ,范围（±250）
 *          encoder: 编码器速度当前值
 * @返回值  电机PWM速度
 */
int16_t motorA_pid(float encoder,float target)
{
	float output;
	
	if(reset_flag_A == 1)
	{
		MotorA.err = 0;
		MotorA.last_error = 0;
		MotorA.Prev_Error = 0;
		MotorA.sum_error = 0;
//		pError=dError=iError=0;
		reset_flag_A = 0;
	}
	MotorA.Target_val=target;
	I_limit(&MotorA,-1000,1000);
	output=PID_Realize(&MotorA,encoder);
	if (output > 3500)
		output = 3500;
	if (output < -3500)
		output = -3500;
	return output;
}

/**
 * @简  述  电机PID控制函数
 * @参  数  target:编码器速度目标值 ,范围（±250）
 *          encoder: 编码器速度当前值
 * @返回值  电机PWM速度
 */
int16_t motorB_pid(float encoder,float target)
{
	float output;
	
	if(reset_flag_B == 1)
	{
		MotorB.err = 0;
		MotorB.last_error = 0;
		MotorB.Prev_Error = 0;
		MotorB.sum_error = 0;
//		pError=dError=iError=0;
		reset_flag_B = 0;
	}
	MotorB.Target_val=target;
	I_limit(&MotorB,-1000,1000);
	output=PID_Realize(&MotorB,encoder);
	if (output > 3500)
		output = 3500;
	if (output < -3500)
		output = -3500;
	return output;
}
/**
 * @简  述  电机PID控制函数
 * @参  数  target:编码器速度目标值 ,范围（±250）
 *          encoder: 编码器速度当前值
 * @返回值  电机PWM速度
 */
int16_t motorC_pid(float encoder,float target)
{
	float output;
	
	if(reset_flag_C == 1)
	{
		MotorC.err = 0;
		MotorC.last_error = 0;
		MotorC.Prev_Error = 0;
		MotorC.sum_error = 0;
//		pError=dError=iError=0;
		reset_flag_C = 0;
	}
	MotorC.Target_val=target;
	I_limit(&MotorC,-1000,1000);
	output=PID_Realize(&MotorC,encoder);
	if (output > 3500)
		output = 3500;
	if (output < -3500)
		output = -3500;
	return output;
}
/**
 * @简  述  电机PID控制函数
 * @参  数  target:编码器速度目标值 ,范围（±250）
 *          encoder: 编码器速度当前值
 * @返回值  电机PWM速度
 */
int16_t motorD_pid(float encoder,float target)
{
	float output;
	
	if(reset_flag_D == 1)
	{
		MotorD.err = 0;
		MotorD.last_error = 0;
		MotorD.Prev_Error = 0;
		MotorD.sum_error = 0;
//		pError=dError=iError=0;
		reset_flag_D = 0;
	}
	MotorD.Target_val=target;
	I_limit(&MotorD,-1000,1000);
	output=PID_Realize(&MotorD,encoder);
	if (output > 3500)
		output = 3500;
	if (output < -3500)
		output = -3500;
	return output;
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_D_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM1, 4200);
		TIM_SetCompare2(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM1, 4200);
		TIM_SetCompare1(TIM1, (4200 + temp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_B_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare3(TIM1, 4200);
		TIM_SetCompare4(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare4(TIM1, 4200);
		TIM_SetCompare3(TIM1, (4200 + temp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_C_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM9, 4200);
		TIM_SetCompare2(TIM9, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM9, 4200);
		TIM_SetCompare1(TIM9, (4200 + temp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_A_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM12, 4200);
		TIM_SetCompare2(TIM12, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM12, 4200);
		TIM_SetCompare1(TIM12, (4200 + temp));
	}
}

void motor_doing_set(u8 index, int vel, int time)
{
	if(index==0){Vel.TG_IX=vel*0.64;}
	else if(index==1){Vel.TG_IX=-vel*0.64;}
	else if(index==2){Vel.TG_IY=vel*0.64;}
	else if(index==3){Vel.TG_IY=-vel*0.64;}
	else if(index==4){Vel.TG_IX=vel*0.64;Vel.TG_IY=vel*0.64;}
	else if(index==5){Vel.TG_IX=-vel*0.64;Vel.TG_IY=vel*0.64;}
	else if(index==6){Vel.TG_IX=vel*0.64;Vel.TG_IY=-vel*0.64;}
	else if(index==7){Vel.TG_IX=-vel*0.64;Vel.TG_IY=-vel*0.64;}
	else if(index==8){Vel.TG_IW=-vel*1.92;}
	else if(index==9){Vel.TG_IW=-vel*1.92;}
}









