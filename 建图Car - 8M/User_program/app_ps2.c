#include "app_ps2.h"
#include "usbh_hid_gamepad.h"

/* PS2手柄接收数据
   ps2_buf[3]<<8 + ps2_buf[4] = (二进制，1释放，0按下)
   x  x  x  x  x  x  x  x   x  x  x  x  x  x  x  x
   LL LD LR LU ST AR AL SE  RL RD RR RU R1 L1 R2 L2

   L=左边 R=右边 D=下 U=上 A=遥感 1=前上 2=前下
   ST=START SE=SELECT
*/
extern u8 zhixing_flag;
const char *ps2_cmd_grn[PSX_BUTTON_NUM] = {
    // 绿灯模式下按键的配置
    "G_L2", // L2
    "G_R2", // R2
    "G_L1", // L1
    "G_R1", // R1
    "G_RU", // RU
    "G_RR", // RR
    "G_RD", // RD
    "G_RL", // RL
    "G_SE", // SE
    "G_AL", // AL
    "G_AR", // AR
    "G_ST", // ST
    "G_LU", // LU
    "G_LR", // LR
    "G_LD", // LD
    "G_LL", // LL
};

/* 初始指令，如果没有从上位机下载指令的话会执行下面的指令 */
const char *pre_cmd_set_grn[PSX_BUTTON_NUM] = {
    // 绿灯模式下按键的配置
    "<G_L2:#005P0600T2000!^#005PDST!>", // L2  左上500
    "<G_R2:#005P2400T2000!^#005PDST!>", // R2	右上500
    "<G_L1:#004P0600T2000!^#004PDST!>", // L1	左上1000
    "<G_R1:#004P2400T2000!^#004PDST!>", // R1	右上1000
    "<G_RU:#002P2400T2000!^#002PDST!>", // RU	前进1000
    "<G_RR:#003P2400T2000!^#003PDST!>", // RR	右平移1000
    "<G_RD:#002P0600T2000!^#002PDST!>", // RD	后退1000
    "<G_RL:#003P0600T2000!^#003PDST!>", // RL	左平移1000
    "<G_SE:$DJR!111>",                     // SE
    "<G_AL:$DJR!222>",                          // AL
    "<G_AR:$DJR!333>",                          // AR
    "<G_ST:$DJR!444>",                     // ST
    "<G_LU:#001P0600T2000!^#001PDST!>", // LU	前进500
    "<G_LR:#000P0600T2000!^#000PDST!>", // LR	右转500
    "<G_LD:#001P2400T2000!^#001PDST!>", // LD	后退500
    "<G_LL:#000P2400T2000!^#000PDST!>", // LL	左转500
};

static u16 ps2_cmd = 0;
static u16 ps2_cmd_last = 0;
static u16 ps2_status_flag = 0xffff;

void app_ps2(void)
{
    uint16_t pos;
    // 有动作执行或者ps2没有读取数据，直接返回
    if ((!group_do_ok) || (!ps2_do_ok))
        return;
    ps2_do_ok = 0;

    ps2_cmd_last = ps2_cmd;
    /* 判断手柄数据，更改为匹配以前代码的格式 */
    if (ps2_buf[6] & 0x01) /* L2 */
        ps2_cmd &= ~0X0001;
    else
        ps2_cmd |= 0X0001;

    if (ps2_buf[6] & 0x02) /* R2 */
        ps2_cmd &= ~0X0002;
    else
        ps2_cmd |= 0X0002;

    if ((ps2_buf[5] & 0x40)) /* L1 */
        ps2_cmd &= ~0X0004;
    else
        ps2_cmd |= 0X0004;

    if ((ps2_buf[5] & 0x80)) /* R1 */
        ps2_cmd &= ~0X0008;
    else
        ps2_cmd |= 0X0008;

    if ((ps2_buf[5] & 0x10)) /* RU */
        ps2_cmd &= ~0X0010;
    else
        ps2_cmd |= 0X0010;

    if ((ps2_buf[5] & 0x02)) /* RR */
        ps2_cmd &= ~0X0020;
    else
        ps2_cmd |= 0X0020;

    if ((ps2_buf[5] & 0x01)) /* RD */
        ps2_cmd &= ~0X0040;
    else
        ps2_cmd |= 0X0040;

    if ((ps2_buf[5] & 0x08)) /* RL */
        ps2_cmd &= ~0X0080;
    else
        ps2_cmd |= 0X0080;

    if ((ps2_buf[6] & 0x04)) /* SE */
        ps2_cmd &= ~0X0100;
    else
        ps2_cmd |= 0X0100;

    if ((ps2_buf[6] & 0x20)) /* AL */
        ps2_cmd &= ~0X0200;
    else
        ps2_cmd |= 0X0200;

    if ((ps2_buf[6] & 0x40)) /* AR */
        ps2_cmd &= ~0X0400;
    else
        ps2_cmd |= 0X0400;

    if ((ps2_buf[6] & 0x08)) /* ST */
		soft_reset();
//        ps2_cmd &= ~0X0800;
    else
        ps2_cmd |= 0X0800;

    if (ps2_buf[4] == 0x00) /* LU */
        ps2_cmd &= ~0X1000;
    else
        ps2_cmd |= 0X1000;

    if (ps2_buf[4] == 0x02) /* LR */
        ps2_cmd &= ~0X2000;
    else
        ps2_cmd |= 0X2000;

    if (ps2_buf[4] == 0x04) /* LD */
        ps2_cmd &= ~0X4000;
    else
        ps2_cmd |= 0X4000;

    if (ps2_buf[4] == 0x06) /* LL */
        ps2_cmd &= ~0X8000;
    else
        ps2_cmd |= 0X8000;
    
    if (ps2_cmd != ps2_cmd_last)
    {
        for (u8 i = 0; i < 16; i++)
        {
            if (!(ps2_cmd & (1 << i))) /* 当前手柄按下 */
            { 
                if ((ps2_status_flag & (1 << i))) /* 上一次手柄未按下 */
                {
                    memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
                    /* 执行一次按下事件 */
                    if (eeprom_info.ps2_cmd_size > 6) /* 数据大于字符串 "<PS2>" */
                    {
                        w25x_read(uart_receive_buf, W25Q64_PS2_ADDR_SAVE_STR, eeprom_info.ps2_cmd_size);

                        strcpy((char *)cmd_return, (char *)ps2_cmd_grn[i]);
                        cmd_return[4] = '0'; /* 按下为0 */
                        cmd_return[5] = '\0';

                        pos = str_contain_str(uart_receive_buf, cmd_return); /* 获取ps2按键数据位置 */
                        /* 复制 */
                        memset(cmd_return, 0, sizeof(cmd_return));
                        memcpy((char *)cmd_return, uart_receive_buf + pos, str_contain_str(uart_receive_buf + pos, "!") - 1);
                        /* 判断指令格式 */
                        if (str_contain_str(cmd_return, (u8 *)"$"))
                        {
                            parse_cmd(cmd_return);
                        }
                        else if (str_contain_str(cmd_return, (u8 *)"#"))
                        {
                            parse_action(cmd_return);
                        }
                        printf("press cmd %s= %s\r\n", ps2_cmd_grn[i], uart_receive_buf);
                    }
                    else /* 没有下载指令，执行原始指令 */
                    {
                        memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));

                        pos = str_contain_str(uart_receive_buf, (u8 *)"^");
                        if (pos)
                            uart_receive_buf[pos - 1] = '\0';

                        strcpy((char *)cmd_return, (char *)uart_receive_buf + 6);
                        /* 判断指令格式 */
                        if (str_contain_str(cmd_return, (u8 *)"$"))
                        {
                            parse_cmd(cmd_return);
                        }
                        else if (str_contain_str(cmd_return, (u8 *)"#"))
                        {
                            parse_action(cmd_return);
                        }
                    }
                }
            }
            else
            {
                if (!(ps2_status_flag & (1 << i)))
                {
                    memset(uart_receive_buf, 0, sizeof(uart_receive_buf));
                    /* 执行一次释放事件 */
                    if (eeprom_info.ps2_cmd_size > 6) /* 数据大于字符串 "<PS2>" */
                    {
                        w25x_read(uart_receive_buf, W25Q64_PS2_ADDR_SAVE_STR, eeprom_info.ps2_cmd_size);

                        strcpy((char *)cmd_return, (char *)ps2_cmd_grn[i]);
                        cmd_return[4] = '1'; /* 释放为1 */
                        cmd_return[5] = '\0';

                        pos = str_contain_str(uart_receive_buf, cmd_return); /* 获取ps2按键数据位置 */
                        /* 复制 */
                        memset(cmd_return, 0, sizeof(cmd_return));
                        memcpy((char *)cmd_return, uart_receive_buf + pos, str_contain_str(uart_receive_buf + pos, "!") - 1);
                        /* 判断指令格式 */
                        if (str_contain_str(cmd_return, (u8 *)"$"))
                        {
                            parse_cmd(cmd_return);
                        }
                        else if (str_contain_str(cmd_return, (u8 *)"#"))
                        {
                            parse_action(cmd_return);
                        }
                        // printf("release cmd %s= %s\r\n", ps2_cmd_grn[i], uart_receive_buf);
                    }
                    else /* 没有下载指令，执行原始指令 */
                    {
                        memcpy((char *)uart_receive_buf, (char *)pre_cmd_set_grn[i], strlen(pre_cmd_set_grn[i]));

                        pos = str_contain_str(uart_receive_buf, (u8 *)"^");
                        if (pos)
                        {
                            strcpy((char *)cmd_return, (char *)uart_receive_buf + pos);
                            /* 判断指令格式 */
                            if (str_contain_str(cmd_return, (u8 *)"$"))
                            {
                                parse_cmd(cmd_return);
                            }
                            else if (str_contain_str(cmd_return, (u8 *)"#"))
                            {
                                parse_action(cmd_return);
                            }
                        }
                    }
                }
            }
        }
        ps2_status_flag = ps2_cmd;
    }

    /* 推动遥感值操作
        上右值； 0-127
        左下值； 255-128
    */
		
//		extern short Tar_V;
//		
//		if(abs(128 - ps2_buf[0]) > 20)
//		{
//				Tar_V = 128 - ps2_buf[0];
//		}
//		else
//		{
//			Tar_V = 0;
//			Car_Stop();
//		}
//			
//		

		
	
		
	short left_up_car=0,right_up_car=0;
    /* 前进后退 */
	//printf("L_UD= %d   ", ps2_buf[1]);
    if (abs(128-ps2_buf[1]) > 20) /* 左边上下 */
    {                                   
        if ((ps2_buf[1] < 128) ) /* 上 */
            left_up_car = (128-ps2_buf[1]) * 8;
        else /* 下 */
            left_up_car = (128-ps2_buf[1])  * 8;
    }
	//printf("R_UD= %d   ", ps2_buf[3]);            
    if (abs(128-ps2_buf[3]) > 20) /* 右边上下 */
    {
        if (ps2_buf[3] < 128) /* 上 */
            right_up_car = (128-ps2_buf[3])  * 8;
        else /* 下 */
            right_up_car = (128-ps2_buf[3])  * 8;
    }
		
		
	Vel.TG_IX = left_up_car+right_up_car;
	
    /* 左边左右,平移 */
    //printf("L_RL= %d   ", ps2_buf[0]);
	if(abs(128-ps2_buf[0]) > 20)
    {
		if (ps2_buf[0] < 128) /* 左 */
			Vel.TG_IY = (128-ps2_buf[0])  * 8;
		else /* 左 */
			Vel.TG_IY = (128-ps2_buf[0]) * 8 ;
	}
	else
	{
		Vel.TG_IY = 0;
	}
    /* 右边左右，转弯 */
    //printf("R_RL= %d\r\n", ps2_buf[2]);
	if(abs(128-ps2_buf[2]) > 20)
    {
		zhixing_flag = 1;
		if (ps2_buf[2] < 128) /* 左 */
			Vel.TG_IW = (128-ps2_buf[2]) * 20;
			//Angle_Target -= (128-ps2_buf[2]) * 0.01;
		else /* 左 */
			Vel.TG_IW = (128-ps2_buf[2]) * 20;
			//Angle_Target -= (128-ps2_buf[2]) * 0.01;
		
//		printf("R_TW= %d   ", Vel.TG_IW);
	}	
	else
	{
		zhixing_flag = 0;
		Vel.TG_IW=0;
	}
	//printf("%d ", Vel.TG_IW);
	Vel.TG_IX=0;
	Vel.TG_IY=0;
}
