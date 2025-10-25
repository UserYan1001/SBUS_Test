///*
// * @�ļ�����:
// * @����: Q
// * @Date: 2023-02-13 14:01:12
// * @LastEditTime: 2023-03-28 09:32:51
// */
#include "app_sensor.h"

static void AI_xunji_moshi(void);
void AI_ziyou_bizhang(void);
static void AI_xunji_moshi_pro(void);
void AI_gensui_moshi(void);

uint8_t xunji_mode = 1;
uint8_t IR_X3, IR_X2, IR_X4, IR_X1;
int trackState = 0;
int T_cross = 0, flag_F, forbid_F;

/**
 * @��������: ����������豸���Ƴ�ʼ��
 * @return {*}
 */
void app_sensor_init(void)
{
    TRACK_IR4_Init();
    ultrasonic_sensor_init();
}

/**
 * @��������: ѭ�����������������ŵ�ADֵ
 * @return {*}
 */
void app_sensor_run(void)
{
    static u8 AI_mode_bak;

    // �ж���ִ�У�ֱ�ӷ���
    if (group_do_ok == 0)
        return;
	
    if (AI_mode == 0)
    {
    }
    else if (AI_mode == 1) /* ��������ѭ��ģʽ����,��ֱ���ڴ���14���ҵ� uint8_t xunji_mode = 1;
                              ����ѭ�������л�������0ʱΪ��ͨʮ��·��ѭ��������1ʱ��ǿ����·ѭ�����ɽ������ֱ��ѭ�������䱾���ͼ��
                            */
    {
        if (xunji_mode == 0)
            AI_xunji_moshi();
        else
            AI_xunji_moshi_pro();
    }
    else if (AI_mode == 2)
    {
        AI_ziyou_bizhang(); // ���ɱ���
    }
    else if (AI_mode == 3)
    {
        AI_gensui_moshi(); // ���湦��
    }
    else if (AI_mode == 10)
    {
        AI_mode = 255;
    }

    if (AI_mode_bak != AI_mode)
    {
        AI_mode_bak = AI_mode;
        group_do_ok = 1;
    }
}

/*************************************************************
�������ƣ�AI_xunji_moshi()
���ܽ��ܣ�ʵ��ѭ������
������������
����ֵ��  ��
*************************************************************/
static void AI_xunji_moshi(void)
{
    static uint32_t stop_time = 0;
    uint8_t IR_X3, IR_X2, IR_X4, IR_X1;
    static uint8_t cross_flag = 0, state = 0, cross = 0;

    IR_X3 = TRTACK_IR4_X3_READ();
    IR_X2 = TRTACK_IR4_X2_READ();
    IR_X1 = TRTACK_IR4_X1_READ();
    IR_X4 = TRTACK_IR4_X4_READ();
	//printf("%d  %d  %d  %d \r\n",IR_X1,IR_X2,IR_X3,IR_X4);

    /**********��������������state����ѭ��״̬��ʮ��·�ڵ�����״̬
     * 0����ͨѭ��ģʽ
     * 1����һ������ʮ��·��
     * 2���ڶ�������ʮ��·��
     * 3������������ʮ��·��
     * cross��state��case 0������ʮ��·�ڵڼ��εı�־λ
     * cross_flag����ʮ��·��ִ����ɺ�ı�־λ
     **************************************/
    /////////////////
    if (group_do_ok && millis() - stop_time > 100)
    {

        stop_time = millis();

        if (cross_flag == 1)
        {
            // ǰ��
            Vel.TG_IX = 200;
            Vel.TG_IY = 0;
            Vel.TG_IW = 0;
            if (IR_X1 == 1 || IR_X4 == 1)
                cross_flag = 0;
        }

        switch (state)
        {
        case 0:
            if (IR_X3 == 0 && IR_X2 == 0 && IR_X1 == 1 && IR_X4 == 1)
            {
                // ǰ��
                Vel.TG_IX = 200;
            }
            else if ((IR_X3 == 0 && IR_X2 == 1) || (IR_X2 == 1 && IR_X4 == 0 && IR_X1 == 1))
            {
                /* �ұ߳�ȥ����ת */
                Vel.TG_IW = 600;
            }
            else if ((IR_X3 == 1 && IR_X2 == 0) || (IR_X3 == 1 && IR_X4 == 1 && IR_X1 == 0))
            {
                // ��ת
                Vel.TG_IW = -600;
            }
            else if (IR_X3 == 0 && IR_X2 == 0 && IR_X1 == 0 && IR_X4 == 0 && (cross_flag == 0))
            {

                Vel.TG_IX = 0;
                Vel.TG_IY = 0;
                Vel.TG_IW = 0;
                if (cross == 0)
                    state = 1;
                else if (cross == 1)
                    state = 2;
                else if (cross == 2)
                    state = 3;
            }
            break;
        case 1:
            if (AI_mode == 1)
            {
                // sprintf(cmd_return, "$DGT:%d-%d,%d!", 9, 17, 1); // ��ӡ�ַ�����������
                // parse_cmd(cmd_return);   // ����������
            }
            state = 0;
            cross = 1;
            cross_flag = 1;
            break;
        case 2:
            if (AI_mode == 1)
            {
                // sprintf(cmd_return, "$DGT:%d-%d,%d!", 9, 17, 1); // ��ӡ�ַ�����������
                // parse_cmd(cmd_return);   // ����������
            }
            state = 0;
            cross = 2;
            cross_flag = 1;
            break;
        case 3:
            beep_on_times(1, 100);
            state = 0;
            cross = 0;
            cross_flag = 0;
            if (AI_mode == 1)
            {
                AI_mode = 255;
            }
            break;
        }
    }
}

static void AI_xunji_moshi_pro(void)
{
    IR_X3 = TRTACK_IR4_X3_READ();
    IR_X2 = TRTACK_IR4_X2_READ();
    IR_X1 = TRTACK_IR4_X1_READ();
    IR_X4 = TRTACK_IR4_X4_READ();

    //	 printf("%d  %d  %d  %d",IR_X1,IR_X2,IR_X3,IR_X4);
    //	Delay_ms(2000);


     /**********��������������trackState��ʾֱ�ǻ����ת��ʱ������״̬
     * 0����ͨѭ��ģʽ
     * 1����ת�������С�������ǶȽ�С�ȸ���·������һֱ��תֱ����ƽ��Ϊֱ�ߵ�ƽ��1001��ʱ�Żָ�����
     * 2����ת�������С�������ǶȽ�С�ȸ���·������һֱ��תֱ����ƽ��Ϊֱ�ߵ�ƽ��1001��ʱ�Żָ�����
     * flag_F��Ϊ��������S���н�ʱX4̽ͷƵ���л�״̬
     * forbid_turn���������ͼ����������1�͵�3��T��·��ʱ������ת�����ֱ��
     * �û���ʹ�ù����У���ͨ������IX\IY\IW����ֵ���ﵽ�õ�ѭ��Ч��
     * ��������ʹ��Delay_ms����������ɳ����쳣 
     **************************************/
    switch (trackState)
    {
    case 0:
        if ((IR_X1 == 1 && IR_X2 == 1 && IR_X3 == 1 && IR_X4 == 1 && flag_F != 1))
        {
            Vel.TG_IX = 0;
            Vel.TG_IY = 0;
            Vel.TG_IW = -400;
        }
        if ((IR_X1 == 1 && IR_X2 == 0 && IR_X3 == 0 && IR_X4 == 1))
        {
            // ��·����������⵽���ߣ�ǰ��
            Vel.TG_IX = 200;
            Vel.TG_IY = 0;
            Vel.TG_IW = 0;
            flag_F = 0;
            forbid_F = 0;

            if (forbid_turn == 1 || forbid_turn == 4)
            {
                forbid_turn = 2;
            }
        }
        else if (IR_X1 == 1 && IR_X2 == 0 && IR_X3 == 0 && IR_X4 == 0)
        {
            // �����
            Vel.TG_IX = 50; 
            Vel.TG_IY = 0;
            Vel.TG_IW = 400; // ��ת
            trackState = 1;
        }
        else if (IR_X1 == 0 && IR_X2 == 0 && IR_X3 == 0 && IR_X4 == 1)
        {
            if (forbid_turn == 0)
            {

                Vel.TG_IX = 200;
                Vel.TG_IY = 0;
                Vel.TG_IW = 0;
                forbid_turn = 1;
            }
            else if (forbid_turn == 3)
            {

                Vel.TG_IX = 200;
                Vel.TG_IY = 0;
                Vel.TG_IW = 0;
                forbid_turn = 4;
            }
            else if (forbid_turn != 0 && forbid_turn != 1 && forbid_turn != 4)
            {

                // �Ҵ���
                Vel.TG_IX = 50;
                Vel.TG_IY = 0;
                Vel.TG_IW = -400; 
                trackState = 2;
            }
        }
        else if (IR_X4 == 0 && forbid_F != 1)
        {
            // ���������
            Vel.TG_IX = 100; 
            Vel.TG_IY = 0;   
            Vel.TG_IW = 1000;
            flag_F = 1;
            // Delay_ms(100);
        }
        else if (IR_X1 == 0)
        {
            // ���������
            Vel.TG_IX = 100;   
            Vel.TG_IY = 0;     
            Vel.TG_IW = -1000; 
                               // Delay_ms(100);
        }
        else if (IR_X1 == 1 && IR_X2 == 1 && IR_X3 == 0 && IR_X4 == 1)
        {
            // �м�����ϵĴ�����΢������ת
            Vel.TG_IX = 200;
            Vel.TG_IY = 0;   
            Vel.TG_IW = 200; // ��ת
        }
        else if (IR_X1 == 1 && IR_X2 == 0 && IR_X3 == 1 && IR_X4 == 1)
        {
            // �м�����ϵĴ�����΢������ת
            Vel.TG_IX = 200;
            Vel.TG_IY = 0;    
            Vel.TG_IW = -200; 
        }

        break;
    case 1:
        Vel.TG_IX = 80;  
        Vel.TG_IY = 0;   
        Vel.TG_IW = 500; 
        if (IR_X1 == 1 && IR_X2 == 0 && IR_X3 == 0 && IR_X4 == 1)
        {
            trackState = 0;
            Vel.TG_IW = 500; 
            forbid_F = 1;
            Delay_ms(200);
        }
        break;
    case 2:
        Vel.TG_IX = 80;   
        Vel.TG_IY = 0;    
        Vel.TG_IW = -500; 
        if (IR_X1 == 1 && IR_X2 == 0 && IR_X3 == 0 && IR_X4 == 1)
        {
            Vel.TG_IW = -500; 
            Delay_ms(200);
            trackState = 0;
            forbid_turn = 3;
        }
    }
}

/*************************************************************
�������ƣ�AI_ziyou_bizhang()
���ܽ��ܣ�ʶ���������Ӷ��ܿ�����ǰ��
������������
����ֵ��  ��
*************************************************************/
void AI_ziyou_bizhang(void)
{
    static u32 systick_ms_bak = 0;
    static int bz_num = 0, bz_num_bak = 0;
    int adc_csb;
    if (millis() - systick_ms_bak > 50)
    {
        systick_ms_bak = millis();

        adc_csb = (int)sensor_sr_ultrasonic_read(); // ��ȡa0��adֵ�����������
        //printf("adc_csb=%d\r\n", adc_csb);
        if ((adc_csb < 35) && (adc_csb > 15))
        {
            
            bz_num = 2;
      
        }
        else if (adc_csb < 15)
        {
					
            bz_num = 1;
					
        }else  {
					
				    bz_num = 3;
				}
				
        if (bz_num != bz_num_bak)
        {
            switch (bz_num)
            {
            case 1:
                Vel.TG_IX = -300;
                Vel.TG_IY = 0;
                Vel.TG_IW = 0;
                break;
            case 2:
                Vel.TG_IX = 0;
                Vel.TG_IY = 0;
                Vel.TG_IW = 1500;

                break;
            case 3:
                Vel.TG_IX = 300;
                Vel.TG_IY = 0;
                Vel.TG_IW = 0;

                break;
            default:
                zx_uart_send_str("error\n");
                break;
            }
            bz_num_bak = bz_num;
        }
    }
}

/*************************************************************
�������ƣ�AI_gensui_moshi()
���ܽ��ܣ����������룬��һ��������ʵ�ָ��湦��
������������
����ֵ��  ��
*************************************************************/
void AI_gensui_moshi(void)
{
    static u32 systick_ms_bak = 0;
    int adc_csb;
    if (millis() - systick_ms_bak > 100)
    {
        systick_ms_bak = millis();
        adc_csb = sensor_sr_ultrasonic_read(); // ��ȡa0��adֵ�����������

        if ((adc_csb > 40))
        {
            // ����30~50cmǰ��
            Vel.TG_IX = 600;
            Vel.TG_IY = 0;
            Vel.TG_IW = 0;
        }
        else if ((adc_csb < 20) && (adc_csb > 0))
        {
            // �������20cm�ͺ���
            Vel.TG_IX = -600;
            Vel.TG_IY = 0;
            Vel.TG_IW = 0;
        }
        else
        {
            // �������ֹͣ
            Vel.TG_IX = 0;
            Vel.TG_IY = 0;
            Vel.TG_IW = 0;
        }
    }
}
