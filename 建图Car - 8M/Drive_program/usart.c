/****************************************************************************
 *	@����	��	YJH
 *	@����	��	2025-09-09
 *	@����	��	�������¿Ƽ����޹�˾
 *****************************************************************************/
#include "main.h"

/**************************���ڴ�ӡ��غ����ض���********************************/
/**
 * @��  ��  �ض���putc����
 */
 
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
}

/**
 * @��  ��  �ض���getc������USART1��
 */
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
	{
	}

	return (int)USART_ReceiveData(USART1);
}
//int fputc(int ch, FILE *f)
//{
//	/* Place your implementation of fputc here */
//	/* e.g. write a character to the USART */
//	USART_SendData(USART1, (uint8_t)ch);

//	/* Loop until the end of transmission */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
//	{
//	}

//	return ch;
//}

///**
// * @��  ��  �ض���getc������USART1��
// */
//int fgetc(FILE *f)
//{
//	/* �ȴ�����1�������� */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
//	{
//	}

//	return (int)USART_ReceiveData(USART1);
//}


//-----------------------------UART1��ͨ����-------------------------------------

/**
 * @��  ��  UART ���ڳ�ʼ��
 * @��  ��  baud�� ����������
 * @����ֵ	 ��
 */
// ����1���Զ����ص�·
void UART1_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ���� USART1���� */
	// ��GPIO��USART������ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// USART1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// USART1 �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USART1��������
	USART_InitStructure.USART_BaudRate = baud; // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	// ʹ�� USART�� �������
	USART_Cmd(USART1, ENABLE);

	// ��ܵ�һ���ַ����������BUG
	//	USART_ClearFlag(USART1, USART_FLAG_TC);

	// �������ڽ����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ��������ж�

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // ����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
}


/***********************************************
	���ܽ��ܣ�	����1�����ֽ�
	����������	dat ���͵��ֽ�
	����ֵ��		��
 ***********************************************/
void uart1_send_byte(u8 dat)
{
	USART_SendData(USART1, dat);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	���ܽ��ܣ�	����1�����ַ���
	����������	*s ���͵��ַ���
	����ֵ��		��
 ***********************************************/
void uart1_send_str(u8 *s)
{
	while (*s)
	{
		uart1_send_byte(*s++);
	}
}



/* QSA */
/**
 * @��  ��  DBUART �����жϷ�����
 * @��  ��  ��
 * @����ֵ  ��
 */
void USART1_IRQHandler(void)
{
   u8 sbuf_bak;
   static u16 buf_index = 0;
   if (USART_GetFlagStatus(USART1, USART_IT_RXNE) == SET)
   {
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART1);

//		USART_SendData(USART1, sbuf_bak);
		if (uart1_get_ok)
			return;
		if (sbuf_bak == '<')
		{
			uart1_mode = 4;
			buf_index = 0;
		}
		else if (uart1_mode == 0)
		{
			if (sbuf_bak == '$')
			{
				uart1_mode = 1;
			}
			else if (sbuf_bak == '#')
			{
				uart1_mode = 2;
			}
			else if (sbuf_bak == '{')
			{
				uart1_mode = 3;
			}
			else if (sbuf_bak == '<')
			{
				uart1_mode = 4;
			}
			else if (sbuf_bak == '[')
			{
				uart1_mode = 5;
			}
			else if (sbuf_bak == '?')
			{
				uart1_mode = 6;
			}
			else if (sbuf_bak == '(')
			{
				uart1_mode = 7;
			}
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;

		if ((uart1_mode == 4) && (sbuf_bak == '>'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 5) && (sbuf_bak == ']'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 6) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 7) && (sbuf_bak == '?'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		if (buf_index >= 1024)
			buf_index = 0;
	}
//		if (uart1_get_ok)
//		  
//		   return;
//		if (sbuf_bak == '<')
//		{
//		   uart1_mode = 4;
//		   buf_index = 0;
//		}
//		else if (uart1_mode == 0)
//		{
//			if (sbuf_bak == '$')
//			{
//				uart1_mode = 1;
//			}
//			else if (sbuf_bak == '[')
//			{
//				uart1_mode = 5;
//				buf_index = 0;
//			}
//			else if (sbuf_bak == '#')
//			{
//				uart1_mode = 2;
//				buf_index = 0;
//			}
//			else if (sbuf_bak == '?')
//			{
//				uart1_mode = 6;
//				buf_index = 0;
//			}
//		}
//		

//		uart_receive_buf[buf_index++] = sbuf_bak;
//		if ((uart1_mode == 2) && (sbuf_bak == '!'))
//		{
//		   uart_receive_buf[buf_index] = '\0';
//		   uart1_get_ok = 1;
//		}
//		else if ((uart1_mode == 1) && (sbuf_bak == '!'))
//		{
//			uart_receive_buf[buf_index] = '\0';
//			uart1_get_ok = 1;
//		}	
//	   else if ((uart1_mode == 4) && (sbuf_bak == '>'))
//		{
//		   uart_receive_buf[buf_index] = '\0';
//		   uart1_get_ok = 1;
//		}
//		else if ((uart1_mode == 5) && (sbuf_bak == ']'))
//		{
//			uart_receive_buf[buf_index] = '\0';
//			uart1_get_ok = 1;
//		}
//		else if ((uart1_mode == 6) && (sbuf_bak == '!'))
//		{
//			uart_receive_buf[buf_index] = '\0';
//			uart1_get_ok = 1;
//		}
//	    if(uart1_get_ok==1)
//	    {
//			uart_receive_num = 1;
//        }
//		
//       if (buf_index >= 2048)
//           buf_index = 0;
//   }
}


//-----------------------------UART2��ͨ����-------------------------------------

/**
 * @��  ��  UART   ���ڳ�ʼ��
 * @��  ��  baud�� ����������
 * @����ֵ	 ��
 */

// �û��ӿڣ�Ҳ��ֱ�ӽ���ݮ�ɵ�GPIO
void UART2_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	// USART �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// USART��������
	USART_InitStructure.USART_BaudRate = baud; // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// USARTʹ��
	USART_Cmd(USART2, ENABLE);

	// �������ڽ����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // ��������ж�

	// USART2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		  // ����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

/***********************************************
	���ܽ��ܣ�	����2�����ֽ�
	����������	dat ���͵��ֽ�
	����ֵ��		��
 ***********************************************/
void uart2_send_byte(u8 dat)
{
	USART_SendData(USART2, dat);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	���ܽ��ܣ�	����2�����ַ���
	����������	*s ���͵��ַ���
	����ֵ��		��
 ***********************************************/
void uart2_send_str(u8 *s)
{
	while (*s)
	{
		uart2_send_byte(*s++);
	}
}


void Send_Number(uint32_t Number, uint8_t len)  //123456  6
{
	uint8_t i=0;
	for(i=0; i<len; i++)     // i=0
	{
		uart2_send_byte(Number % 10 + '0');    //
	}

}
/**
 * @��  ��  UART �����жϷ�����
 * @��  ��  ��
 * @����ֵ  ��
 */
void USART2_IRQHandler(void)
{
	u8 sbuf_bak;
	static u16 buf_index = 0;
	//USART_SendData(USART2, sbuf_bak);
	if (USART_GetFlagStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART2);
		
		if (uart1_get_ok)
			return;
		if (sbuf_bak == '<')
		{
			uart1_mode = 4;
			buf_index = 0;
		}
		else if (uart1_mode == 0)
		{
			if (sbuf_bak == '$')
			{
				uart1_mode = 1;
			}
			else if (sbuf_bak == '#')
			{
				uart1_mode = 2;
			}
			else if (sbuf_bak == '{')
			{
				uart1_mode = 3;
			}
			else if (sbuf_bak == '<')
			{
				uart1_mode = 4;
			}
			else if (sbuf_bak == '[')
			{
				uart1_mode = 5;
			}
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;

		if ((uart1_mode == 4) && (sbuf_bak == '>'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 5) && (sbuf_bak == ']'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}

		if (buf_index >= 1024)
			buf_index = 0;
	}
}



//-----------------------------UART3���ߴ���-------------------------------------

/************************************************
�������� �� USART3_Send_ArrayU8
��    �� �� ���߶���Ĵ�������
��    �� �� pData ---- �ַ���
            Length --- ����
�� �� ֵ �� ��
*************************************************/
/* QSA */
void USART3_Init(uint32_t baud)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		/* ���� USART3���� */
		// ��GPIO��USART������ʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		// USART3��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

		// USART3 �˿�����
		// USART �˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //��©����
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		// USART3��������
		USART_InitStructure.USART_BaudRate = baud; // ������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART3, &USART_InitStructure);
		USART_HalfDuplexCmd(USART3, ENABLE);  	//ע�������������˫��ģʽ

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;         // ����3�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        // �����ȼ�
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);                           // ����ָ���Ĳ�����ʼ��VIC�Ĵ���

		// �������ڽ����ж�
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // ��������ж�
		USART_ITConfig(USART3, USART_IT_TXE, DISABLE); /* ��ֹ���ڷ����ж� */
		// ʹ�� USART�� �������
		USART_Cmd(USART3, ENABLE);
}

/***********************************************
	��������:	uart3_send_str()
	���ܽ���:	����3�����ַ���
	��������:	*s ���͵��ַ���
	����ֵ:		��
 ***********************************************/
void uart3_send_str(uint8_t *s)
{
	sendOnly(USART3);
	while (*s)
	{
		USART_SendData(USART3, *s++);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
			;
	}
	USART_SendData(USART3, 0xff);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
		;
	readOnly(USART3);
}

/*  �����жϽ��մ��� */
void USART3_IRQHandler(void)
{
    u8 sbuf_bak;
	static u16 buf_index = 0;
	if (USART_GetFlagStatus(USART3, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART3);
		printf("BUS3 Receive");
		//USART_SendData(USART1, sbuf_bak);
		if (uart1_get_ok)
			return;

		if (uart1_mode == 0)
		{
			if (sbuf_bak == '$')
			{
				uart1_mode = 1;
			}
			else if (sbuf_bak == '#')
			{
				uart1_mode = 2;
			}
			else if (sbuf_bak == '{')
			{
				uart1_mode = 3;
			}
			else if (sbuf_bak == '[')
			{
				uart1_mode = 5;
			}
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;


		if ((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 5) && (sbuf_bak == ']'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		if(uart1_get_ok==1)
		{
			uart_receive_num = 3;
		}

		if (buf_index >= 1024)
			buf_index = 0;
	}
}


//-----------------------------UART4��ͨ����-------------------------------------

/**
 * @��  ��  UART   ���ڳ�ʼ��
 * @��  ��  baud�� ����������
 * @����ֵ	 ��
 */

// �û��ӿڣ�Ҳ��ֱ�ӽ���ݮ�ɵ�GPIO
void UART4_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	// USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

	// USART �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// USART��������
	USART_InitStructure.USART_BaudRate = baud; // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	// USARTʹ��
	USART_Cmd(UART4, ENABLE);

	// �������ڽ����ж�
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE); // ��������ж�

	// UART4 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		  // ����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

/***********************************************
	���ܽ��ܣ�	����4�����ֽ�
	����������	dat ���͵��ֽ�
	����ֵ��		��
 ***********************************************/
void uart4_send_byte(u8 dat)
{
	USART_SendData(UART4, dat);
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	���ܽ��ܣ�	����4�����ַ���
	����������	*s ���͵��ַ���
	����ֵ��		��
 ***********************************************/
void uart4_send_str(u8 *s)
{
	while (*s)
	{
		uart4_send_byte(*s++);
	}
}

/**
 * @��  ��  UART �����жϷ�����
 * @��  ��  ��
 * @����ֵ  ��
 */
void UART4_IRQHandler(void)
{
	u8 sbuf_bak;
	static u16 buf_index = 0;
	if (USART_GetFlagStatus(UART4, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(UART4);
		// USART_SendData(USART1, sbuf_bak);
		if (uart1_get_ok)
			return;
		if (sbuf_bak == '<')
		{
			uart1_mode = 4;
			buf_index = 0;
		}
		else if (uart1_mode == 0)
		{
			if (sbuf_bak == '$')
			{
				uart1_mode = 1;
			}
			else if (sbuf_bak == '#')
			{
				uart1_mode = 2;
			}
			else if (sbuf_bak == '{')
			{
				uart1_mode = 3;
			}
			else if (sbuf_bak == '<')
			{
				uart1_mode = 4;
			}
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;

		if ((uart1_mode == 4) && (sbuf_bak == '>'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}

		if (buf_index >= 1024)
			buf_index = 0;
	}
}


//-----------------------------UART5-SBUS-����-------------------------------------

#include "robot.h"

uint8_t uart_sbus_rx_buf[SBUS_FRAME_LENGTH] = {0};
uint8_t uart_sbus_rx_con = 0;
uint8_t uart_sbus_rx_complete = 0;
uint16_t sbus_buf[16] = {0};

/**
  * @��  ��  SBUS���ڳ�ʼ��
  * @��  ��  ��
  * @����ֵ	 ��
  */
void SBUS_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	//USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5); 

	//USART �˿�����	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 

	//USART��������
	USART_InitStructure.USART_BaudRate = 100000;    //������
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(UART5, &USART_InitStructure);
	
    //USART3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	
	//�������ڽ����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	
		//USARTʹ��
	USART_Cmd(UART5, ENABLE); 
	
}

/**
  * @��  ��  UART �����жϷ�����
  * @��  ��  �� 
  * @����ֵ  ��
  */
void UART5_IRQHandler(void)
{
    uint8_t Res;
    uint16_t temp;
    
    // �����ж�
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  
    {
        Res = USART_ReceiveData(UART5);
        
        // SBUS֡��0x0F��ʼ�����֡ͷ
        if(uart_sbus_rx_con == 0 && Res != 0x0F)
        {
            return;  // ����֡ͷ������
        }
        
        // ���ƻ�������С����ֹ���
        if(uart_sbus_rx_con < SBUS_FRAME_LENGTH)
        {
            uart_sbus_rx_buf[uart_sbus_rx_con++] = Res;
        }
        else
        {
            uart_sbus_rx_con = 0;  // �������
        }
        
        USART_ClearITPendingBit(UART5, USART_IT_RXNE);  // ����жϱ�־
    }
    
    // ���߿����ж�
    else if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        // �����ȡ״̬�Ĵ��������ݼĴ�����������ж�
        temp = UART5->SR;
        temp = UART5->DR;
        (void)temp;  // ����δʹ�ñ�������
        
        // ����Ƿ���յ�������25�ֽ�֡
        if(uart_sbus_rx_con == SBUS_FRAME_LENGTH && 
           uart_sbus_rx_buf[0] == 0x0F && 
           uart_sbus_rx_buf[24] == 0x00)
        {
            // ������ݽ������
            uart_sbus_rx_complete = 1;
        }
        
        // ���ý��ռ�����
        uart_sbus_rx_con = 0;
        USART_ClearITPendingBit(UART5, USART_IT_IDLE);  // ����жϱ�־
    }
}




//-----------------------------UART6���ߴ���-------------------------------------

/************************************************
�������� �� USART6_Send_ArrayU8
��    �� �� ����6�����ݷ��ͺ���
��    �� �� pData ---- ��������
            Length --- ���ݳ���
�� �� ֵ �� ��
*************************************************/
/* QSA */
void USART6_Init(uint32_t baud)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		/* ���� USART6���� */
		// ��GPIO��USART������ʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  // USART6ʹ��GPIOC
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); // USART6λ��APB2����

		// USART6��Ӧ���Ÿ���ӳ�� (PC6->TX, PC7->RX)
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  // TX����
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);  // RX����

		// USART6 �˿�����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  // ͬʱ����TX��RX
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // ��©ģʽ
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		// USART6��������
		USART_InitStructure.USART_BaudRate = baud;  // �������ɲ���ָ��
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;  // 8λ����λ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;       // 1λֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;          // ����żУ��
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ��Ӳ������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  // ͬʱʹ���շ�
		USART_Init(USART6, &USART_InitStructure);
		USART_HalfDuplexCmd(USART6, ENABLE);  // ���ð�˫��ģʽ

		// �����ж����ȼ�
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;          // USART6�ж�ͨ��
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  // ��ռ���ȼ�
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;         // �����ȼ�����USART3��ͬ
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // ʹ���ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);

		// �������ڽ����ж�
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);  // ʹ�ܽ����ж�
		USART_ITConfig(USART6, USART_IT_TXE, DISABLE);  // ��ֹ�����ж�
		USART_Cmd(USART6, ENABLE);  // ʹ��USART6
}

/***********************************************
	��������:	uart6_send_str()
	���ܽ���:	����6�����ַ���
	��������:	*s ���͵��ַ���ָ��
	����ֵ:		��
 ***********************************************/
void uart6_send_str(uint8_t *s)
{
	sendOnly(USART6);  // �л�������ģʽ
	while (*s)  // ѭ������ÿ���ַ�ֱ���ַ�������
	{
		USART_SendData(USART6, *s++);  // ���͵�ǰ�ַ����ƶ�ָ��
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);  // �ȴ��������
	}
	USART_SendData(USART6, 0xff);  // ���ͽ�����־
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);  // �ȴ��������
	readOnly(USART6);  // �л��ؽ���ģʽ
}

/*  ����6�жϽ��մ����� */
void USART6_IRQHandler(void)
{
    u8 sbuf_bak;
	static u16 buf6_index = 0;  // ʹ�ö����Ļ������������������������ڳ�ͻ
	
	if (USART_GetFlagStatus(USART6, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);  // ����жϱ�־
		sbuf_bak = USART_ReceiveData(USART6);  // ��ȡ���յ�����
		printf("BUS6 Receive");
		// ����Ѿ����յ��������ݣ������������
		if (uart1_get_ok)
			return;

		// ������ʼ�ַ����ý���ģʽ
		if (uart1_mode == 0)
		{
			if (sbuf_bak == '$')
			{
				uart1_mode = 1;
			}
			else if (sbuf_bak == '#')
			{
				uart1_mode = 2;
			}
			else if (sbuf_bak == '{')
			{
				uart1_mode = 3;
			}
			buf6_index = 0;  // ���û���������
		}

		// �����յ����ݴ��뻺����
		uart_receive_buf[buf6_index++] = sbuf_bak;

		// ����Ƿ���յ�������־
		if ((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf6_index] = '\0';  // ����ַ���������
			uart1_get_ok = 1;  // ��ǽ������
		}
		else if ((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf6_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart_receive_buf[buf6_index] = '\0';
			uart1_get_ok = 1;
		}

		// ��Ǵ���6���յ�����
		if(uart1_get_ok == 1)
		{
			uart_receive_num = 6;  // ��6��ʶ�Ǵ���6������
		}

		// ��ֹ���������
		if (buf6_index >= 1024)
			buf6_index = 0;
	}
}

void analysis(char *a,double *number)
{
   int len = strlen(a), i, j, count = 0, wei[20], times = 0,dot=0;
    double num[10] = {0};
    int ctoi = 0, befctoi = 0, hasDecimal = 0,flag=0;
    for (i = 0; i < len + 1; i++)
    {
        if (a[i]>='0'&&a[i]<='9')
        {
            ctoi = 1;
        }
        else if (a[i] == '.' && !hasDecimal)
        {
            ctoi = 1;
            hasDecimal = 1;
			dot=i;
        }
		else if (a[i]=='-')
        {
        	flag=1;
		}
        else
        {
            ctoi = 0;
        }
        if (befctoi == 0 && ctoi == 1) // ������
        {
            wei[count] = a[i] - '0';
            befctoi = 1;
            count++;
        }
        else if (befctoi == 1 && ctoi == 1) // ��λ
        {
            wei[count] = a[i] - '0';
            count++;
        }
        else if (befctoi == 1 && ctoi == 0) // �½���
        {
            double decimalMultiplier = 1.0;
            double decimalNum = 0.0;
            for (j = 0; j < count; j++)
            {
                if (hasDecimal && a[i - count + j] == '.')
                {
                    decimalMultiplier = pow(10, count - j - 1);

                }
                else
                {
                    if (!hasDecimal)
                    {
                        num[times] += wei[j] * pow(10, count - j - 1);
                    }
                    else
                    {
                        if(i - count + j+1 <=dot)
                            decimalNum += wei[j] * pow(10, count - j -2);//С����ǰ
                        else  if(i - count + j+1 >dot)
                            decimalNum += wei[j] * pow(10, count - j -1);//С�����
                    }
                }
            }
            if (hasDecimal)
            {
                num[times]+= decimalNum / decimalMultiplier;
            }
			if(flag==1)num[times]*=-1;
            times++;
            befctoi = 0;
            count = 0;
            hasDecimal = 0;
			flag=0;
        }
    }

    for (i = 0; i < times; i++)number[i]=num[i];
}
