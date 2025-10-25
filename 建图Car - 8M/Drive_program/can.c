
#include "main.h"

/**
  * @��  ��  CAN��ʼ����
  * @��  ��  tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq - CAN_SJW_4tq 
  *          tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq - CAN_BS2_8tq;
  *          tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq  - CAN_BS1_16tq
  *          brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
  *          mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
  * @˵  ��  ������ = Fpclk1/((tbs1+1+tbs2+1+1)*brp);
  *          Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
  *          ������Ϊ:42M/((6+7+1)*6)=500Kbps
  * @����ֵ  ��
  */
void CAN1_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)
{

		GPIO_InitTypeDef GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;

		//ʹ�����ʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��PORTʱ��	                   											 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

		//��ʼ��GPIO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		//���Ÿ���ӳ������
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); 

		//CAN��Ԫ����
		CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
		CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
		CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
		CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
		CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
		CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
		CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
		CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
		CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
		CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
		CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
		CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 

		//���ù�����
		CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
		CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��	
} 

/**
  * @��  ��  CAN����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
  * @��  ��  msg:����ָ��,���Ϊ8���ֽ�
  *          num:���ݳ���(���Ϊ8)	
  * @����ֵ  0,�ɹ�������,ʧ�ܣ�
  */
uint8_t CAN_SendMsg(uint8_t* msg, uint8_t num)
{	
  uint8_t mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
	
  //���ݷ�װ
  TxMessage.StdId=0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=num;	  // ������֡��Ϣ
  for(i=0; i<num; i++)
	TxMessage.Data[i]=msg[i]; 
	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
	
  //�ȴ����ͽ���
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
	  i++;	
  
  //����״̬
  if(i>=0XFFF)
	  return 1;
  return 0;		

}

/**
  * @��  ��  CAN�������ݲ�ѯ
  * @��  ��  msg:�������ݻ�����	
  * @����ֵ  0,�����ݱ��յ���  ����,���յ����ݳ��ȣ�
  */
uint8_t CAN_RecvMsg(uint8_t *msg)
{		   		   
 	uint32_t i;
	CanRxMsg RxMessage;
	
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
		return 0;		//û�н��յ�����,ֱ���˳�
	
	//��ȡ����	
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
    for(i=0; i<RxMessage.DLC; i++)
	    msg[i]=RxMessage.Data[i];  
	
	return RxMessage.DLC;	
}

/******************* (C) ��Ȩ 2022 XTARK **************************************/
