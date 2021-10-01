#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
 
 
/************************************************
 ALIENTEKս��STM32������ʵ��4
 ����ʵ�� 
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


 int main(void)
 {		
// 	u16 t; 
//	u16 t2;	
	u16 t3;
//	u16 len;	
//	u16 len2;
	u16 len3;
	u16 times=0;
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
 	LED_Init();			     //LED�˿ڳ�ʼ��
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�
 	while(1)
	{
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			printf("\r\nUSART1 Send to USART2:\r\n\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART_SendData(USART3, USART_RX_BUF[t]);//�򴮿�1��������
//				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//				}
////			printf("\r\n\r\n");//���뻻��
//			USART_RX_STA=0;
//		}
//		if(USART_RX_STA2&0x8000)
//		{					   
//			len2=USART_RX_STA2&0x3fff;//�õ��˴ν��յ������ݳ���
//			printf("\r\nUSART2 Sen to USART1:\r\n\r\n");
//			for(t2=0;t2<len2;t2++)
//			{
//				USART_SendData(USART3, USART_RX_BUF2[t2]);//�򴮿�1��������
//				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//			} 
////			printf("\r\n\r\n");//���뻻��
//			USART_RX_STA2=0;
//		}
		if(USART_RX_STA3&0x8000)
		{					   
			len3=USART_RX_STA3&0x3fff;//�õ��˴ν��յ������ݳ���
			printf("\r\n�����͵���ϢΪ:\r\n\r\n");
			for(t3=0;t3<len3;t3++)
			{
				USART_SendData(USART3, USART_RX_BUF3[t3]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			for(t3=0;t3<len3;t3++)
			{
				USART_SendData(USART1, USART_RX_BUF3[t3]);//�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			printf("\r\n\r\n");//���뻻��
			USART_RX_STA3=0;
		}
		
		{
			times++;
			if(times%500==0)
			{
				printf("\r\nQuick\r\n\r\n");
			}
			
			delay_ms(10);   
		}

			
	}	 
 }

