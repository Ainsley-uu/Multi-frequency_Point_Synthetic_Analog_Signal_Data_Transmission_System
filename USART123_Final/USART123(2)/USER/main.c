#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
 
 
/************************************************
 ALIENTEK战舰STM32开发板实验4
 串口实验 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
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
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
 	LED_Init();			     //LED端口初始化
	KEY_Init();          //初始化与按键连接的硬件接口
 	while(1)
	{
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//			printf("\r\nUSART1 Send to USART2:\r\n\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART_SendData(USART3, USART_RX_BUF[t]);//向串口1发送数据
//				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
//				}
////			printf("\r\n\r\n");//插入换行
//			USART_RX_STA=0;
//		}
//		if(USART_RX_STA2&0x8000)
//		{					   
//			len2=USART_RX_STA2&0x3fff;//得到此次接收到的数据长度
//			printf("\r\nUSART2 Sen to USART1:\r\n\r\n");
//			for(t2=0;t2<len2;t2++)
//			{
//				USART_SendData(USART3, USART_RX_BUF2[t2]);//向串口1发送数据
//				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
//			} 
////			printf("\r\n\r\n");//插入换行
//			USART_RX_STA2=0;
//		}
		if(USART_RX_STA3&0x8000)
		{					   
			len3=USART_RX_STA3&0x3fff;//得到此次接收到的数据长度
			printf("\r\n您发送的消息为:\r\n\r\n");
			for(t3=0;t3<len3;t3++)
			{
				USART_SendData(USART3, USART_RX_BUF3[t3]);//向串口1发送数据
				while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
			}
			for(t3=0;t3<len3;t3++)
			{
				USART_SendData(USART1, USART_RX_BUF3[t3]);//向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
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

