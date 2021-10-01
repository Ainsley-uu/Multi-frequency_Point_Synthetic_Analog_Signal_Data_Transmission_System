#include "system.h"
#include "SysTick.h"
#include "led.h"
#include "usart.h"
#include "key.h"
#include "dma.h"
#include "time.h"
#include "stm32f10x_dac.h"
#include "misc.h"
#include "stdio.h"
#include "math.h"
#include "adc.h"
#define send_buf_len 8
#define PI 3.1415926
#define POINT_NUM 128
float da;
int time=0;
int bufb[8][4];
int flagadc=1;
u8 send_buf[send_buf_len];
u8 buf8[8];
u16 vol[8][POINT_NUM];
float res[8][POINT_NUM];
u8 sendvalue4[8];
u32 sendvalue32;
u8 flag = 1;

const uint16_t Sine12bit[POINT_NUM] = {
	2048	, 2460	, 2856	, 3218	, 3532	, 3786	, 3969	, 4072	,
	4093	, 4031	, 3887	, 3668	, 3382	, 3042	,	2661	, 2255	, 
	1841	, 1435	, 1054	, 714		, 428		, 209		, 65		, 3			,
	24		, 127		, 310		, 564		, 878		, 1240	, 1636	, 2048
};

uint16_t Sin4[POINT_NUM]={512,537,562,587,612,636,661,684,708,731,753,775,796,
	817,837,856,874,891,908,923,938,951,964,975,985,994,1002,1009,1014,1018,1022,
	1023,1024,1023,1022,1018,1014,1009,1002,994,985,975,964,951,938,923,908,891,
	874,856,837,817,796,775,753,731,708,684,661,636,612,587,562,537,512,487,462,
	437,412,388,363,340,316,293,271,249,228,207,187,168,150,133,116,101,86,73,60,
	49,39,30,22,15,10,6,2,1,0,1,2,6,10,15,22,30,39,49,60,73,86,101,116,133,150,168,
187,207,228,249,271,293,316,340,363,388,412,437,462,487};

uint16_t Sin8[POINT_NUM]={512,562,612,661,708,753,796,837,874,908,938,964,985,
1002,1014,1022,1024,1022,1014,1002,985,964,938,908,874,837,796,753,708,661,612,
562,512,462,412,363,316,271,228,187,150,116,86,60,39,22,10,2,0,2,10,22,39,60,86,
116,150,187,228,271,316,363,412,462,512,562,612,661,708,753,796,837,874,908,938,
964,985,1002,1014,1022,1024,1022,1014,1002,985,964,938,908,874,837,796,753,708,
661,612,562,512,462,412,363,316,271,228,187,150,116,86,60,39,22,10,2,0,2,10,22,
39,60,86,116,150,187,228,271,316,363,412,462};

uint16_t Sin12[POINT_NUM]={512,587,661,731,796,856,908,951,985,1009,1022,1023,
	1014,994,964,923,874,817,753,684,612,537,462,388,316,249,187,133,86,49,22,6,
	0,6,22,49,86,133,187,249,316,388,462,537,612,684,753,817,874,923,964,994,1014,
	1023,1022,1009,985,951,908,856,796,731,661,587,512,437,363,293,228,168,116,73,
	39,15,2,1,10,30,60,101,150,207,271,340,412,487,562,636,708,775,837,891,938,975,
	1002,1018,1024,1018,1002,975,938,891,837,775,708,636,562,487,412,340,271,207,
	150,101,60,30,10,1,2,15,39,73,116,168,228,293,363,437};

uint16_t Sin16[POINT_NUM]={512,612,708,796,874,938,985,1014,1024,1014,985,938,874,
796,708,612,512,412,316,228,150,86,39,10,0,10,39,86,150,228,316,412,512,612,708,
796,874,938,985,1014,1024,1014,985,938,874,796,708,612,512,412,316,228,150,86,39,
	10,0,10,39,86,150,228,316,412,512,612,708,796,874,938,985,1014,1024,1014,985,938,
	874,796,708,612,512,412,316,228,150,86,39,10,0,10,39,86,150,228,316,412,512,612,
	708,796,874,938,985,1014,1024,1014,985,938,874,796,708,612,512,412,316,228,150,
	86,39,10,0,10,39,86,150,228,316,412};

void U32ToU4Array(uint8_t *buf, uint32_t u32Value)
{
    buf[0] = ((u32Value >> 28) & 0x0F);
    buf[1] = ((u32Value >> 24) & 0x0F);
    buf[2] = ((u32Value >> 20) & 0x0F);
		buf[3] = ((u32Value >> 16) & 0x0F);
    buf[4] = ((u32Value >> 12) & 0x0F);
    buf[5] = ((u32Value >> 8) & 0x0F);
		buf[6] = ((u32Value >> 4) & 0x0F);
    buf[7] = (u32Value & 0x0F);
}
void U4ToBitArray(int *buf, uint8_t u32Value)
{
    buf[0] = ((u32Value >> 3) & 0x01);
    buf[1] = ((u32Value >> 2) & 0x01);
    buf[2] = ((u32Value >> 1) & 0x01);
	buf[3] = ((u32Value ) & 0x01);
}

uint8_t BitToU4Array(uint8_t *bufb)
{
	return bufb[0]<<3|bufb[1]<<2|bufb[2]<<1|bufb[3];
}

uint32_t U4ToU32Array(uint8_t *value4)
{
	return value4[0]<<28|value4[1]<<24|value4[2]<<20|value4[3]<<16|value4[4]<<12|value4[5]<<8|value4[6]<<4|value4[7];
}

void addvol(u32 data)
{
	int j,i;
	U32ToU4Array(buf8,data);

	for(j=0;j<8;j++)
	{
		U4ToBitArray(bufb[j],buf8[j]);
		for(i=0;i<POINT_NUM;i++)
			vol[j][i] = bufb[j][0]*Sin4[i] + bufb[j][1]*Sin8[i] + bufb[j][2]*Sin12[i] + bufb[j][3]*Sin16[i];
	}

}
void typeConvert(u8 sendvalue4[], u8 len)
{
	u8 i;
	for( i=0 ; i<len ; i++)
	{
		if( sendvalue4[i] < 10 && sendvalue4[i] >= 0 )
			sendvalue4[i] = sendvalue4[i]+'0';
		else
			sendvalue4[i] = sendvalue4[i] - 10 + 'A';
	}
}

void SendFunction(u8 p[])
{
	DMAx_Init(DMA1_Channel4,(u32)&USART1->DR,(u32)p,send_buf_len);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ ܴ   1  DMA         
	DMAx_Enable(DMA1_Channel4,send_buf_len);     //  ʼһ  DMA   䣡
		while(1)
			{
				if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=0)// ж ͨ  4       
				{
					DMA_ClearFlag(DMA1_FLAG_TC4);
					break;
				}
				led2=!led2;
				delay_ms(300);	
			}
}

void RCCINIT(void)		
{
    SystemInit();//72m
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
}

void GPIOINIT(void)			
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	//LED
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;//RX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

void CONTROL_INIT(void)			
{
  GPIO_InitTypeDef GPIO_InitStructure;//定义结构体变量
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;  //选择你要设置的IO口
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	 //设置推挽输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	  //设置传输速率
	GPIO_Init(GPIOA,&GPIO_InitStructure); 	   /* 初始化GPIO */
	
}

void control_key(int value4[])
{
	int i;
	for(i=2;i<6;i++)
	{
		if(value4[i-2]==0) PAout(i)=0;//该频率对应控制信号为0，控制信号低电平截止
		else PAout(i)=1;
	}
}
void NVICINIT(void)			
{
   	NVIC_InitTypeDef NVIC_InitStructure; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

void USARTINIT(void)			
{
    USART_InitTypeDef  USART_InitStructure;

	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;

	USART_Init(USART1,&USART_InitStructure);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
}

void DACINIT(void)			 
{
	
	GPIO_InitTypeDef GPIO_InitStructure;	
	DAC_InitTypeDef DAC_InitStructure;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;//TX
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
	
	DAC_InitStructure.DAC_Trigger=DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration=DAC_WaveGeneration_None;
		DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;
	
	DAC_InitStructure.DAC_OutputBuffer=DAC_OutputBuffer_Disable;

	DAC_Init(DAC_Channel_1,&DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1,ENABLE);

	DAC_SetChannel1Data(DAC_Align_12b_R,0);
}

void Send_Data(u8 *p)
{
	u16 i;
	for(i=0;i<send_buf_len;i++)
	{
		*p='5';
		p++;
	}
}
void kfft(float pr[],float pi[],int n,int k,float fr[],float fi[])
{ 
	int it,m,is,i,j,nv,l0;
    float p,q,s,vr,vi,poddr,poddi;
    for (it=0; it<=n-1; it++)  //将pr[0]和pi[0]循环赋值给fr[]和fi[]
    { 
		m=it; 
		is=0;
		for(i=0; i<=k-1; i++)
        { 
			j=m/2; 
			is=2*is+(m-2*j); 
			m=j;
		}
        fr[it]=pr[is]; 
        fi[it]=pi[is];
    }
    pr[0]=1.0; 
    pi[0]=0.0;
    p=6.283185306/(1.0*n);
    pr[1]=cos(p); //将w=e^-j2pi/n用欧拉公式表示
    pi[1]=-sin(p);

    for (i=2; i<=n-1; i++)  //计算pr[]
    { 
		p=pr[i-1]*pr[1]; 
		q=pi[i-1]*pi[1];
		s=(pr[i-1]+pi[i-1])*(pr[1]+pi[1]);
		pr[i]=p-q; pi[i]=s-p-q;
    }
    for (it=0; it<=n-2; it=it+2)  
    { 
		vr=fr[it]; 
		vi=fi[it];
		fr[it]=vr+fr[it+1]; 
		fi[it]=vi+fi[it+1];
		fr[it+1]=vr-fr[it+1]; 
		fi[it+1]=vi-fi[it+1];
    }
	m=n/2; 
	nv=2;
    for (l0=k-2; l0>=0; l0--) //蝴蝶操作
    { 
		m=m/2; 
		nv=2*nv;
        for (it=0; it<=(m-1)*nv; it=it+nv)
          for (j=0; j<=(nv/2)-1; j++)
            { 
				p=pr[m*j]*fr[it+j+nv/2];
				q=pi[m*j]*fi[it+j+nv/2];
				s=pr[m*j]+pi[m*j];
				s=s*(fr[it+j+nv/2]+fi[it+j+nv/2]);
				poddr=p-q; 
				poddi=s-p-q;
				fr[it+j+nv/2]=fr[it+j]-poddr;
				fi[it+j+nv/2]=fi[it+j]-poddi;
				fr[it+j]=fr[it+j]+poddr;
				fi[it+j]=fi[it+j]+poddi;
            }
    }
    for (i=0; i<=n-1; i++)
       { 
		  pr[i]=sqrt(fr[i]*fr[i]+fi[i]*fi[i]);  //幅值计算
       }
			 
//		for( i = 0 ; i < POINT_NUM ; i++ )
//        printf("%d %f\r\n",i,pr[i]); //输出结果
    return;
  }


int main()
{
	u8 i=0,j,k;
	u8 key;
	u32 data;
	int a[4]={1,0,0,0};
	u8 sendvaluebit[8][4];
  float pr[POINT_NUM],pi[POINT_NUM],fr[POINT_NUM],fi[POINT_NUM],t[POINT_NUM];
	SysTick_Init(72);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	//LED_Init();
	USART1_Init(9600);
	CONTROL_INIT();
	//KEY_Init();
	//DACINIT();
	ADCx_Init();
	TIM4_Init(7200-1,20000-1);  
	//TIM4_Init(7200-1,10000-1);  //定时1s
	TIM4_Init(36-1,2-1);//定时2us?
	DMAx_Init(DMA1_Channel4,(u32)&USART1->DR,(u32)send_buf,send_buf_len);
	Send_Data(send_buf);
	
	while(1)
	{
		if(flag==1&&flagadc==1)
		{
//			data=0x1A2524EA;
//			addvol(data);
//			flag=0;
		}
		
		control_key(a);
		
		if(flagadc==0)
		{
			
			for(j=0;j<POINT_NUM;j++)
				printf(" %d",vol[0][j]);
			printf("\r\n");
			
			USART_SendData(USART1, 12);//向串口1发送数据
      while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);  
			
//			for (i=0; i<POINT_NUM; i++)  //生成输入信号
//				{ 
//					//t[i] = 1.0 * i/(4000*128);
//					pr[i]= 1.0 * vol[0][i];
//					pi[i]=0.0;
//				}
//			kfft(pr,pi,POINT_NUM,7,fr,fi);  //调用FFT函数
			
			for( j=0 ; j<8 ; j++ )
			{
				for (i=0; i<POINT_NUM; i++)  //生成输入信号
				{
					pr[i]= 1.0 * vol[j][i];
					pi[i]=0.0;
				}
				kfft(pr,pi,POINT_NUM,7,fr,fi);  //调用FFT函数
				for( i=0 ; i<POINT_NUM; i++)  
					res[j][i] = pr[i];
			
				for (i=0; i<POINT_NUM&&pr[i]>1; i++)
					printf("%d_%d_%f\r\n",j,i,pr[i]); //输出结果
				
				
				for(i=0;i<4;i++)
					if (i==0) 
						{ if (res[j][3]+res[j][4]>30000) 
								sendvaluebit[j][i]=1;
						}
					else if (i==1) 
						{ if (res[j][6]+res[j][7]>30000) 
								sendvaluebit[j][i]=1;
						}
					else if (i==2) 
						{ if (res[j][9]+res[j][10]+res[j][11]>30000) 
								sendvaluebit[j][i]=1;
						}
					else if (i==3) 
						{ if (res[j][13]+res[j][14]>30000) 
								sendvaluebit[j][i]=1;
						}
			}
		
		
//		for( j=0 ; j<8 ; j++ )
//		{
//      for (i=0; i<POINT_NUM; i++)  //生成输入信号
//      {
//        pr[i]= 1.0 * vol[j][i];
//        pi[i]=0.0;
//      }
//      kfft(pr,pi,POINT_NUM,7,fr,fi);  //调用FFT函数
//			for( i=0 ; i<POINT_NUM; i++)  
//				res[j][i] = pr[i];
//			for( i = 0 ; i < POINT_NUM &&pr[i]>1; i++ )
//        printf("%d %d %f\r\n",j,i,res[j][i]); //输出结果
			
			
				for( j=0 ; j<8 ; j++ )
				{
					sendvalue4[j]=BitToU4Array(sendvaluebit[j]);
					//printf(" _%d \r\n",sendvalue4[j]);
				}
			sendvalue32=U4ToU32Array(sendvalue4);
			
				typeConvert(sendvalue4, 8);
				SendFunction(sendvalue4);
				printf("!!!!\r\n");
				
				//printf("0x%x\r\n",sendvalue32);
				//printf("\r\n");
			}
		
		
	}
}


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update))
	{ 
		if(flagadc==1)
		{
			if(time<POINT_NUM) 
			{
				vol[0][time]=(int)Get_ADC_Value(ADC_Channel_1,1);
				//printf("%d ",vol[0][time]);
				time++;
				
			}
		}
		if(time==128) {time=0;flagadc=0;}
	}

	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	
}