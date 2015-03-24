#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "IIC.h"

//片内硬件初始化
void ChipHardwareInit(void)
{
        Stm32_Clock_Init(9);
	GPIO_Configuration();
	USART_Configuration(USART1,115200);
	IIC_Init();
	
}
//片外硬件初始化
void MYRCC_DeInit(void)
{	
 	RCC->APB1RSTR = 0x00000000;//复位结束			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //睡眠模式闪存和SRAM时钟使能.其他关闭.	  
  	RCC->APB2ENR = 0x00000000; //外设时钟关闭.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //使能内部高速时钟HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //复位HSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //复位HSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //关闭所有中断		 
	//配置向量表				  

}

void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //复位并配置向量表
 	RCC->CR|=0x00010000;  //外部高速时钟使能HSEON
	while(!(RCC->CR>>17));//等待外部时钟就绪
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//抵消2个单位
	RCC->CFGR|=PLL<<18;   //设置PLL值 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2个延时周期

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//等待PLL锁定
	RCC->CFGR|=0x00000002;//PLL作为系统时钟	 
	while(temp!=0x02)     //等待PLL作为系统时钟设置成功
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}	

u8 Sys_Init( void )
{
	u16 Time_Out=0;
	Sys_DeInit();
	RCC->CR   |= 0x00010000;
	do
	{
		Time_Out++;
	}
	while(((RCC->CR &0x00020000)==0)&&Time_Out<=0x0500);
	if((RCC->CR &0x00020000)!=0)
	{
		FLASH->ACR |=0x32;						//预缓冲开启 启用预缓冲区 闪存访问2个等待.
		RCC->CFGR |= 0x00000800; 				//APB2不分频 APB1 二分频 AHB不分频；
		RCC->CFGR &= 0xffc0ffff;				//清零PLL倍频系数 等。
		RCC->CFGR |= (PLL_MUL-2)<<18;			//倍频系数设定 
		RCC->CFGR |= 0x00010000;				//HSE作为PLL输入时钟源
		RCC->CR   |= 0x01000000;				//使能PLL
		while((RCC->CR & 0x01000000) == 0)
		RCC->CFGR &=~0x00000003;				//清零系统时钟选择位
		RCC->CFGR |= 0x00000002;				//设置 PLL为系统时钟
		while ((RCC->CFGR & 0x0000000c)!=0x08);//等待PLL作为系统时钟；
	}
	else return 1;
	

//	SysTick->CTRL =0xffffffff;				//systick 选择外部时钟 倒数到0产生中断 使能计数器
	/**/
        SysTick->LOAD =	 9 ;
	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	SysTick->VAL =0;           				//清空计数器
	SysTick->CTRL=0x03 ;          			//开始倒数  
        return 1;
}

void Sys_DeInit( void )
{
	RCC->CR   |= 0x00000001;		//复位RCC时钟源 为默认的 内部晶振 HSI
	RCC->CFGR &= 0xF0FF0000;		//复位时钟配置寄存器 设为默认值	 SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
	RCC->CR   &= 0xFEF6FFFF;		//复位外部HSE振荡器 时钟监测器 	  HSEON, CSSON and PLLON
	RCC->CR   &= 0xFFFBFFFF;		//复位HSE就绪标志  HSEBYP
	RCC->CFGR &= 0xFF80FFFF;		//复位  PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE
	RCC->CIR   = 0x009F0000;		//清零各种中断标志 
	
	RCC->APB1ENR&=0x00000000;
	RCC->APB2ENR&=0x00000000;	
}

void GPIO_Configuration(void)
{
	RCC->APB2ENR |= 0x00000008;			 	//使能APB2上的 GPIOB
	RCC->APB2ENR |= 0x00000040;				//使能APB2上的 GPIOE

	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0x00000033;				//通用推完输出 PB 8  9
	GPIOE->CRL&=0XFFFFFF00;
	GPIOE->CRL|=0x00000033;				//通用推挽输出 PE 0  1

	GPIOB->CRH&=0XFFF0FFFF;
	GPIOB->CRH|=0x00040000;				//通用推完输出 PB 8  9
}

void NVIC_Configuration(void)
{
	
	
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);               //选择中断分组2
        NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn;             //选择串口1中断
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     //抢占式中断优先级设置为0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            //响应式中断优先级设置为0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //使能中断
        NVIC_Init(&NVIC_InitStructure);  
}

void uart_NVIC_Configuration(void)
{
	
	
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);               //选择中断分组2
        NVIC_InitStructure.NVIC_IRQChannel =USART1_IRQn;             //选择串口1中断
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     //抢占式中断优先级设置为0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            //响应式中断优先级设置为0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //使能中断
        NVIC_Init(&NVIC_InitStructure);  
}

void USART_Configuration(USART_TypeDef* USARTx, u32 baud)
{
	u16 mantissa;
	u16 fraction;
	float temp;	

	RCC->APB2ENR |= 0x00000004;			 	//使能APB2上的 GPIOA
	RCC->APB2ENR |= 0x00004000;				//使能APB2上的 USART1
	RCC->APB2ENR |= 0x00000001;				//使能APB2上的 AFIO

	GPIOA->CRH&=0XFFFFF00F;
	GPIOA->CRH|=0x000004b0;					//PA 9复用推挽输出 PA10 复用浮空输入

	RCC->APB2RSTR |= 0x00004000;			//复位USART1 复位后寄存器值为0x0000 很多设置采用默认
	RCC->APB2RSTR &=~0x00004000; 
        GPIO_InitTypeDef GPIO_InitStructure;
    
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	temp=(float)(PLL_MUL*8*1000000)/(baud*16);
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
        mantissa<<=4;
	mantissa+=fraction;
	USARTx->BRR |=mantissa;
	USARTx->CR1 |=0x200C;     
}


u8 USART_Send(u8 us)
{
	USART1->DR = (us );
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
        return 1;
} 

u8 USART_CRCSend(int *ch1, int *ch2, int *ch3, int *ch4)
{
	u8 Array[8];
	u8 Rcvbuf[2];
	u8 i;

	Array[0]=*ch1;
	Array[1]=*ch1>>8;
	Array[2]=*ch2;
	Array[3]=*ch2>>8;
	Array[4]=*ch3;
	Array[5]=*ch3>>8;
	Array[6]=*ch4;
	Array[7]=*ch4>>8;
	CRC16(Array, Rcvbuf,8);
	for(i=0; i<8; i++)USART_Send(Array[i]);
	USART_Send(Rcvbuf[1]);
	USART_Send(Rcvbuf[0]);
        return 1;
}

void CRC16(u8 *Array, u8 *Rcvbuf,u16 Len) 
{ 
 	u16  IX,IY,CRC1; 
 	CRC1=0xFFFF;//set all 1 
 	if (Len<=0)CRC1 = 0; 
 	else 
 	{ 
  		Len--; 
  		for (IX=0;IX<=Len;IX++) 
   		{ 
    	CRC1=CRC1^(unsigned int)(Array[IX]); 
    	for(IY=0;IY<=7;IY++) 
    	if ((CRC1&1)!=0) CRC1=(CRC1>>1)^0xA001; 
    	else CRC1=CRC1>>1;    
  
   		} 
	} 
	Rcvbuf[0] = (CRC1 & 0xff00)>>8;//高位置 
	Rcvbuf[1] = (CRC1 & 0x00ff);  //低位置 
}


void EXTI_Configuration(void )
{
	
	
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);		     	
}

void Timer_Configuration(void)
{
 /**/
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;   
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

TIM_DeInit(TIM2);


TIM_TimeBaseInitStructure.TIM_Period=40; // 自动重装载寄存器的值???????????
TIM_TimeBaseInitStructure.TIM_Prescaler= (36000 - 1); // 时钟预分频数 ?????


TIM_TimeBaseInitStructure.TIM_ClockDivision=0x00; // 采样分

TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
TIM_ClearFlag(TIM2, TIM_FLAG_Update); // 清除溢出中断标志

TIM2->DIER|=1<<0;   //允许更新中断	
TIM_Cmd(TIM2, ENABLE); //开启时钟	


}


volatile u32 Time;
void Delay_Us(u32 a)	//示波器显示 1us 偏差较大  10us稍有偏差 其他正常 
{
	Time=a;
	while(Time--);
}
void Delay_Ms(u16 a)	//65535/1000 =65.535 所以 Delay_Us 用u32 
{						//示波器显示1-1000MS 都准确
	Delay_Us(a*1000);
	while(Time--);
}				
				
void Delay(u32 a)
{
	while(a--);
}  




