#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "IIC.h"

//Ƭ��Ӳ����ʼ��
void ChipHardwareInit(void)
{
        Stm32_Clock_Init(9);
	GPIO_Configuration();
	USART_Configuration(USART1,115200);
	IIC_Init();
	
}
//Ƭ��Ӳ����ʼ��
void MYRCC_DeInit(void)
{	
 	RCC->APB1RSTR = 0x00000000;//��λ����			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //˯��ģʽ�����SRAMʱ��ʹ��.�����ر�.	  
  	RCC->APB2ENR = 0x00000000; //����ʱ�ӹر�.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //ʹ���ڲ�����ʱ��HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //��λHSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //��λHSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //�ر������ж�		 
	//����������				  

}

void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //��λ������������
 	RCC->CR|=0x00010000;  //�ⲿ����ʱ��ʹ��HSEON
	while(!(RCC->CR>>17));//�ȴ��ⲿʱ�Ӿ���
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//����2����λ
	RCC->CFGR|=PLL<<18;   //����PLLֵ 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2����ʱ����

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//�ȴ�PLL����
	RCC->CFGR|=0x00000002;//PLL��Ϊϵͳʱ��	 
	while(temp!=0x02)     //�ȴ�PLL��Ϊϵͳʱ�����óɹ�
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
		FLASH->ACR |=0x32;						//Ԥ���忪�� ����Ԥ������ �������2���ȴ�.
		RCC->CFGR |= 0x00000800; 				//APB2����Ƶ APB1 ����Ƶ AHB����Ƶ��
		RCC->CFGR &= 0xffc0ffff;				//����PLL��Ƶϵ�� �ȡ�
		RCC->CFGR |= (PLL_MUL-2)<<18;			//��Ƶϵ���趨 
		RCC->CFGR |= 0x00010000;				//HSE��ΪPLL����ʱ��Դ
		RCC->CR   |= 0x01000000;				//ʹ��PLL
		while((RCC->CR & 0x01000000) == 0)
		RCC->CFGR &=~0x00000003;				//����ϵͳʱ��ѡ��λ
		RCC->CFGR |= 0x00000002;				//���� PLLΪϵͳʱ��
		while ((RCC->CFGR & 0x0000000c)!=0x08);//�ȴ�PLL��Ϊϵͳʱ�ӣ�
	}
	else return 1;
	

//	SysTick->CTRL =0xffffffff;				//systick ѡ���ⲿʱ�� ������0�����ж� ʹ�ܼ�����
	/**/
        SysTick->LOAD =	 9 ;
	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	SysTick->VAL =0;           				//��ռ�����
	SysTick->CTRL=0x03 ;          			//��ʼ����  
        return 1;
}

void Sys_DeInit( void )
{
	RCC->CR   |= 0x00000001;		//��λRCCʱ��Դ ΪĬ�ϵ� �ڲ����� HSI
	RCC->CFGR &= 0xF0FF0000;		//��λʱ�����üĴ��� ��ΪĬ��ֵ	 SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
	RCC->CR   &= 0xFEF6FFFF;		//��λ�ⲿHSE���� ʱ�Ӽ���� 	  HSEON, CSSON and PLLON
	RCC->CR   &= 0xFFFBFFFF;		//��λHSE������־  HSEBYP
	RCC->CFGR &= 0xFF80FFFF;		//��λ  PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE
	RCC->CIR   = 0x009F0000;		//��������жϱ�־ 
	
	RCC->APB1ENR&=0x00000000;
	RCC->APB2ENR&=0x00000000;	
}

void GPIO_Configuration(void)
{
	RCC->APB2ENR |= 0x00000008;			 	//ʹ��APB2�ϵ� GPIOB
	RCC->APB2ENR |= 0x00000040;				//ʹ��APB2�ϵ� GPIOE

	GPIOB->CRH&=0XFFFFFF00;
	GPIOB->CRH|=0x00000033;				//ͨ��������� PB 8  9
	GPIOE->CRL&=0XFFFFFF00;
	GPIOE->CRL|=0x00000033;				//ͨ��������� PE 0  1

	GPIOB->CRH&=0XFFF0FFFF;
	GPIOB->CRH|=0x00040000;				//ͨ��������� PB 8  9
}

void NVIC_Configuration(void)
{
	
	
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);               //ѡ���жϷ���2
        NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn;             //ѡ�񴮿�1�ж�
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     //��ռʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            //��Ӧʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //ʹ���ж�
        NVIC_Init(&NVIC_InitStructure);  
}

void uart_NVIC_Configuration(void)
{
	
	
        NVIC_InitTypeDef NVIC_InitStructure;

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);               //ѡ���жϷ���2
        NVIC_InitStructure.NVIC_IRQChannel =USART1_IRQn;             //ѡ�񴮿�1�ж�
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     //��ռʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            //��Ӧʽ�ж����ȼ�����Ϊ0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               //ʹ���ж�
        NVIC_Init(&NVIC_InitStructure);  
}

void USART_Configuration(USART_TypeDef* USARTx, u32 baud)
{
	u16 mantissa;
	u16 fraction;
	float temp;	

	RCC->APB2ENR |= 0x00000004;			 	//ʹ��APB2�ϵ� GPIOA
	RCC->APB2ENR |= 0x00004000;				//ʹ��APB2�ϵ� USART1
	RCC->APB2ENR |= 0x00000001;				//ʹ��APB2�ϵ� AFIO

	GPIOA->CRH&=0XFFFFF00F;
	GPIOA->CRH|=0x000004b0;					//PA 9����������� PA10 ���ø�������

	RCC->APB2RSTR |= 0x00004000;			//��λUSART1 ��λ��Ĵ���ֵΪ0x0000 �ܶ����ò���Ĭ��
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
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
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
	Rcvbuf[0] = (CRC1 & 0xff00)>>8;//��λ�� 
	Rcvbuf[1] = (CRC1 & 0x00ff);  //��λ�� 
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


TIM_TimeBaseInitStructure.TIM_Period=40; // �Զ���װ�ؼĴ�����ֵ???????????
TIM_TimeBaseInitStructure.TIM_Prescaler= (36000 - 1); // ʱ��Ԥ��Ƶ�� ?????


TIM_TimeBaseInitStructure.TIM_ClockDivision=0x00; // ������

TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
TIM_ClearFlag(TIM2, TIM_FLAG_Update); // �������жϱ�־

TIM2->DIER|=1<<0;   //��������ж�	
TIM_Cmd(TIM2, ENABLE); //����ʱ��	


}


volatile u32 Time;
void Delay_Us(u32 a)	//ʾ������ʾ 1us ƫ��ϴ�  10us����ƫ�� �������� 
{
	Time=a;
	while(Time--);
}
void Delay_Ms(u16 a)	//65535/1000 =65.535 ���� Delay_Us ��u32 
{						//ʾ������ʾ1-1000MS ��׼ȷ
	Delay_Us(a*1000);
	while(Time--);
}				
				
void Delay(u32 a)
{
	while(a--);
}  




