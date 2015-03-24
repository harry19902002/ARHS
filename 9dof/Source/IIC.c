#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "IIC.h"
//1�������ÿ��д��1�ֽں�Ҫ��ʱ1ms������ȷ���� ACK�ź�  ����ʱҲ��д��ȥ ���ǽ���ACK�źų���

//sbit SDA=P0^0;              /*ģ��I2C���ݴ���λ*/
//sbit SCL=P0^2;              /*ģ��I2Cʱ�ӿ���λ*/

//bit ack;            /*Ӧ���־λ*/
#define IIC_SCL_SET		 GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define IIC_SCL_RESET	 GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define IIC_SDA_SET		 GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define IIC_SDA_RESET	 GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define IIC_SDA_GET		 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)
#define IIC_PORT		 GPIOB
#define IIC_SDA			 GPIO_Pin_7
#define IIC_SCL			 GPIO_Pin_6

void IIC_SDA_Out(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(IIC_PORT, &GPIO_InitStructure);
}
void IIC_SDA_In(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = IIC_SDA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init(IIC_PORT, &GPIO_InitStructure);
}

void IIC_Start(void)								   //IIC��ʼ����
{
	
        IIC_SDA_Out();
	IIC_SDA_SET; 
	IIC_SCL_SET;
	Delay_Us(4);
	IIC_SDA_RESET;
	Delay_Us(4);
	IIC_SCL_RESET;			   
}

void IIC_Stop(void)									   //IIC��������
{
	IIC_SDA_Out();
	IIC_SCL_RESET;
	IIC_SDA_RESET;
	Delay_Us(4);
	IIC_SCL_SET;
	Delay_Us(4);
	IIC_SDA_SET;
	
}

void IIC_Respond(u8 re)								   //IICӦ����
{
	
	IIC_SDA_Out();
	if(re) 	IIC_SDA_SET;
	else	IIC_SDA_RESET;
	IIC_SCL_SET;
	Delay_Us(5);	 
	IIC_SCL_RESET;
	Delay_Us(2);
}
u8 IIC_WaitAck(void)
{
	int Out_Time=300;
	
	IIC_SDA_SET;
	IIC_SDA_In();
	Delay_Us(1);
	IIC_SCL_SET;
	Delay_Us(1);

       
	while(IIC_SDA_GET)
	{
		
		if(--Out_Time)
		{
			
			IIC_Stop();
			return 1;
		}
	}
     
	IIC_SCL_RESET;
	return 0;

} 
void IIC_Init(void)										   //IIC��ʼ������
{
	RCC->APB2ENR |= 0x00000008;
	GPIOB->CRL&=0X00FFFFFF;
	GPIOB->CRL|=0x33000000;	
   	IIC_SDA_Out();
   	IIC_SDA_SET;
   	IIC_SCL_SET;
}
void IIC_WriteBit(u8 Temp)		   //��2402��дһ�ֽ�����
{
	u8 i;
	IIC_SDA_Out();
	IIC_SCL_RESET;
	for(i=0;i<8;i++)
	{
		if(Temp&0x80)
		{
			IIC_SDA_SET;		                
		}
		else
		{
			IIC_SDA_RESET;
		}
		Temp<<=1;
		Delay_Us(2);
		IIC_SCL_SET;
		Delay_Us(4);
		IIC_SCL_RESET;
		Delay_Us(2);
		
	}

}

u8 IIC_ReadBit(void)					   //��2402�ж�ȡһ�ֽں���
{
	u8 i,Temp;
	
	IIC_SDA_In();
	for(i=0;i<8;i++)
	{
		IIC_SCL_RESET;
		Delay_Us(2);
		IIC_SCL_SET;
		Temp<<=1;							   //����ԭ�������� д��Temp<<1;
		if(IIC_SDA_GET)Temp++;
		Delay_Us(2);		     
	}
	IIC_SCL_RESET;
	return Temp;
}

void AT24C02_WriteByte(u8 Address,u8 W_Data)			 //��ָ����ַ��д������
{
	
	IIC_Start();
	IIC_WriteBit(0xa0);												 //д��������ַ
	IIC_WaitAck();

	IIC_WriteBit(Address);											 //д���ݵ�ַ
	IIC_WaitAck();
	IIC_WriteBit(W_Data);											  //д����
	IIC_WaitAck();
	IIC_Stop();
	Delay_Us(1000);
}
u8 AT24C02_ReadByte(u8 Address)						 //��ָ����ַ�ж�ȡ����
{
	u8 Temp;
	IIC_Start();
	IIC_WriteBit(0xa0);
	IIC_WaitAck();
	IIC_WriteBit(Address);
	IIC_WaitAck();
	IIC_Start();
	IIC_WriteBit(0xa1);
	IIC_WaitAck();
	Temp=IIC_ReadBit();
	IIC_Stop();
	return Temp;	
}
u8 IIC_WriteData(u8 dev_addr, u8 reg_addr, u8 *pdata, u8 count)			 //��ָ����ַ��д������
{
	u8 i;
	IIC_Start(); 
	IIC_WriteBit(dev_addr);		 									 //д��������ַ
	IIC_WaitAck();
	
	IIC_WriteBit(reg_addr);											 //д���ݵ�ַ
	IIC_WaitAck();
	for(i=0; i<count; i++)
	{
           
		IIC_WriteBit(*pdata);											  //д����
		IIC_WaitAck();
	
		pdata++;
	}	
        	
	IIC_Stop();
      
        Delay_Us(10000);
	return 0;

}
u8 IIC_ReadData(u8 dev_addr, u8 reg_addr)						 //��ָ����ַ�ж�ȡ����
{
	u8 Temp;
 
	IIC_Start();
       
	IIC_WriteBit(dev_addr);
	IIC_WaitAck();

	IIC_WriteBit(reg_addr);
	IIC_WaitAck();
	IIC_Start();

	IIC_WriteBit(dev_addr+1);
	IIC_WaitAck();

	Temp=IIC_ReadBit();
	IIC_Stop();		 
	return Temp;	
}



unsigned char IIC_ReadMPU(u8 byAD, u8 byRA, u8 * pData, u8 byCount )
{
	 u8 i;

    IIC_Start();                   /*��������*/
    IIC_WriteBit(byAD);                /*����������ַ*/   
    IIC_WaitAck();
    
    IIC_WriteBit(byRA);                /*���������ӵ�ַ---���ݼĴ���*/
    IIC_WaitAck();
    IIC_Start();                   /*������������*/

    IIC_WriteBit(byAD + 1);//address_read
    IIC_WaitAck();

    for(i = 0; i < (byCount-1); i++)
    {
        *pData=IIC_ReadBit();                /*��������*/
        IIC_Respond(0);                       /*���;ʹ�λ*/  
        pData++;
    }

    *pData=IIC_ReadBit();
    IIC_Respond(1);                    /*���ͷ�Ӧλ*/
    IIC_Stop();                    /*��������*/ 
    return(1);	
}


unsigned char IIC_ReadCmpData(unsigned char byAD, unsigned char * pData, 
                                unsigned char byCount)
{
    u8 i;

    IIC_Start();                   /*��������*/
    IIC_WriteBit(byAD);                /*����������ַ*/   
    IIC_WaitAck();

    for(i = 0; i < (byCount-1); i++)
    {
        *pData=IIC_ReadBit();                /*��������*/
        IIC_Respond(0);                      /*���;ʹ�λ*/  
        pData++;
    }

    *pData=IIC_ReadBit();
    IIC_Respond(1);                    /*���ͷ�Ӧλ*/
    IIC_Stop();                    /*��������*/ 
    return(1);
}




