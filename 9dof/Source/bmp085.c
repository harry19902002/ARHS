#include "IIC.h"
#include "bmp085.h"
#include "stdio.h"
int AC1,AC2,AC3;
unsigned int AC4,AC5,AC6;
int B1,B2;
int MB,MC,MD;
long UT;
long UP;
long Press,T;

void BMP_Init(void)
{
	u8 btemp[22];
	u8 i;
	for(i=0; i<22; i++)
	{
		btemp[i]=IIC_ReadData(0xEE, 0xAA+i);	
	}
	AC1= (btemp[0]<<8) | btemp[1];
	AC2= (btemp[2]<<8) | btemp[3];
	AC3= (btemp[4]<<8) | btemp[5];
	AC4= (btemp[6]<<8) | btemp[7];
	AC5= (btemp[8]<<8) | btemp[9];
	AC6= (btemp[10]<<8) | btemp[11];

	B1= (btemp[12]<<8) | btemp[13];
	B2= (btemp[14]<<8) | btemp[15];

	MB= (btemp[16]<<8) | btemp[17];
	MC= (btemp[18]<<8) | btemp[19];
	MD= (btemp[20]<<8) | btemp[21];

	printf("AC1: %d\r\n",AC1);
	printf("AC2: %d\r\n",AC2);
	printf("AC3: %d\r\n",AC3);
	printf("AC4: %d\r\n",AC4);
	printf("B2: %d\r\n",B2);
	
				
}
void ReadBmpUT(u8 OSS )
{
   	u8 i;
	u8 btemp[3];
	u8 Bmp_Cmd;

	long X1,X2,X3,B3,B5,B6;
	unsigned long B4,B7;
	

	Bmp_Cmd=0x2E;									//开始转换温度命令
	IIC_WriteData(0xEE, 0xF4, &Bmp_Cmd, 1);		 	//发送开始转换的命令
	Delay_Ms(5);									//延时 4.5ms以上；
	for(i=0; i<2; i++)
	{
		btemp[i]=IIC_ReadData(0xEE, 0xF6+i);		//发送读取结果命令		
	}
	UT=(btemp[0]<<8) | btemp[1];
//	LCD_WriteNum(250,100,UT,6,ASCI_0816,1);
	btemp[0]=0;
	btemp[1]=0;
	btemp[2]=0;
	
	Bmp_Cmd=0x34 + OSS<<6;							//开始转换压力命令
	IIC_WriteData(0xEE, 0xF4, &Bmp_Cmd, 1);
	Delay_Ms(20); 
	for(i=0; i<3; i++)
	{
		btemp[i]=IIC_ReadData(0xEE, 0xF6+i);		//发送读取结果命令		
	}
	UP=((btemp[0]<<16) | btemp[1]<<8 | btemp[2])>>(8-OSS);
//	LCD_WriteNum(250,50,UP,6,ASCI_0816,1);
	
	X1=(UT-AC6)*(AC5>>15);						   	//计算温度
	X2=(MC<<11)/(X1+MD);
	B5=X1+X2;
	T=(B5+8)>>4;

	
	B6=B5-4000;										//计算压力
	X1=B2;
	X1=(X1*((B6*B6)>>12))>>11;
//	X2=AC2;
	X2=(X2*B6)>>11;
	X3=X1+X2;
//	B3=AC1;
	B3=(((AC1*4+X3)<<OSS)+2)>>2;	
//	X1=AC3;
	X1=(AC3*B6)>>13;
//	X2=B1;
	X2=(B1*((B6*B6)>>12))>>16;
	X3=((X1+X2)+2)>>2;
//	B4=AC4;
	B4=(AC4*(unsigned long)(X3+32768))>>15;
	B7=((unsigned long)UP-B3)*(50000>>OSS);	
	if(B7<0x80000000)
	{Press=(B7*2)/B4;}
	else {Press=(B7/B4)*2;}
	X1=(Press>>8)*(Press>>8);
	X1=(X1*3038)>>16;
	X2=(-7357*Press)>>16;
	Press=Press+(X1+X2+3791)>>4;

}















