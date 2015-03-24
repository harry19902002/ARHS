#ifndef __IIC_H
#define __IIC_H

#include "stm32f10x.h"



void IIC_SDA_Out(void);
void IIC_SDA_In(void);
void IIC_Start(void);								   //IIC开始函数
void IIC_Stop(void);									   //IIC结束函数
void IIC_Init(void)	;
u8 IIC_WaitAck(void) ;
void IIC_Respond(u8 re);								   
void IIC_WriteBit(u8 Temp);		   
u8 IIC_ReadBit(void);					   
u8 IIC_WriteData(u8 dev_addr, u8 reg_addr, u8 *pdata, u8 count);			 
//u8 IIC_ReadData(u8 Address)	;					 
u8 IIC_ReadGyroData(u8 byAD, u8 byRA, u8 * pData, u8 byCount);
u8 AT24C02_ReadByte(u8 Address)	;
void AT24C02_WriteByte(u8 Address,u8 W_Data);
u8 IIC_ReadData(u8 dev_addr, u8 reg_addr);
unsigned char IIC_ReadMPU(u8 byAD, u8 byRA, u8 * pData, u8 byCount );
unsigned char IIC_ReadCmpData(unsigned char byAD, unsigned char * pData, 
                                unsigned char byCount);

#endif





