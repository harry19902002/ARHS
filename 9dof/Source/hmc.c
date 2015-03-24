/***********************************************************************
File Name : 	hmc.c
Author :  		Name_006
Version :  		V1.0
Date :  		2011.12.10
Description :  	磁敏传感器部分 IC选用的是 HMC5883L
Others：		参考网友Colin程序 特此感谢~~
				本店淘宝 http://shop65888055.taobao.com/
				专营 陀螺仪 加速度计 磁阻传感器 等传感器模块  欢迎选购
				QQ ：353973314  姿态算法群 ： 199376386
**********************************************************************/
#include <stdio.h>
#include <math.h>
#include <IIC.h>
#include "hmc.h"
#define u8 unsigned char
// 初始化HMC5883L，仅供参考，要根据自己需求进行调整
int cnt=0;
void InitCmp(void)
{

    u8 byData[4];

   // CFG_A=0xC1, CFG_B=0xA0, MODE=0x00可测试出模块焊接后芯片是否被烧坏，此时如果输出X,Y,Z数据为400-1000的数据则芯片是正常的
    byData[0] = 0x70; //byData[0] = 0x70; //0xC0; 	// SELF-TEST时为0xC1	   			 //平均采样数8  15hz  
    byData[1] = 0x20; //byData[1] = 0x20; // 0x40; 	//0x20; // SELF-TEST时为0xA0		 //范围+_1.9Ga   增益820高斯
    byData[2] = 0x00; // MODE // SELF-TEST时为0X00    设置为连续测量模式  			 //

    IIC_WriteData(DEVICE_WR_ADDR, CFG_A, byData, 3);   //连续写三个寄存器

    cnt=3;

}


void ReadCmpOut( int *pwX,
                int *pwY, int *pwZ,int *angle_int)
{
    unsigned char  status=0;
    unsigned char byData[6];
    unsigned int wTemp;

    int i;
    unsigned char len = 6;
   // float xx=0,yy=0,zz=0;
 


    for (i=0; i<len; i++)
    {
        byData[i] = 0;
    }

    // 查询数据是否就绪
    while (1)
    {
        IIC_WriteData(DEVICE_WR_ADDR, STATUS, byData, 0);	 //0x9  虚写，设置位置指针	     0个字节
        IIC_ReadCmpData(DEVICE_RD_ADDR, &status, 1);		 // 读出状态寄存器，判断数据是否就绪
        if (status&0x01 == 1)
        {
            break;
        }
        Delay_Us(20);
    }

    // 设置读指针
    
    IIC_WriteData(DEVICE_WR_ADDR, X_M, byData, 0);	  //X_M   X轴寄存器高地址	 虚写

    // 读数据
    IIC_ReadCmpData(DEVICE_RD_ADDR, byData, len);	  //接上面写时序，重启总线,直接读取子地址内容
    

    wTemp  = 0;
    wTemp = byData[0] << 8;			 //磁力数据为高位在前
    wTemp |= byData[1];
    if(wTemp>0xf000){
    *pwX = wTemp-65536;
    }
    else
    {
    *pwX = wTemp;
    }
    wTemp  = 0;
    wTemp = byData[2] << 8;
    wTemp |= byData[3];
     if(wTemp>0xf000)
     {
    *pwZ = wTemp-65536;
     }
     else
     {
    *pwZ = wTemp;
     }
    wTemp  = 0;
    wTemp = byData[4] << 8;
    wTemp |= byData[5];
    if(wTemp>0xf000)
    {
    *pwY = wTemp-65536;
    }
    else
    {
    *pwY = wTemp;
    }


//	y = (double)(*pwY); //(float)(*pwY*YSF + YOffset);

//    angle = atan2((double)y,(double)x) * (180 / 3.14159265) + 180;

//    *angle_int = (int)angle;			  //由5883l计算角度？？？？
	  *angle_int = 0;
    //printf("Double:%f\r\n", angle);

}


