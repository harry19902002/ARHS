/***********************************************************************
File Name : 	hmc.c
Author :  		Name_006
Version :  		V1.0
Date :  		2011.12.10
Description :  	�������������� ICѡ�õ��� HMC5883L
Others��		�ο�����Colin���� �ش˸�л~~
				�����Ա� http://shop65888055.taobao.com/
				רӪ ������ ���ٶȼ� ���贫���� �ȴ�����ģ��  ��ӭѡ��
				QQ ��353973314  ��̬�㷨Ⱥ �� 199376386
**********************************************************************/
#include <stdio.h>
#include <math.h>
#include <IIC.h>
#include "hmc.h"
#define u8 unsigned char
// ��ʼ��HMC5883L�������ο���Ҫ�����Լ�������е���
int cnt=0;
void InitCmp(void)
{

    u8 byData[4];

   // CFG_A=0xC1, CFG_B=0xA0, MODE=0x00�ɲ��Գ�ģ�麸�Ӻ�оƬ�Ƿ��ջ�����ʱ������X,Y,Z����Ϊ400-1000��������оƬ��������
    byData[0] = 0x70; //byData[0] = 0x70; //0xC0; 	// SELF-TESTʱΪ0xC1	   			 //ƽ��������8  15hz  
    byData[1] = 0x20; //byData[1] = 0x20; // 0x40; 	//0x20; // SELF-TESTʱΪ0xA0		 //��Χ+_1.9Ga   ����820��˹
    byData[2] = 0x00; // MODE // SELF-TESTʱΪ0X00    ����Ϊ��������ģʽ  			 //

    IIC_WriteData(DEVICE_WR_ADDR, CFG_A, byData, 3);   //����д�����Ĵ���

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

    // ��ѯ�����Ƿ����
    while (1)
    {
        IIC_WriteData(DEVICE_WR_ADDR, STATUS, byData, 0);	 //0x9  ��д������λ��ָ��	     0���ֽ�
        IIC_ReadCmpData(DEVICE_RD_ADDR, &status, 1);		 // ����״̬�Ĵ������ж������Ƿ����
        if (status&0x01 == 1)
        {
            break;
        }
        Delay_Us(20);
    }

    // ���ö�ָ��
    
    IIC_WriteData(DEVICE_WR_ADDR, X_M, byData, 0);	  //X_M   X��Ĵ����ߵ�ַ	 ��д

    // ������
    IIC_ReadCmpData(DEVICE_RD_ADDR, byData, len);	  //������дʱ����������,ֱ�Ӷ�ȡ�ӵ�ַ����
    

    wTemp  = 0;
    wTemp = byData[0] << 8;			 //��������Ϊ��λ��ǰ
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

//    *angle_int = (int)angle;			  //��5883l����Ƕȣ�������
	  *angle_int = 0;
    //printf("Double:%f\r\n", angle);

}


