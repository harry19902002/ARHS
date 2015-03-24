/***********************************************************************
File Name : 	hmc.h
Author :  		Name_006
Version :  		V1.0
Date :  		2011.12.10
Description :  	����������ͷ�ļ�
Others��		�ο�����Colin���� �ش˸�л~~
				�����Ա� http://shop65888055.taobao.com/
				רӪ ������ ���ٶȼ� ���贫���� �ȴ�����ģ��  ��ӭѡ��
				QQ �� 353973314  ��̬�㷨Ⱥ �� 199376386
**********************************************************************/
#ifndef __HMC5883L_H__
#define  __HMC5883L_H__

#define DEVICE_IIC_ADDR    0x1E
#define DEVICE_RD_ADDR     0x3D
#define DEVICE_WR_ADDR     0x3C		//д

#define RadToDeg 57.29578

#define CFG_A       0x00
#define CFG_B       0x01
#define MODE        0x02
#define X_M         0x03
#define X_L         0x04
#define Z_M         0x05
#define Z_L         0x06
#define Y_M         0x07
#define Y_L         0x08
#define STATUS      0x09
#define IDF_A       0x0A
#define IDF_B       0x0B
#define IDF_C       0x0C


// �Ĵ�����ַ����
void InitCmp(void);
void ReadCmpOut( int *pwX,int *pwY, int *pwZ,int *angle_int);

#endif

