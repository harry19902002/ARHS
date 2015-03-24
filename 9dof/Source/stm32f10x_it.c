
#include "stm32f10x_it.h"
#include "stm32f10x_usart.h"
#include <stdio.h>
#include <ins.h>
#include <acc_comp.h>
#include <MPU6050.h>
#include "hmc.h"
#include "stm32f10x_conf.h"
#include <math.h>


#define int16_t int
float  d1[4]={1,0,0,0};
extern float angle_c[3];
extern float q_error[7];
extern float af[3];
extern int flagzuhe;
extern int caflag;
extern int ca_flag;/*中断自标定标志位*/
int flag=1;/*姿态初始化标志位，这个也可以在电子罗盘中使用*/
int count=0;
int count_ca=0;
float ACCk1a[3],ACCk2a[3]={0};
float mpuoffsetx=0;
float mpuoffsety=0;
float mpuoffsetz=0;
float mpugsetx=0;
float mpugsety=0;
float mpugsetz=0;
float mpugkx=0;
float mpugky=0;
float mpugkz=0;
float mpuxx[100]={0};
float mpuyy[100]={0};
float mpuzz[100]={0};
float mpusumxx=0;
float mpusumyy=0;
float mpusumzz=0;
float insss[3]={0};
float kk0,kk1,kk2,kk3;
extern float PP[81];
float kaccx=0.00022611904672910;
float kaccy=0.00022587173715452;
float kaccz=0.00022596115528347;
float kaccyx=0.00000240639008783;
float kacczx=0.00000324865200147;
float kaccxy=0;
float kacczy=0.00000680384503523;
float kaccxz=0;
float kaccyz=0;
float accxoffset=2998;
float accyoffset=2490;
float acczoffset=2781;
float mpukx=0.0609756;
float mpuky=0.0609756;
float mpukz=0.0609756;//0.007633
float mpukyx=0;
float mpukzx=0;
float mpukxy=0;
float mpukzy=0;
float mpukxz=0;
float mpukyz=0;
float magkxx=0.00206462671802 ;
float magkyy=0.00209944148615;
float magkzz=0.00219841489868;
float magkyx=0.00004159929119;
float magkzx=0.00005731700049;
float magkzy=-0.00002956138373;
float magxoffset=40.73557872040836;
float magyoffset=52.06736729108723;
float mag0ffset=-7.93856818618579;
int calflag=0;
int16_t  yaww=0;
int16_t  pitchh=0;
int16_t  rolll=0;
int16_t  altt=0;
int16_t  tempp=0;
int16_t  presss=0;
int16_t  pre=0;
int16_t  aax=0;
int16_t  aay=0;
int16_t  aaz=0;
int16_t  ggx=0;
int16_t  ggy=0;
int16_t  ggz=0;
int16_t  hhx=0;
int16_t  hhy=0;
int16_t  hhz=0;

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
 
  while (1)
  {
    printf("\n\rhard error\n\r");
  }
}

void MemManage_Handler(void)
{
 
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}
void TIM2_IRQHandler(void)
{
 if(TIM2->SR&0X0001)//溢出中断
  {
  calflag=1;  
  }
 TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}
/*姿态解算程序*/
void cal(void)
{
 int gyrodatass[3];
 int i=0;
 int accggg[3]={0};
 int  magicdataa[4]={0};
 
 float anglee[3]={0};

 float cbna[9]={0};
 float agg[3]={0};
 float mangle[3]={0};
 float compss[3]={0};
 float gryo[3]={0};
 float gryox[16]={0};
 float gryoy[16]={0};
 float gryoz[16]={0};
 float aggx[16]={0};
 float aggy[16]={0};
 float aggz[16]={0};
 float mtt[3]={0};
 float q_erro[4]={0};

 

 for(i=0;i<5;i++)
 {
         MPU6050ReadGyro(gyrodatass);
         gryox[i]= (gyrodatass[0]-mpuoffsetx)*mpukx+(gyrodatass[1]-mpuoffsety)*mpukyx+(gyrodatass[2]-mpuoffsetz)*mpukzx;
         gryoy[i]= (gyrodatass[1]-mpuoffsety)*mpuky+(gyrodatass[0]-mpuoffsetx)*mpukxy+(gyrodatass[2]-mpuoffsetz)*mpukzy;
         gryoz[i]= (gyrodatass[2]-mpuoffsetz)*mpukz+(gyrodatass[0]-mpuoffsetx)*mpukxz+(gyrodatass[1]-mpuoffsety)*mpukyz;
         gryo[0]+=gryox[i];
         gryo[1]+=gryoy[i];
         gryo[2]+=gryoz[i];
          
         MPU6050ReadAcc(accggg);
         aggx[i]=(accggg[0]-accxoffset)*kaccx+(accggg[1]-accyoffset)*kaccyx+(accggg[2]-acczoffset)*kacczx;
         aggy[i]=(accggg[1]-accyoffset)*kaccy+(accggg[0]-accxoffset)*kaccxy+(accggg[2]-acczoffset)*kacczy;
         aggz[i]=(accggg[2]-acczoffset)*kaccz+(accggg[0]-accxoffset)*kaccxz+(accggg[1]-accyoffset)*kaccyz;
         agg[0]+=aggx[i];
         agg[1]+=aggy[i];
         agg[2]+=aggz[i];
 }
         gryo[0]*=0.2;
         gryo[1]*=0.2;
         gryo[2]*=0.2;
         agg[0]*=0.2;
         agg[1]*=0.2;
         agg[2]*=0.2;
         ReadCmpOut(&magicdataa[0],&magicdataa[1], &magicdataa[2], &magicdataa[3]);
          
         compss[0]=(magicdataa[0]-magxoffset)*magkxx+(magicdataa[1]-magyoffset)*magkyx+(magicdataa[2]-mag0ffset)*magkzx; 
         compss[1]=(magicdataa[1]-magyoffset)*magkyy+(magicdataa[2]-mag0ffset)*magkzy;
         compss[2]=(magicdataa[2]-mag0ffset)*magkzz;

            if(flag==1)
            {
           compass(agg,compss,mangle,mtt);
           init_dcm(mangle,d1);   
           }
           else
           {
            
           }


DCM(flag,d1,gryo,cbna,agg,compss,0.02,anglee,q_erro);
flag=0;
yaww=anglee[0]*10*57.296;
pitchh=anglee[1]*10*57.296;
rolll=anglee[2]*10*57.296;
UART1_ReportIMU(yaww,pitchh,rolll,altt,tempp,presss,pre);
}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
