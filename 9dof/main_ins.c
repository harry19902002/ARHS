#include <stdio.h>

#include "misc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h" 
//#include "ins.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_it.h"
#include "stm32f10x.h"

#include "IIC.h"
#include <MPU6050.h> 
#include "hmc.h"


#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif


uint16_t cmd;
char D_data[4];
extern int calflag;

void UART_Res()
{
while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));
cmd=USART_ReceiveData(USART1);

}

int main(void)
{       
        int i=0;
        int initaccgg[3]={0};
        int accgg[3]={0};
        int  magicdataa[4]={0};
        float compss[3]={0};
        float agg[3]={0};
        /*ϵͳ�ʹ�������ʼ�������ȳ�ʼ�����ݣ��ٳ�ʼ�����ٶȣ����˳����AHRS����һ��Ӱ��*/
        ChipHardwareInit();  
        MPU6050GyroInit();	
        MPU6050AccInit(); 
       	InitCmp();
        MPU6050ReadAcc(initaccgg);        
        UART_Res();
       
        if(cmd==5)//0x05���궨��ʼ���ͨ��
          {
            while(1)
            {
        for(i=0;i<16;i++)
 {
          
         MPU6050ReadAcc(accgg);

         agg[0]+=accgg[0];
         agg[1]+=accgg[1];
         agg[2]+=accgg[2];
 }
         agg[0]*=0.0625;
         agg[1]*=0.0625;
         agg[2]*=0.0625;
         ReadCmpOut(&magicdataa[0],&magicdataa[1], &magicdataa[2], &magicdataa[3]);
          
         compss[0]=magicdataa[0]; 
         compss[1]=magicdataa[1];
         compss[2]=magicdataa[2];
         printf("%.3f %.3f %.3f %.3f %.3f %.3f\r\n",agg[0],agg[1],agg[2],compss[0],compss[1],compss[2]);/*����������ԭʼ���ݷ��������ڴ������궨*/
        }
          }

          if(cmd==35)//0x23������������ͨ��
          {
          //caflag=1;
          mpu_calibration();       
          Timer_Configuration();   
          NVIC_Configuration();
          
          while(1)
          {
            if(calflag==1)
            {
            cal();
            calflag=0;/*�����־λ�������־�ڶ�ʱ���ж��б���1��˵��������������*/
            }
            
            else
            {}
          }
          }
        }     
  


    PUTCHAR_PROTOTYPE
{

  USART_SendData(USART1, (uint8_t) ch);

  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}
  return ch;
}


