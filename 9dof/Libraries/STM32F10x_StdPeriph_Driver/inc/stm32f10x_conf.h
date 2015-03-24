/**
  ******************************************************************************
  * @file USART/Printf/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Library configuration file.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Uncomment the line below to enable peripheral header file inclusion */
/* #include "stm32f10x_adc.h" */
/* #include "stm32f10x_bkp.h" */
/* #include "stm32f10x_can.h" */
/* #include "stm32f10x_cec.h" */
/* #include "stm32f10x_crc.h" */
/* #include "stm32f10x_dac.h" */
/* #include "stm32f10x_dbgmcu.h" */
/* #include "stm32f10x_dma.h" */
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h" 
/* #include "stm32f10x_fsmc.h" */
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h" 
/* #include "stm32f10x_iwdg.h" */
/* #include "stm32f10x_pwr.h" */
#include "stm32f10x_rcc.h"
/* #include "stm32f10x_rtc.h" */
/* #include "stm32f10x_sdio.h" */
/* #include "stm32f10x_spi.h" */
 #include "stm32f10x_tim.h" 
#include "stm32f10x_usart.h"
/* #include "stm32f10x_wwdg.h" */
#include "misc.h"  /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */


#define LED1_ON GPIOB->BRR|=1<<8
#define LED2_ON GPIOB->BRR|=1<<9
#define LED3_ON GPIOE->BRR|=1<<0
#define LED4_ON GPIOE->BRR|=1<<1

#define LED1_OFF GPIOB->BSRR|=1<<8
#define LED2_OFF GPIOB->BSRR|=1<<9
#define LED3_OFF GPIOE->BSRR|=1<<0
#define LED4_OFF GPIOE->BSRR|=1<<1


#define PLL_MUL	9		// PLL9±¶Æµ

void ChipHardwareInit(void);
void ChipOutHardwareInit(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);
void USART_Configuration(USART_TypeDef* USARTx, u32 baud);
void NVIC_Configuration(void);
void uart_NVIC_Configuration(void);
void Delay_Us(u32 a);
void Delay_Ms(u16 a);
void Timer_Configuration(void);
u8 USART_Send(u8 us);
//void UART_Res(char ws);
void EXTI_Configuration(void );
void Delay(u32 a);
u8 Sys_Init( void );
void Sys_DeInit( void );
u8 USART_CRCSend(int *ch1, int *ch2, int *ch3, int *ch4);
void CRC16(u8 *Array, u8 *Rcvbuf,u16 Len);
void Stm32_Clock_Init(u8 PLL);
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
