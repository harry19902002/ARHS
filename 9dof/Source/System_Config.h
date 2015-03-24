#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H



#define LED1_ON GPIOB->BRR|=1<<8
#define LED2_ON GPIOB->BRR|=1<<9
#define LED3_ON GPIOE->BRR|=1<<0
#define LED4_ON GPIOE->BRR|=1<<1

#define LED1_OFF GPIOB->BSRR|=1<<8
#define LED2_OFF GPIOB->BSRR|=1<<9
#define LED3_OFF GPIOE->BSRR|=1<<0
#define LED4_OFF GPIOE->BSRR|=1<<1


#define PLL_MUL	10		// PLL9±¶Æµ

void ChipHardwareInit(void);
void ChipOutHardwareInit(void);
void GPIO_Configuration(void);
void RCC_Configuration(void);
void USART_Configuration(USART_TypeDef* USARTx, u32 baud);
void NVIC_Configuration(void);
void Delay_Us(u32 a);
void Delay_Ms(u16 a);
void Timer_Configuration(void);
u8 USART_Send(u8 us);
void EXTI_Configuration(void );
void Delay(u32 a);
u8 Sys_Init( void );
void Sys_DeInit( void );
u8 USART_CRCSend(int *ch1, int *ch2, int *ch3, int *ch4);
void CRC16(u8 *Array, u8 *Rcvbuf,u16 Len);
#endif

