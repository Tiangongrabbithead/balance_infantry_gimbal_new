#ifndef __BSP_USART_H
#define __BSP_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"



void usart1_init(void);


void DRV_UART_IRQHandler(UART_HandleTypeDef *huart);
void UART6_SendData(uint8_t *Data,uint16_t Size);	
extern void usart6_init(void);		
#ifdef __cplusplus
}
#endif

#endif 
