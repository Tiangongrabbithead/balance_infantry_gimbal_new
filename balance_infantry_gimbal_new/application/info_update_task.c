#include "info_update_task.h"
#include "usart.h"
#include "judge.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "stdbool.h"
//#include "ina226.h"
//#include "crc_check.h"
#include "i2c.h"
#include "bsp_usart.h"
#include "chassis_task.h"
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
//ina226_t ina226_data;
//裁判系统发过来的数据暂存在这里
//uint32_t my_system_time_ticks    = 0;  /* 系统时钟计数器，在SysTick_Handler(void)中累加 */
//uint32_t my_system_timeout_ticks = 0;  /* 超时计数器 */
uint8_t Judge_Buffer[JUDGE_BUFFER_LEN] = {0};

bool Judge_Read_Data(uint8_t *ReadFromUsart);

void Info_update_Task(void const * argumt)
{
		usart1_init();
	  usart6_init();
		vTaskDelay(10);
	  uint32_t currentTime;
//		INA226_setConfig(&hi2c2,INA226_ADDRESS, 0x4527 );
//		INA226_setCalibrationReg(&hi2c2, INA226_ADDRESS, INA226_CALIB_VAL);
	for(;;)
	{
//	   currentTime = xTaskGetTickCount();//当前系统时间		
			vTaskDelay(2);			//2ms	
			Judge_Read_Data(Judge_Buffer);		//读取裁判系统数据	
//			ina226_data.BusV = INA226_getBusV(&hi2c2,INA226_ADDRESS);
//			ina226_data.Current = INA226_getCurrent(&hi2c2,INA226_ADDRESS);
//			ina226_data.Power = INA226_getPower(&hi2c2,INA226_ADDRESS);
//			if(my_system_time_ticks - my_system_timeout_ticks >= 10 )/* 定时10ms */
//			{				
//				my_system_timeout_ticks = my_system_time_ticks;	

//				pm01_access_poll();

//		    vTaskDelayUntil(&currentTime, 100);//绝对延时
//			}
	}
}


//void Cap_Update_Task(void const * argumt)
//{
//	for(;;)
//	{
//		vTaskDelay(1000);
//		pm01_access_poll();	
//	}
//}


void USART1_IRQHandler(void)
{
     if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
		}	
	  __HAL_DMA_DISABLE(huart1.hdmarx);
	

	//????????????????????????????????????
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);
//	__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5 );
	
//    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
//    {
//       HAL_UART_Receive_DMA(&huart3,Judge_Buffer,JUDGE_BUFFER_LEN);
//    }
//	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, JUDGE_BUFFER_LEN);

		__HAL_DMA_ENABLE(huart1.hdmarx);
		HAL_UART_Receive_DMA(&huart1,Judge_Buffer,JUDGE_BUFFER_LEN);		
				Judge_Read_Data(Judge_Buffer);
}

// void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART6_IRQn 0 */
//	DRV_UART_IRQHandler(&huart6);
//  /* USER CODE END USART6_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART6_IRQn 1 */
//     if(USART6->SR & UART_FLAG_IDLE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart6);
//		}	
//	  __HAL_DMA_DISABLE(huart6.hdmarx);
//	

//	//????????????????????????????????????
//		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);
////	__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5 );
//	
////    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
////    {
////       HAL_UART_Receive_DMA(&huart3,Judge_Buffer,JUDGE_BUFFER_LEN);
////    }
////	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
//    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, JUDGE_BUFFER_LEN);

//		__HAL_DMA_ENABLE(huart6.hdmarx);
//		HAL_UART_Receive_DMA(&huart6,Judge_Buffer,JUDGE_BUFFER_LEN);		
//		Judge_Read_Data(Judge_Buffer);
//  /* USER CODE END USART6_IRQn 1 */
//}
void USART6_IRQHandler(void)
{
   if(USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);
		}	
	  __HAL_DMA_DISABLE(huart6.hdmarx);
	

		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);

    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, JUDGE_BUFFER_LEN);

		__HAL_DMA_ENABLE(huart6.hdmarx);
			HAL_UART_Receive_DMA(&huart6,Judge_Buffer,JUDGE_BUFFER_LEN);		
			Judge_Read_Data(Judge_Buffer);
}

