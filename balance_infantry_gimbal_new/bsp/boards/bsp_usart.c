#include "bsp_usart.h"
#include "Calibrate_Task.h"
//#include "Vision_Task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "judge.h"
#include "Info_update_Task.h"
extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t Judge_Buffer[JUDGE_BUFFER_LEN];

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

/* Private macro -------------------------------------------------------------*/
//#define USART1_RX_DATA_FRAME_LEN	(18u)	// 串口2数据帧长度
//#define USART1_RX_BUF_LEN			(USART1_RX_DATA_FRAME_LEN + 6u)	// 串口2接收缓冲区长度
#define USART1_RX_BUF_LEN           16
#define USART6_RX_BUF_LEN			200//16
//#define UART8_RX_BUF_LEN			1

/* Private function prototypes -----------------------------------------------*/
__WEAK void USART1_rxDataHandler(uint8_t *rxBuf);
__WEAK void USART6_rxDataHandler(uint8_t *rxBuf);

static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength);

/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//裁判系统用
uint8_t usart6_dma_rxbuf[USART6_RX_BUF_LEN];
//测距用
uint8_t usart1_dma_rxbuf[USART1_RX_BUF_LEN];
//uint8_t uart8_dma_rxbuf[UART8_RX_BUF_LEN];


void TOFsense_process(void);
void process(void);

/* Exported variables --------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
//	// 将当前目标内存设置为Memory1
//	hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
//	USART1_rxDataHandler(usart1_dma_rxbuf[0]);
}

static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
//	// 将当前目标内存设置为Memory0
//	hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
//	USART1_rxDataHandler(usart1_dma_rxbuf[1]);
}

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in DRV_UART_IRQHandler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);	
	/* handle received data in idle interrupt */
	if (huart == &huart6)
	{
		/* clear DMA transfer complete flag */
//		__HAL_DMA_DISABLE(huart->hdmarx);
		HAL_UART_DMAStop(&huart6);
		/* handle dbus data dbus_buf from DMA */	
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);
//		__HAL_DMA_SET_COUNTER(&hdma_usart6_rx, USART6_RX_BUF_LEN);
		USART6_rxDataHandler(usart6_dma_rxbuf);			
		memset(usart6_dma_rxbuf, 0, USART6_RX_BUF_LEN);
		/* restart dma transmission */	 
		HAL_UART_Receive_DMA(&huart6,usart6_dma_rxbuf,USART6_RX_BUF_LEN);//重新打开DMA接收
//		__HAL_DMA_ENABLE(huart->hdmarx);	
	}
}



/**
 *	@brief	start double buffer dma transfer with no interrupt
 */
static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/* Set the UART DMA transfer complete callback */
	/* Current memory buffer used is Memory 1 callback */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Current memory buffer used is Memory 0 callback */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/* Check callback functions */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
	hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
	return HAL_ERROR;
	}
	
	/* Process locked */
	__HAL_LOCK(hdma);
	
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Enable the Double buffer mode */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* Configure DMA Stream destination address */
		hdma->Instance->M1AR = SecondMemAddress;		

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;		
		
		/* Peripheral to Memory */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Memory to Peripheral */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/* Clear TC flags */
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Enable TC interrupts*/
//		hdma->Instance->CR  |= DMA_IT_TC;
		
		/* Enable the peripheral */
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);	  

		/* Return error status */
		status = HAL_BUSY;		
	}
	/* Process unlocked */
	__HAL_UNLOCK(hdma);

	return status; 	
}

static HAL_StatusTypeDef DMA_Start(DMA_HandleTypeDef *hdma, \
                            uint32_t SrcAddress, \
                            uint32_t DstAddress, \
                            uint32_t DataLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	/* Process locked */
	__HAL_LOCK(hdma);
	if(HAL_DMA_STATE_READY == hdma->State)
	{
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Configure the source, destination address and the data length */
		/* Clear DBM bit */
		hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;

		/* Memory to Peripheral */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Peripheral to Memory */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}

		/* Enable the Peripheral */
		__HAL_DMA_ENABLE(hdma);
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);

		/* Return error status */
		status = HAL_BUSY;
	} 
	return status; 	
}
/* Exported functions --------------------------------------------------------*/
/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void DRV_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    // 判断是否为空闲中断
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}

void usart6_init(void)
{
//	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//	
//	// Enable the DMA transfer for the receiver request
//	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);	
//	
//	DMA_Start(huart6.hdmarx, \
//			  (uint32_t)&huart6.Instance->DR, \
//			  (uint32_t)usart6_dma_rxbuf, \
//			  USART6_RX_BUF_LEN);
	



    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	
	//enable DMA
    //使能DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
	
	//?????????????
   while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }
		
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);
	hdma_usart6_rx.Instance->M0AR = (uint32_t)(Judge_Buffer);
	
	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	__HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
	__HAL_DMA_ENABLE(&hdma_usart6_rx);
}

/**
 *	@brief	USART6 SendData
 */
void UART6_SendData(uint8_t *Data,uint16_t Size)
{
	HAL_UART_Transmit(&huart6,Data,Size,1);
}



/**
 *	@brief	[__WEAK] 需要在Potocol Layer中实现具体的 USART3 处理协议
 */


__WEAK void USART6_rxDataHandler(uint8_t *rxBuf)
{	
}



void usart1_init()
{

//    //enable the DMA transfer for the receiver and tramsmit request
//    //使能DMA串口接收和发送
//    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
//    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

//    //enalbe idle interrupt
//    //使能空闲中断
//    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



//    //disable DMA
//    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart6_rx);
//    
//    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart6_rx);
//    }

//    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

//    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
//    //memory buffer 1
//    //内存缓冲区1
//    hdma_usart6_rx.Instance->M0AR = (uint32_t)(Judge_Buffer);

//    //data length
//    //数据长度
//    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, USART_RX_BUF_LENGHT);
//    //enable DMA
//    //使能DMA
//    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	//enable DMA
    //使能DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
	
	//?????????????
   while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
		
	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5);
	hdma_usart1_rx.Instance->M0AR = (uint32_t)(Judge_Buffer);
	
	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
	__HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
	__HAL_DMA_ENABLE(&hdma_usart1_rx);

}
