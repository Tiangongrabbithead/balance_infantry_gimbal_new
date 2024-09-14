#ifndef __INFO_UPDATE_TASK_H
#define __INFO_UPDATE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "stdbool.h"	


//#define JUDGE_BUFFER_LEN            200
#define	USART_RX_BUF_LENGHT 		512
	
typedef struct
{
    float YawGet_KF;
    float YawTarget_KF;

    float PitchGet_KF;
    float PitchTarget_KF;

    float DistanceGet_KF;
    float DistanceTarget_KF;
} Visual_TypeDef;


typedef struct {
    QueueObj speed_queue;
    QueueObj accel_queue;
    QueueObj dis_queue;
    Visual_TypeDef  data_kal;
    float predict_angle;
    float feedforwaurd_angle;
    float speed_get;
    float accel_get;
    float distend_get;
    float offset_yaw;
    float offset_pitch;
    bool  gyro_anti;
    bool  gyro_judge;
} Vision_process_t;	
	
extern 	Vision_process_t Vision_process;
	
	
extern void Vision_Task(void const * argumt);

#ifdef __cplusplus
}
#endif

#endif 
