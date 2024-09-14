#ifndef __VISION_H
#define __VISION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"

#define TRUE 1
#define FALSE 0
/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        22     		 //暂定22字节,头3字节,数据17字节,尾2字节

//起始字节,协议固定为0xA5
#define    VISION_SOF_UP         (0xA5)
#define    VISION_SOF_DOWN         (0xB5)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    3         //帧头长
#define    VISION_LEN_DATA      59       //数据段长度,可自定义
#define    VISION_LEN_TAIL      2	      //帧尾CRC16
#define    VISION_LEN_PACKED   64       //数据包长度,可自定义

#define GUP 0
#define GDOWN 1

#define NOW  0
#define LAST 1

#define VisionBufferLength_SH       150

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

//帧头加CRC8校验,保证发送的指令是正确的

//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
}extVisionSendHeader_t;


//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{
	//3+3*4+5+42+2=64
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令

	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   is_switched;
	uint8_t   is_findtarget;
	uint8_t   is_spinning;
	uint8_t   is_shooting;
	uint8_t   is_middle;
	//uint8_t	  auto_too_close;   //目标距离太近,视觉发1，否则发0
	
	 uint8_t  Vision_empty_time[42];
	/* 尾 */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;


//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{
//	/* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5
//	uint8_t   CmdID;		//指令
//	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
//	float     pitch_angle;
//	float     yaw_angle;
//	float     distance;			//距离
//	uint8_t   lock_sentry;		//是否在抬头识别哨兵
//	uint8_t   base;				//吊射
//	
//	uint8_t   blank_a;		//预留
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
	//除去帧头3位 一共61位  11*4+15+2
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float		 pitch_revolver_angle;
	float		 yaw_relative_angle;
	float    Bullet_speed[1];
	
  uint8_t  Vision_empty_time[15];
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendData_t;


/**
 *	@brief	视觉模式
 */
typedef enum
{
	VISION_MODE_MANUAL		= 0,	// 手动模式
	VISION_MODE_AUTO		= 1,	// 自瞄模式
	VISION_MODE_BIG_BUFF	= 2,	// 打大符模式
	VISION_MODE_SMALL_BUFF	= 3,	// 打小符模式
} Vision_Mode_t;
/* 辅助标识变量 */
typedef struct
{
	uint8_t 		my_color;			// 用0/1表示颜色
	Vision_Mode_t	mode;				// 视觉模式
	uint8_t  		rx_data_valid;		// 接收数据的正确性
	uint16_t 		rx_err_cnt;			// 接收数据的错误统计
	uint32_t		rx_cnt;				// 接收数据包的统计
	bool		    rx_data_update;		// 接收数据是否更新
	uint32_t 		rx_time_prev;		// 接收数据的前一时刻
	uint32_t 		rx_time_now;		// 接收数据的当前时刻
	uint16_t 		rx_time_fps;		// 帧率
	bool            predict_state;       //是否开启预测

} Vision_State_t;



/* 视觉通信数据包格式 */
typedef struct {
	extVisionRecvData_t RxPacket;
	extVisionSendData_t TxPacket;
	Vision_State_t       State;
	float        real_time_bullet_speed;
} vision_info_t;
typedef struct {
 const fp32 *ins_quat;
 const fp32 *ins_gyro;
 const fp32 *ins_accel;
 fp32 INS_quat1[4];
 fp32 INS_gyro1[3];
 fp32 INS_accel1[3];
}Gyroscope_data_sent;
extern vision_info_t vision_info;
extern Gyroscope_data_sent vision_output;
void Vision_Send_Data( uint8_t CmdID );
void Vision_Read_Data(vision_info_t* vision_update ,uint8_t *ReadFromUsart);
//获取yaw误差角度
extern void Vision_Error_Angle_Yaw(float *error,bool_t which_gimbal);
//获取pitch误差角度
void Vision_Error_Angle_Pitch(float *error,bool_t which_gimbal);
//获取距离
void Vision_Get_Distance(float *distance,bool_t which_gimbal);

//判断视觉数据更新了吗
bool Vision_Down_If_Update(void);
//视觉数据更新标志位手动置0(false)
void Vision_Down_Clean_Update_Flag(void);
extern void Vision_Data_Update(void);
extern void Vision_init(void);
#ifdef __cplusplus
}
#endif

#endif 
