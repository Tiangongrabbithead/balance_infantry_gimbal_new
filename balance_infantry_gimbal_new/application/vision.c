#include "vision.h"
#include "cmsis_os.h"
#include "main.h"
#include "INS_task.h"	
#include "Gimbal_Task.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "stdarg.h"
#include "crc8_crc16.h"
#include "stdbool.h"
#include "AHRS_MiddleWare.h"
#include "math.h"
#include "judge.h"

extVisionSendHeader_t    VisionSendHeader;  //头
extVisionRecvData_t      VisionRecvData_Up;    //视觉接收结构体
extVisionRecvData_t      VisionRecvData_Down;    //视觉接收结构体
extVisionSendData_t      VisionSendData;    //视觉发送结构体
Gyroscope_data_sent vision_output;
vision_info_t  vision_info;

//接收到的视觉数据暂存在这里
extern  uint8_t  my_Vision_data[ VisionBufferLength_SH ];

uint32_t Vision_Time_Test[2] = {0};//前后两次事件
uint16_t Vision_Ping = 0;//测试时间间隔
//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Up_Get_New_Data = FALSE,Vision_Down_Get_New_Data = FALSE;

//自瞄陀螺仪赋值
void Vision_init()
{
	vision_output.ins_accel = get_accel_data_point();
	vision_output.ins_gyro = get_gyro_data_point();
	vision_output.ins_quat = get_INS_quat_point();

}void Vision_Data_Update()
{
	vision_output.INS_accel1[0] = vision_output.ins_accel[0];
	vision_output.INS_accel1[1] = vision_output.ins_accel[1];
	vision_output.INS_accel1[2] = vision_output.ins_accel[2];
	vision_output.INS_gyro1[0] = vision_output.ins_gyro[0];
	vision_output.INS_gyro1[1] = vision_output.ins_gyro[1];
	vision_output.INS_gyro1[2] = vision_output.ins_gyro[2];
	vision_output.INS_quat1[0] = vision_output.ins_quat[0];
	vision_output.INS_quat1[1] = vision_output.ins_quat[1];
	vision_output.INS_quat1[2] = vision_output.ins_quat[2];
	vision_output.INS_quat1[3] = vision_output.ins_quat[3];
}



//打符是否换装甲了
uint8_t Vision_Armor = FALSE;
void Vision_Read_Data(vision_info_t* vision_update ,uint8_t *ReadFromUsart)
{
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[0] == 0xA5)
	{
		//帧头CRC8校验
		if(verify_CRC8_check_sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//帧尾CRC16校验
			if(verify_CRC16_check_sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//接收数据拷贝
				memcpy(&vision_update->RxPacket, ReadFromUsart, VISION_LEN_PACKED);	
				//帧计算
				vision_update->State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_update->State.rx_time_fps  = vision_update->State.rx_time_now - vision_update->State.rx_time_prev;
				vision_update->State.rx_time_prev = vision_update->State.rx_time_now;
				
				vision_update->State.rx_data_update = true;//标记视觉数据更新了
			}
		}
	}
	
//	 if(ReadFromUsart[0] == VISION_SOF_UP||ReadFromUsart[21] == VISION_SOF_UP)
//	{
//		if(ReadFromUsart[21] == VISION_SOF_UP)
//		{
//			ReadFromUsart = ReadFromUsart+21;
//		}
//		
//		//帧头CRC8校验
//		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
//		{
//			//帧尾CRC16校验
//			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
//			{
//				//接收数据拷贝
//				memcpy( &VisionRecvData_Up, ReadFromUsart, VISION_LEN_PACKED);	
//				Vision_Up_Get_New_Data = TRUE;//标记视觉数据更新了
////				//帧计算
////				Vision_Time_Test[NOW] = xTaskGetTickCount();
////				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
////				Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
//			}
//		}
//	}
	
	if(VisionRecvData_Down.yaw_angle == 99.99f)
	{
		memset(my_Vision_data, 0, 100);
	}
}
//发送函数
void Vision_Send_Data( uint8_t CmdID )
{
	uint8_t vision_send_pack[64] = {0};//大于22就行

	VisionSendHeader.SOF = VISION_SOF_UP;
	VisionSendHeader.CmdID = CmdID;//对视觉来说最重要的数据
	
	//写入帧头
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//帧头CRC8校验协议
	append_CRC8_check_sum( vision_send_pack, VISION_LEN_HEADER );
	
	//中间数据不用管,视觉用不到,用到了也是后面自瞄自动开火,用到角度补偿数据
	vision_info.TxPacket.INS_quat_send[0] = vision_output.INS_quat1[0];
	vision_info.TxPacket.INS_quat_send[1] = vision_output.INS_quat1[1];
	vision_info.TxPacket.INS_quat_send[2] = vision_output.INS_quat1[2];
	vision_info.TxPacket.INS_quat_send[3] = vision_output.INS_quat1[3];
	vision_info.TxPacket.INS_gyro_send[0] = vision_output.INS_gyro1[0];
	vision_info.TxPacket.INS_gyro_send[1] = vision_output.INS_gyro1[1];
	vision_info.TxPacket.INS_gyro_send[2] = vision_output.INS_gyro1[2];
	vision_info.TxPacket.INS_accel_send[0] = vision_output.INS_accel1[0];
	vision_info.TxPacket.INS_accel_send[1] = vision_output.INS_accel1[1];
	vision_info.TxPacket.INS_accel_send[2] = vision_output.INS_accel1[2];

	vision_info.TxPacket.Bullet_speed[0] =  JUDGE_usGetSpeedHeat();

//	memcpy(vision_info.TxPacket.INS_quat_send,gimbal_control.Vision_INS_quat,4);
//	memcpy(vision_info.TxPacket.INS_gyro_send,gimbal_control.Vision_INS_gyro,3);
//	memcpy(vision_info.TxPacket.INS_accel_send,gimbal_control.Vision_INS_accel,3);
 
   for(uint8_t i=0;i<=14;i++)
	 vision_info.TxPacket.Vision_empty_time[i]=0;
	 
   memcpy( vision_send_pack + 3, &vision_info.TxPacket, VISION_LEN_DATA);
	HAL_Delay(1);
	//帧尾CRC16校验协议
	append_CRC16_check_sum( vision_send_pack, VISION_LEN_PACKED );
	
	//将打包好的数据发送
	CDC_Transmit_FS(vision_send_pack, VISION_LEN_PACKED);
		
	memset(vision_send_pack, 0, VISION_LEN_PACKED);
}

/**
  * @brief  获取yaw误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Yaw(float *error,bool_t which_gimbal)
{
//	if(which_gimbal==GUP)
//	{
//		//视觉左负右正,请根据云台控制角度选择正负(左加右减)
//		*error = (-VisionRecvData_Up.yaw_angle +0) / 360.0f * 6.28f;
//	          //* 8192.0f / 360.0f / 10.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
//		if(VisionRecvData_Up.yaw_angle == 0)//发零
//		{
//			*error = 0;
//		}
//	}
//	else if(which_gimbal==GDOWN)
//	{
		//视觉左负右正,请根据云台控制角度选择正负(左加右减)
		*error = (vision_info.RxPacket.yaw_angle +0.0f) / 360.0f * 6.28f;
	          //* 8192.0f / 360.0f / 10.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
		if(vision_info.RxPacket.yaw_angle == 0)//发零
		{
			*error = 0;
		}
//	}

}

/**
  * @brief  获取pitch误差角度，自瞄专用
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Pitch(float *error,bool_t which_gimbal)
{	
//	if(which_gimbal==GUP)
//	{
//		*error = -(VisionRecvData_Up.pitch_angle + 0.0f)/ 360.0f * 6.28f ;
//		//深大此处数值为0，但是程序是有对距离进行补偿的代码的。
//		if(VisionRecvData_Up.pitch_angle == 0)
//		{
//			*error = 0;
//		}
//	}
//	else if(which_gimbal==GDOWN)
//	{
		*error =(vision_info.RxPacket.pitch_angle - 0.0f)/ 360.0f * 6.28f ;//- 18.0f
		//深大此处数值为0，但是程序是有对距离进行补偿的代码的。
		if(vision_info.RxPacket.pitch_angle == 0)
		{
			*error = 0;
		}
//	}
//	
}


/**
  * @brief  获取距离  单位M
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance,bool_t which_gimbal)
{
//	if(which_gimbal==GUP)
//	{
//		*distance = VisionRecvData_Up.distance;
//		if(VisionRecvData_Up.distance < 0)
//		{
//			*distance = 0;
//		}
//	}
//	else if(which_gimbal == GDOWN)
//	{
		*distance = vision_info.RxPacket.distance;
		if(vision_info.RxPacket.distance < 0)
		{
			*distance = 0;
		}
//	}
}

/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_Up_If_Update(void)
{
	return Vision_Up_Get_New_Data;
}

//判断视觉有数据
bool Vision_Down_If_Update(void)
{
	return vision_info.State.rx_data_update;
} 

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Up_Clean_Update_Flag(void)
{
	Vision_Up_Get_New_Data = FALSE;
}
void Vision_Down_Clean_Update_Flag(void)
{
	vision_info.State.rx_data_update = false;
}

