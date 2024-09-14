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

extVisionSendHeader_t    VisionSendHeader;  //ͷ
extVisionRecvData_t      VisionRecvData_Up;    //�Ӿ����սṹ��
extVisionRecvData_t      VisionRecvData_Down;    //�Ӿ����սṹ��
extVisionSendData_t      VisionSendData;    //�Ӿ����ͽṹ��
Gyroscope_data_sent vision_output;
vision_info_t  vision_info;

//���յ����Ӿ������ݴ�������
extern  uint8_t  my_Vision_data[ VisionBufferLength_SH ];

uint32_t Vision_Time_Test[2] = {0};//ǰ�������¼�
uint16_t Vision_Ping = 0;//����ʱ����
//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
uint8_t Vision_Up_Get_New_Data = FALSE,Vision_Down_Get_New_Data = FALSE;

//���������Ǹ�ֵ
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



//����Ƿ�װ����
uint8_t Vision_Armor = FALSE;
void Vision_Read_Data(vision_info_t* vision_update ,uint8_t *ReadFromUsart)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[0] == 0xA5)
	{
		//֡ͷCRC8У��
		if(verify_CRC8_check_sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//֡βCRC16У��
			if(verify_CRC16_check_sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//�������ݿ���
				memcpy(&vision_update->RxPacket, ReadFromUsart, VISION_LEN_PACKED);	
				//֡����
				vision_update->State.rx_time_now  = xTaskGetTickCountFromISR();
				vision_update->State.rx_time_fps  = vision_update->State.rx_time_now - vision_update->State.rx_time_prev;
				vision_update->State.rx_time_prev = vision_update->State.rx_time_now;
				
				vision_update->State.rx_data_update = true;//����Ӿ����ݸ�����
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
//		//֡ͷCRC8У��
//		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
//		{
//			//֡βCRC16У��
//			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
//			{
//				//�������ݿ���
//				memcpy( &VisionRecvData_Up, ReadFromUsart, VISION_LEN_PACKED);	
//				Vision_Up_Get_New_Data = TRUE;//����Ӿ����ݸ�����
////				//֡����
////				Vision_Time_Test[NOW] = xTaskGetTickCount();
////				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//����ʱ����
////				Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
//			}
//		}
//	}
	
	if(VisionRecvData_Down.yaw_angle == 99.99f)
	{
		memset(my_Vision_data, 0, 100);
	}
}
//���ͺ���
void Vision_Send_Data( uint8_t CmdID )
{
	uint8_t vision_send_pack[64] = {0};//����22����

	VisionSendHeader.SOF = VISION_SOF_UP;
	VisionSendHeader.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	
	//д��֡ͷ
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	append_CRC8_check_sum( vision_send_pack, VISION_LEN_HEADER );
	
	//�м����ݲ��ù�,�Ӿ��ò���,�õ���Ҳ�Ǻ��������Զ�����,�õ��ǶȲ�������
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
	//֡βCRC16У��Э��
	append_CRC16_check_sum( vision_send_pack, VISION_LEN_PACKED );
	
	//������õ����ݷ���
	CDC_Transmit_FS(vision_send_pack, VISION_LEN_PACKED);
		
	memset(vision_send_pack, 0, VISION_LEN_PACKED);
}

/**
  * @brief  ��ȡyaw���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void Vision_Error_Angle_Yaw(float *error,bool_t which_gimbal)
{
//	if(which_gimbal==GUP)
//	{
//		//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
//		*error = (-VisionRecvData_Up.yaw_angle +0) / 360.0f * 6.28f;
//	          //* 8192.0f / 360.0f / 10.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
//		if(VisionRecvData_Up.yaw_angle == 0)//����
//		{
//			*error = 0;
//		}
//	}
//	else if(which_gimbal==GDOWN)
//	{
		//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
		*error = (vision_info.RxPacket.yaw_angle +0.0f) / 360.0f * 6.28f;
	          //* 8192.0f / 360.0f / 10.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
		if(vision_info.RxPacket.yaw_angle == 0)//����
		{
			*error = 0;
		}
//	}

}

/**
  * @brief  ��ȡpitch���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void Vision_Error_Angle_Pitch(float *error,bool_t which_gimbal)
{	
//	if(which_gimbal==GUP)
//	{
//		*error = -(VisionRecvData_Up.pitch_angle + 0.0f)/ 360.0f * 6.28f ;
//		//���˴���ֵΪ0�����ǳ������жԾ�����в����Ĵ���ġ�
//		if(VisionRecvData_Up.pitch_angle == 0)
//		{
//			*error = 0;
//		}
//	}
//	else if(which_gimbal==GDOWN)
//	{
		*error =(vision_info.RxPacket.pitch_angle - 0.0f)/ 360.0f * 6.28f ;//- 18.0f
		//���˴���ֵΪ0�����ǳ������жԾ�����в����Ĵ���ġ�
		if(vision_info.RxPacket.pitch_angle == 0)
		{
			*error = 0;
		}
//	}
//	
}


/**
  * @brief  ��ȡ����  ��λM
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
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool Vision_Up_If_Update(void)
{
	return Vision_Up_Get_New_Data;
}

//�ж��Ӿ�������
bool Vision_Down_If_Update(void)
{
	return vision_info.State.rx_data_update;
} 

/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Up_Clean_Update_Flag(void)
{
	Vision_Up_Get_New_Data = FALSE;
}
void Vision_Down_Clean_Update_Flag(void)
{
	vision_info.State.rx_data_update = false;
}

