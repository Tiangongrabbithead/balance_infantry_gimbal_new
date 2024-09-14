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
/*--------------------------------�ݶ�Э��-------------------------------------*/

#define    VISION_LENGTH        22     		 //�ݶ�22�ֽ�,ͷ3�ֽ�,����17�ֽ�,β2�ֽ�

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    VISION_SOF_UP         (0xA5)
#define    VISION_SOF_DOWN         (0xB5)

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    VISION_LEN_HEADER    3         //֡ͷ��
#define    VISION_LEN_DATA      59       //���ݶγ���,���Զ���
#define    VISION_LEN_TAIL      2	      //֡βCRC16
#define    VISION_LEN_PACKED   64       //���ݰ�����,���Զ���

#define GUP 0
#define GDOWN 1

#define NOW  0
#define LAST 1

#define VisionBufferLength_SH       150

//�������պͷ���ָ������бȽ�,���պͷ���ָ������ͬʱ,���ж�Ϊ���ݿ���

//֡ͷ��CRC8У��,��֤���͵�ָ������ȷ��

//PC�շ���STM32�շ��ɾ����ϵ,���½ṹ��������STM32,PC�������޸�

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
}extVisionSendHeader_t;


//STM32����,ֱ�ӽ����ڽ��յ������ݿ������ṹ��
typedef __packed struct
{
	//3+3*4+5+42+2=64
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��

	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	uint8_t   is_switched;
	uint8_t   is_findtarget;
	uint8_t   is_spinning;
	uint8_t   is_shooting;
	uint8_t   is_middle;
	//uint8_t	  auto_too_close;   //Ŀ�����̫��,�Ӿ���1������0
	
	 uint8_t  Vision_empty_time[42];
	/* β */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;


//STM32����,ֱ�ӽ�����õ�����һ���ֽ�һ���ֽڵط��ͳ�ȥ
typedef struct
{
//	/* ͷ */
//	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
//	uint8_t   CmdID;		//ָ��
//	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
//	float     pitch_angle;
//	float     yaw_angle;
//	float     distance;			//����
//	uint8_t   lock_sentry;		//�Ƿ���̧ͷʶ���ڱ�
//	uint8_t   base;				//����
//	
//	uint8_t   blank_a;		//Ԥ��
//	uint8_t	  blank_b;
//	uint8_t	  blank_c;	
	//��ȥ֡ͷ3λ һ��61λ  11*4+15+2
	float    INS_quat_send[4];
	float    INS_gyro_send[3];
	float    INS_accel_send[3];
	float		 pitch_revolver_angle;
	float		 yaw_relative_angle;
	float    Bullet_speed[1];
	
  uint8_t  Vision_empty_time[15];
	/* β */
	uint16_t  CRC16;
	
}extVisionSendData_t;


/**
 *	@brief	�Ӿ�ģʽ
 */
typedef enum
{
	VISION_MODE_MANUAL		= 0,	// �ֶ�ģʽ
	VISION_MODE_AUTO		= 1,	// ����ģʽ
	VISION_MODE_BIG_BUFF	= 2,	// ����ģʽ
	VISION_MODE_SMALL_BUFF	= 3,	// ��С��ģʽ
} Vision_Mode_t;
/* ������ʶ���� */
typedef struct
{
	uint8_t 		my_color;			// ��0/1��ʾ��ɫ
	Vision_Mode_t	mode;				// �Ӿ�ģʽ
	uint8_t  		rx_data_valid;		// �������ݵ���ȷ��
	uint16_t 		rx_err_cnt;			// �������ݵĴ���ͳ��
	uint32_t		rx_cnt;				// �������ݰ���ͳ��
	bool		    rx_data_update;		// ���������Ƿ����
	uint32_t 		rx_time_prev;		// �������ݵ�ǰһʱ��
	uint32_t 		rx_time_now;		// �������ݵĵ�ǰʱ��
	uint16_t 		rx_time_fps;		// ֡��
	bool            predict_state;       //�Ƿ���Ԥ��

} Vision_State_t;



/* �Ӿ�ͨ�����ݰ���ʽ */
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
//��ȡyaw���Ƕ�
extern void Vision_Error_Angle_Yaw(float *error,bool_t which_gimbal);
//��ȡpitch���Ƕ�
void Vision_Error_Angle_Pitch(float *error,bool_t which_gimbal);
//��ȡ����
void Vision_Get_Distance(float *distance,bool_t which_gimbal);

//�ж��Ӿ����ݸ�������
bool Vision_Down_If_Update(void);
//�Ӿ����ݸ��±�־λ�ֶ���0(false)
void Vision_Down_Clean_Update_Flag(void);
extern void Vision_Data_Update(void);
extern void Vision_init(void);
#ifdef __cplusplus
}
#endif

#endif 
