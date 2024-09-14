#include "Vision_Task.h"
#include "Gimbal_Task.h"
#include "vision.h"
#include "bsp_usart.h"
#include "usart.h"
//#include "judge.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"


Vision_process_t Vision_process;
//float vision_mach_yaw,vision_mach_pitch,vision_dis_meter;//�Ӿ�����ת��

extern uint8_t usart6_dma_rxbuf[200];

float YawTarget_now,PitchTarget_now;//ʵ���Ӿ������Ƕ�
float update_cloud_yaw = 0,update_cloud_pitch=0;	/*��¼�Ӿ���������ʱ����̨���ݣ����´ν�����*/
float lastupdate_cloud_yaw = 0,lastupdate_cloud_pitch=0; /*ǰ��֡������*/
static void Vision_Normal(void);
static void Vision_Pridict(void);
static void AntiNormal(void) ;
//static void Anti_Target(void);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t Info_updateTaskStack;
#endif

void Vision_Task(void const * argumt)
{
	Vision_process.speed_queue.queueLength = 60;
	Vision_process.accel_queue.queueLength = 60;
	Vision_process.dis_queue.queueLength = 60;
	Vision_init();

    while (1) 
    {
		Vision_Data_Update();
		Vision_Send_Data(1);
		Vision_Normal();
		if(vision_info.State.predict_state)
		{
			Vision_Pridict();
		}
		//����С���ݾ����տ�ʱʹ��
//		Anti_Target();
		AntiNormal();
	
		vTaskDelay(2);			//2ms

		
#if INCLUDE_uxTaskGetStackHighWaterMark
        Info_updateTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


////static void Offset_Angle_Get_2_1()
////{
////    if(Vision_process.data_kal.DistanceGet_KF > 0.5f && Vision_process.data_kal.DistanceGet_KF <= 3.5f)
////        Vision_process.offset_pitch = 0.7954f * Vision_process.data_kal.DistanceGet_KF - 0.1428f + 1.f;
////    else if(Vision_process.data_kal.DistanceGet_KF > 3.5f &&  Vision_process.data_kal.DistanceGet_KF <= 7.f)// <= 6.25f
////        Vision_process.offset_pitch = 0.1827f * Vision_process.data_kal.DistanceGet_KF + 2.0095f + 1.f;//���һ�����ֶ�����1.3
////    else
////        Vision_process.offset_pitch = 0;

////    Vision_process.offset_yaw = 0.9f;//-0.6

//////	    Vision_process.offset_yaw = 0;
//////	  Vision_process.offset_pitch = 0;
////}

static void Vision_Normal()
{
//    static uint16_t active_cnt=0,lost_cnt=0;/*�������/��ʧ����--����ʶ���δʶ���໥�л��Ĺ���*/
	if(vision_info.State.rx_data_update)
	{
		if(vision_info.RxPacket.is_spinning == 1)
		{
			Vision_process.gyro_anti = true;
		}else
		{
			Vision_process.gyro_anti = false;
		}
		  Vision_Error_Angle_Yaw(&(gimbal_control.gimbal_kalman.Auto_Error_Yaw[NOW]),GDOWN);
		  Vision_Error_Angle_Pitch(&(gimbal_control.gimbal_kalman.Auto_Error_Pitch[NOW]),GDOWN);
		  Vision_Get_Distance(&(gimbal_control.gimbal_kalman.Auto_Distance),GDOWN);
		
		  Vision_process.data_kal.YawGet_KF = KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Error_Vis_Kalman,gimbal_control.gimbal_kalman.Auto_Error_Yaw[NOW]); 	/*���Ӿ��Ƕ��������������˲�*/
		  Vision_process.data_kal.PitchGet_KF = KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Error_Vis_Kalman,gimbal_control.gimbal_kalman.Auto_Error_Pitch[NOW]);
		  Vision_process.data_kal.DistanceGet_KF =KalmanFilter(&gimbal_control.gimbal_kalman.Vision_Distance_Kalman,gimbal_control.gimbal_kalman.Auto_Distance);
		
		  YawTarget_now=lastupdate_cloud_yaw+Vision_process.data_kal.YawGet_KF;
      PitchTarget_now=lastupdate_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

//		YawTarget_now=update_cloud_yaw+Vision_process.data_kal.YawGet_KF;
//        PitchTarget_now=update_cloud_pitch+Vision_process.data_kal.PitchGet_KF;

          lastupdate_cloud_yaw = update_cloud_yaw;
          lastupdate_cloud_pitch = update_cloud_pitch;      /*��¼ǰ2֡������*/
          update_cloud_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;/*�Ӿ����ݸ���ʱ����̨�Ƕ�*/
          update_cloud_pitch =  gimbal_control.gimbal_pitch_motor.absolute_angle;
		
		 /*�Ӿ���������.......................................*/
        Vision_process.speed_get = Get_Diff(3,&Vision_process.speed_queue,YawTarget_now);//20
        Vision_process.speed_get = 48 * (Vision_process.speed_get/vision_info.State.rx_time_fps); //ÿ����
        Vision_process.speed_get = KalmanFilter(&gimbal_control.gimbal_kalman.Gimbal_Yaw_Gyro_Kalman,Vision_process.speed_get);
//      Vision_process.speed_get = DeathZoom(Vision_process.speed_get,0,1);
        Vision_process.speed_get = fp32_constrain(Vision_process.speed_get, -0.030, 0.030);

        Vision_process.accel_get = Get_Diff(5,&Vision_process.accel_queue,Vision_process.speed_get);	 /*�°��ȡ���ٶ�10*/
        Vision_process.accel_get = 25 * (Vision_process.accel_get/vision_info.State.rx_time_fps);//ÿ����
        Vision_process.accel_get = KalmanFilter(&gimbal_control.gimbal_kalman.Gimbal_Yaw_Accle_Kalman,Vision_process.accel_get);
//      Vision_process.accel_get = DeathZoom(Vision_process.accel_get,0,0.1);		/*�������� - �˳�0�㸽��������*/
        Vision_process.accel_get = fp32_constrain(Vision_process.accel_get, -0.023, 0.023);

        Vision_process.distend_get =  Get_Diff(5,&Vision_process.dis_queue,Vision_process.data_kal.DistanceGet_KF);
		
		vision_info.State.rx_data_update = false;
	}
}

static void Vision_Pridict()
{
    static float acc_use = 1.0f;
    static float predic_use = 0.6f;//  3 ,10
    float dir_factor;
    if( (Vision_process.speed_get * Vision_process.accel_get)>=0 )
    {
        dir_factor= 1.0f;//1  2
    }
    else
    {
        dir_factor= 2.0f;//1.5  4
    }

    Vision_process.feedforwaurd_angle = acc_use * Vision_process.accel_get; 	/*����ǰ����*/

    Vision_process.predict_angle = predic_use * (1.1f*Vision_process.speed_get*Vision_process.data_kal.DistanceGet_KF+1.5f*dir_factor*Vision_process.feedforwaurd_angle*Vision_process.data_kal.DistanceGet_KF) ;//�ٶ�1.1�����ٶ�3
    Vision_process.predict_angle = fp32_constrain(Vision_process.predict_angle, -0.092, 0.092);
	
//--------------------------------------------------------------------------------------//	
//������ֹװ�װ�ɿ��Ǵ��ײ���
//	Vision_process.predict_angle = chassis_move.motor_chassis.speed * 1.0f ;
	
}
static void AntiNormal()
{
    /*ֱ�Ӹ������.................................*/
	    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,YawTarget_now);
//	    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,PitchTarget_now);
    /*..................................................*/

//    Vision_process.data_kal.YawTarget_KF=YawTarget_now+Vision_process.predict_angle;
//    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,Vision_process.data_kal.YawTarget_KF);
	
//    Vision_process.data_kal.PitchTarget_KF=PitchTarget_now+10*Vision_process.distend_get;                               //�������pitch��Ԥ���ϵ��
//    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&kalman_targetPitch,Vision_process.data_kal.PitchTarget_KF);
    Vision_process.data_kal.PitchTarget_KF= KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);//����Ԥ��
}

//static void AntiGyro()
//{
//	/*ֱ�Ӹ������.................................*/
//    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,YawTarget_now);
//    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);
//    /*..................................................*/
//    Vision_process.data_kal.YawTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Yaw_Set_Gim_Kalman,Vision_process.data_kal.YawTarget_KF);
//    Vision_process.data_kal.PitchTarget_KF=KalmanFilter(&gimbal_control.gimbal_kalman.Pitch_Set_Gim_Kalman,PitchTarget_now);//����Ԥ��
//}

//static void Anti_Target()
//{
//    if(Vision_process.gyro_anti)
//        AntiGyro();  
//    else
//        AntiNormal();
//}

