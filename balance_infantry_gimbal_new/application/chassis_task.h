/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "gimbal_task.h"
#include "chassis_task.h"

//in the beginning of task ,wait a time
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

//the channel of choosing chassis mode,
//ѡ�����״̬ ����ͨ����
#define CHASSIS_MODE_CHANNEL 0
//rocker value (max 660) change to vertial speed (m/s) 
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.050f
//rocker value (max 660) change to horizontal speed (m/s)
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.030f

#define CHASSIS_WZ_SPAIN 10.0f
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define CHASSIS_CONTROL_HALF_TIME_MS 1
//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//press the key, chassis will swing
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D




//chassis forward or back max speed
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 25.0f
//chassis left or right max speed
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 25.0f


//chassis follow angle PID
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.00002f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 3.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 8.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow yaw gimbal motor relative angle.���̻������̨��ԽǶ�
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. ��������ת�ٶȿ���
  CHASSIS_VECTOR_ZERO_FORCE,          //����ģʽ
	CHASSIS_SPIN_MODE,

} chassis_mode_e;
typedef enum
{
  CHASSIS_ZERO_FORCE,                   //chassis will be like no power,��������, ��û�ϵ�����
  CHASSIS_FOLLOW_GIMBAL_YAW,   //chassis will follow gimbal, usually in infantry,�����������̸�����̨  
  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed,���̲�����Ƕȣ��Ƕ��ǿ����ģ������������ٶȻ�
	CHASSIS_SPIN,
} chassis_behaviour_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;


typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  chassis_behaviour_e chassis_behaviour_mode;
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  const gimbal_motor_t *chassis_yaw_motor;           //��ȡyaw����ṹ��ָ��
  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid
  Gimbal_order_t gimbal_order;                //�������̵�����

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //the set relative angle.���������̨���ƽǶ�             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  bool_t  Chassis_Mode_Change_Sign;
} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern void GIMBAL_absolute_angle_NOlimit(gimbal_motor_t *gimbal_motor, fp32 add);
extern chassis_move_t chassis_move;

#endif
