/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "pid.h"
#include "ladrc_feedforward.h"
#include "Vision_Task.h"
#include "vision.h"
#include "chassis_task.h"


//motor enconde value format, range[0-8191]
//�������ֵ���� 0��8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

		#define RC_CALI_VALUE_HOLE          600 

		int i;
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          limit angle set in GIMBAL_MOTOR_GYRO mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��GIMBAL_MOTOR_GYROģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

static void gimbal_absolute_angle_limit1(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��GIMBAL_MOTOR_ENCONDEģʽ�����ƽǶ��趨,��ֹ�������
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @brief          ��̨�Ƕ�PID��ʼ��, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      maxout: pid������
  * @param[in]      intergral_limit: pid���������
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
//static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
//static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
/**
  * @brief          ��̨�Ƕ�PID����, ��Ϊ�Ƕȷ�Χ��(-pi,pi)��������PID.c��PID
  * @param[out]     pid:��̨PIDָ��
  * @param[in]      get: �Ƕȷ���
  * @param[in]      set: �Ƕ��趨
  * @param[in]      error_delta: ���ٶ�
  * @retval         pid ���
  */
//static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);


bool_t rc_vision(gimbal_control_t* Gimbal_Control_rc);
void GIMBAL_AUTO_Mode_Ctrl_aimbot(gimbal_control_t* gimbal_auto_control_aimbot);
 extern chassis_move_t chassis_move;
bool_t not_follow_gimbal_loge(chassis_move_t* model);




#if GIMBAL_TEST_MODE
//j-scope ����pid����
static void J_scope_gimbal_test(void);
#endif




//gimbal control data
//��̨���������������
gimbal_control_t gimbal_control;

//motor current 
//���͵ĵ������

static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
static uint16_t  attack_switch_ramp = ATTACK_SWITCH_TIME;	

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          ��̨���񣬼�� GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */

void gimbal_task(void const *pvParameters)
{
    //�ȴ������������������������
    //wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //gimbal init
    //��̨��ʼ��
    gimbal_init(&gimbal_control);

    vTaskDelay(GIMBAL_CONTROL_TIME);
    gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���


    while (1)
    {
				i++;
        gimbal_set_mode(&gimbal_control);                    //������̨����ģʽ
        gimbal_mode_change_control_transit(&gimbal_control); //����ģʽ�л� �������ݹ���
        gimbal_feedback_update(&gimbal_control);             //��̨���ݷ���
        gimbal_set_control(&gimbal_control);                 //������̨������
        gimbal_control_loop(&gimbal_control);                //��̨����PID����

        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
				if(yaw_can_set_current> 30000)
				{
				yaw_can_set_current = 30000;
				}
				 if(yaw_can_set_current < -30000)
				{
				yaw_can_set_current =-30000;
				}
				
				
				if(pitch_can_set_current > 30000)
				{
				pitch_can_set_current=30000;
				}
			 if(pitch_can_set_current < -30000)
				{
				pitch_can_set_current = -30000;
				}
				
        CAN_cmd_gimbal_can2(yaw_can_set_current, 0, 0, 0);
        CAN_cmd_gimbal(0,0,pitch_can_set_current, 0);
//       

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        vTaskDelay(2);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  * @param[in]      yaw_offse:yaw ��ֵ
  * @param[in]      pitch_offset:pitch ��ֵ
  * @param[in]      max_yaw:max_yaw:yaw �����ԽǶ�
  * @param[in]      min_yaw:yaw ��С��ԽǶ�
  * @param[in]      max_yaw:pitch �����ԽǶ�
  * @param[in]      min_yaw:pitch ��С��ԽǶ�
  * @retval         ���ؿ�
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}
void set_cali_gimbal_hook1(const uint16_t yaw_offset1, const uint16_t pitch_offset1, const fp32 max_yaw1, const fp32 min_yaw1, const fp32 max_pitch1, const fp32 min_pitch1)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset1;
    gimbal_control.gimbal_yaw_motor.max_relative_angle1 = max_yaw1;
    gimbal_control.gimbal_yaw_motor.min_relative_angle1 = min_yaw1;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset1;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch1;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch1;
}

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
	//�������˲�
	KalmanCreate(&(init->gimbal_kalman.Yaw_Error_Vis_Kalman),    1,  0);
	KalmanCreate(&(init->gimbal_kalman.Pitch_Error_Vis_Kalman),  1,  0);
	KalmanCreate(&(init->gimbal_kalman.Yaw_Set_Gim_Kalman),      1, 400);
	KalmanCreate(&(init->gimbal_kalman.Pitch_Set_Gim_Kalman),    1, 100);//
	KalmanCreate(&(init->gimbal_kalman.Vision_Distance_Kalman),  1, 10);
	KalmanCreate(&(init->gimbal_kalman.Gimbal_Yaw_Accle_Kalman), 1, 10);
	KalmanCreate(&(init->gimbal_kalman.Gimbal_Yaw_Gyro_Kalman),  1, 100);
//����ģʽ��
    set_cali_gimbal_hook(YAW_OFFSET,PITCH_OFFSET,YAW_MAX,-YAW_MAX,PITCH_MAX,-PITCH_MAX);
//����Ե��еĽǶ�����
    set_cali_gimbal_hook1(YAW_OFFSET,PITCH_OFFSET,-1.1,-1.9,PITCH_MAX,-PITCH_MAX);
//    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
//    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //�������ָ���ȡ
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //����������ָ���ȡ
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    //ң��������ָ���ȡ
    init->gimbal_rc_ctrl = get_remote_control_point();
    //��ʼ�����ģʽ
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE;
//    //��ʼ��yaw���pid
//    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
//    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
//    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
//    //��ʼ��pitch���pid
//    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
//    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
//    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

//    //�������PID
//    gimbal_total_pid_clear(init);

//    gimbal_feedback_update(init);

//    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
//    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
//    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;


//    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
//    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
//    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
 
		//��ʼ��Yaw���ADRSMC����
		LADRC_FDW_init(&init->gimbal_yaw_motor.ladrc,30,0.005,120,29000,50,0);
		//��ʼ��Pitch���ADRSMC��ģ����
		LADRC_FDW_init(&init->gimbal_pitch_motor.ladrc,35,0.007,180,29000,40,0);

}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    feedback_update->gimbal_pitch_motor.absolute_angle = -*(feedback_update->gimbal_INT_angle_point + 1);
    feedback_update->gimbal_pitch_motor.relative_angle =motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
		
		
//		feedback_update->gimbal_pitch_motor.relative_angle1 = feedback_update->gimbal_INT_angle_point+2;
    feedback_update->gimbal_pitch_motor.motor_gyro = -*(feedback_update->gimbal_INT_gyro_point + 1);
		feedback_update->gimbal_pitch_motor.motor_speed = -0.1046*feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
	
//Q����Ϊyaw��΢����
		feedback_update->Q_mount = 0.000008f*(feedback_update->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q);

    feedback_update->gimbal_yaw_motor.absolute_angle = +*(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.relative_angle =+motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
    feedback_update->gimbal_yaw_motor.motor_gyro = (arm_cos_f32(feedback_update->gimbal_pitch_motor.absolute_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        + arm_sin_f32(feedback_update->gimbal_pitch_motor.absolute_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET)));
    feedback_update->gimbal_yaw_motor.motor_speed = -0.1046*feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
	

}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     gimbal_mode_change:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
		

    //yaw���״̬���л���������
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ZERO_FORCE && gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
		else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
		{
				gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = 0;
		}
	
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;
	

    //pitch���״̬���л���������
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ZERO_FORCE && gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }
	else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
	{
	    gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = 0;
	}

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
		


//����ģʽ�ж�
			if((set_control->gimbal_rc_ctrl->mouse.press_r == 1 || rc_vision(set_control) || V_press_detection() == 1  ) && set_control->gimbal_behaviour !=GIMBAL_INIT )
		{
			gimbal_control.auto_mode=1;
			set_control->gimbal_sub_mode = GIMBAL_AUTO;
			set_control->gimbal_yaw_motor.absolute_angle_set = set_control->gimbal_yaw_motor.absolute_angle;
			set_control->gimbal_pitch_motor.absolute_angle_set =set_control->gimbal_pitch_motor.absolute_angle;// set_control->gimbal_pitch_motor.absolute_angle;
	
		}
		else 
		{
			gimbal_control.auto_mode=0;
			set_control->gimbal_sub_mode = GIMBAL_NORMAL;
			attack_switch_ramp = ATTACK_SWITCH_TIME;
			
			Clear_Queue(&Vision_process.speed_queue);
			Clear_Queue(&Vision_process.accel_queue);
			Clear_Queue(&Vision_process.dis_queue);
			/*���������Ϣ����ֹ��һ�������ܵ�Ӱ��*/
			Vision_process.data_kal.DistanceGet_KF = 0;//��ȷ���¾���
			Vision_process.feedforwaurd_angle = 0;
			Vision_process.predict_angle = 0;//��0Ԥ���
			vision_info.State.predict_state = false;			/*�ر�Ԥ��*/
			//ң�������̵�����
      gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
		}
		
		
		
		
		
		
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    //yaw���ģʽ����
   
		
		
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
		 if(chassis_move.chassis_mode == CHASSIS_SPIN)
		{
			GIMBAL_absolute_angle_NOlimit(&set_control->gimbal_yaw_motor, add_yaw_angle);
		}
		else
		{
			if(chassis_move.Chassis_Mode_Change_Sign == 0)
			{
			  if(not_follow_gimbal_loge(&chassis_move))
        {//gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit1(&set_control->gimbal_yaw_motor, add_yaw_angle);
				}
				else
				{
			  gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
				}
				
			}	
			else
			{
			GIMBAL_absolute_angle_NOlimit(&set_control->gimbal_yaw_motor, add_yaw_angle);
			chassis_move.Chassis_Mode_Change_Sign = 0;
			}				
				
		}	
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        
			
			  //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);

    }

    //pitch���ģʽ����
    
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
	//����ģʽ				
//		if (set_control->gimbal_behaviour == GIMBAL_ZERO_FORCE && set_control->gimbal_sub_mode == GIMBAL_AUTO)
//		{		
////		set_control->gimbal_yaw_motor.absolute_angle_set = set_control->gimbal_yaw_motor.absolute_angle;
////			set_control->gimbal_pitch_motor.absolute_angle_set = 0;
//				GIMBAL_AUTO_Mode_Ctrl_aimbot(set_control);
//				gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
//				gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
//		}
		 if (set_control->gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE && set_control->gimbal_sub_mode == GIMBAL_AUTO)
    {
			

			
        //gyroģʽ�£������ǽǶȿ���
				GIMBAL_AUTO_Mode_Ctrl_aimbot(set_control);
			
				if(not_follow_gimbal_loge(&chassis_move))
        {//gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit1(&set_control->gimbal_yaw_motor, add_yaw_angle);
				}
				else
				{
			  gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);

				}
				gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
			//gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);

    }
		
		
}




static void GIMBAL_absolute_angle_NOlimit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //now angle error
    //��ǰ�������Ƕ�
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //�����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            //calculate max add_angle
            //�����һ��������ӽǶȣ�
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}


//��̨���������ר��
static void gimbal_absolute_angle_limit1(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //now angle error
    //��ǰ�������Ƕ�
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //relative angle + angle error + add_angle > max_relative angle
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle1)
    {
        //�����������е�Ƕȿ��Ʒ���
        if (add > 0.0f)
        {
            //calculate max add_angle
            //�����һ��������ӽǶȣ�
            add = gimbal_motor->max_relative_angle1 - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle1)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle1 - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}


/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //�Ƿ񳬹���� ��Сֵ
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
//      gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

 
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
		if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE)
		{
			control_loop->gimbal_pitch_motor.given_current=0;
			control_loop->gimbal_yaw_motor.given_current=0;
		}
	
		 if (control_loop->gimbal_behaviour == GIMBAL_MOTOR_GYRO && control_loop->gimbal_sub_mode == GIMBAL_AUTO)
	{
			gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
			gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
	}
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{fp32 current_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
//    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
//    current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
		current_set =LADRC_FDW_calc(&gimbal_motor->ladrc,gimbal_motor->absolute_angle,gimbal_motor->absolute_angle_set,gimbal_motor->motor_gyro);

    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)current_set;
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{fp32 current_set;
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ����ٶȻ�����pid����
//    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
//    current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_speed, gimbal_motor->motor_gyro_set);
		current_set =LADRC_FDW_calc(&gimbal_motor->ladrc,gimbal_motor->relative_angle,gimbal_motor->relative_angle_set,gimbal_motor->motor_gyro);
    //����ֵ��ֵ
    gimbal_motor->given_current = (int16_t)current_set;
}




#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     gimbal_init:"gimbal_control"����ָ��.
  * @retval         none
  */
//static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
//{
//    if (pid == NULL)
//    {
//        return;
//    }
//    pid->kp = kp;
//    pid->ki = ki;
//    pid->kd = kd;

//    pid->err = 0.0f;
//    pid->get = 0.0f;

//    pid->max_iout = max_iout;
//    pid->max_out = maxout;
//}

//static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
//{
//    fp32 err;
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
//    pid->get = get;
//    pid->set = set;

//    err = set - get;
//    pid->err = rad_format(err);
//    pid->Pout = pid->kp * pid->err;
//    pid->Iout += pid->ki * pid->err;
//    pid->Dout = -pid->kd * error_delta;
//    abs_limit(&pid->Iout, pid->max_iout);
//    pid->out = pid->Pout + pid->Iout + pid->Dout;
//    abs_limit(&pid->out, pid->max_out);
//    return pid->out;
//}

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
//static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
//{
//    if (gimbal_pid_clear == NULL)
//    {
//        return;
//    }
//    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
//    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
//}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_control.gimbal_behaviour == GIMBAL_INIT || gimbal_control.gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_control.gimbal_behaviour == GIMBAL_INIT || gimbal_control.gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

bool_t rc_vision(gimbal_control_t* Gimbal_Control_rc)
{
	static int16_t rc1_vision_time;

	int  self_aiming_sign;
//		if (Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[0] > RC_CALI_VALUE_HOLE &&  \
//			  Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[1] > RC_CALI_VALUE_HOLE &&  \
//		    Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[2] < -RC_CALI_VALUE_HOLE && \
//		    Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[3] < -RC_CALI_VALUE_HOLE &&
    if( Gimbal_Control_rc->gimbal_rc_ctrl->rc.ch[0] ==-660)		\
//		     switch_is_down(Gimbal_Control_rc->gimbal_rc_ctrl->rc.s[0]))
		{
			rc1_vision_time++;
		}
		else
		{
		rc1_vision_time = 0;
		}
		
		
		
		if(rc1_vision_time >800)
    {
		self_aiming_sign=1;
		}
    else
		self_aiming_sign =0;
		if (self_aiming_sign == 1)
		return 1;
		else 
		return 0;
}








void GIMBAL_AUTO_Mode_Ctrl_aimbot(gimbal_control_t* gimbal_auto_control)
{
	if	(vision_info.RxPacket.distance == 0)
	{
		gimbal_auto_control->gimbal_pitch_motor.absolute_angle_set  =  gimbal_auto_control->gimbal_pitch_motor.absolute_angle;
		gimbal_auto_control->gimbal_yaw_motor.absolute_angle_set    =  gimbal_auto_control->gimbal_yaw_motor.absolute_angle;
	}
	else
	{
		if (vision_info.State.predict_state == true)
		{
			gimbal_auto_control->gimbal_pitch_motor.absolute_angle_set  =  Vision_process.data_kal.PitchTarget_KF;
			gimbal_auto_control->gimbal_yaw_motor.absolute_angle_set    =  Vision_process.data_kal.YawTarget_KF;
		}
		else
		{
			if(Vision_Down_If_Update() == 1)
			{
			gimbal_auto_control->gimbal_yaw_motor.absolute_angle_set =  Vision_process.data_kal.YawTarget_KF;
			gimbal_auto_control->gimbal_pitch_motor.absolute_angle_set =  Vision_process.data_kal.PitchTarget_KF;
      }
			else
			{
			gimbal_auto_control->gimbal_yaw_motor.absolute_angle_set =gimbal_auto_control->gimbal_pitch_motor.absolute_angle;
			gimbal_auto_control->gimbal_pitch_motor.absolute_angle_set = gimbal_auto_control->gimbal_yaw_motor.absolute_angle;
      }
			
			vision_info.State.predict_state = false;
       attack_switch_ramp--;
//        if( (abs(Vision_process.data_kal.PitchGet_KF)<=5)&&(abs(Vision_process.data_kal.YawGet_KF)<=20) )
//            sys.predict_state.PREDICT_OPEN = true;
        if(attack_switch_ramp == 0 )
            vision_info.State.predict_state = true;
		}
	}

}
bool_t not_follow_gimbal_loge(chassis_move_t* model)
{
if(model->chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
	return 1;
else
	return 0;
}

bool_t rc_V_model;
	bool_t rc_V_laxt_model;
	bool_t model_V_change;

bool_t V_press_detection(void)
{
//	bool_t rc_V_model;
//	bool_t rc_V_laxt_model;
//	bool_t model_V_change;
	
	if(IF_KEY_PRESSED_V)
{
rc_V_model =1;
	
}
else
{
  rc_V_model =0;
	rc_V_laxt_model =  rc_V_model;
}
if (rc_V_laxt_model == 0 &&  rc_V_model==1 )
{
  model_V_change =! model_V_change;
	rc_V_laxt_model =  rc_V_model;
}
if(model_V_change == 1)
	return 1;
else
	return 0;


}

bool_t RESET_press_detection(void)
{
	bool_t rc_shift_model;
	bool_t rc_shift_laxt_model;
	bool_t model_shift_change;
	
	if(IF_KEY_PRESSED_SHIFT)
{
rc_shift_model =1;
	
}
else
{
  rc_shift_model =0;
	rc_shift_laxt_model =  rc_shift_model;
}
if (rc_shift_laxt_model == 0 &&  rc_shift_model==1 )
{
  model_shift_change =! model_shift_change;
	rc_shift_laxt_model =  rc_shift_model;
}
if(model_shift_change == 1)
	return 1;
else
	return 0;


}


