/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "judge.h"
#include "remote_control.h"
#include "gimbal_task.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);


static void chassis_order_transmit(chassis_move_t *chassis_order);
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//�����˶�����
chassis_move_t chassis_move;

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
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //���̳�ʼ��
    chassis_init(&chassis_move);
 
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);


    while (1)
    {
        //set chassis control mode
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //���̿���������
        chassis_set_contorl(&chassis_move);
		    //���������
       chassis_order_transmit(&chassis_move);
         
				if (IF_KEY_PRESSED_B == 1 && chassis_move.gimbal_order.move_enable ==0 )
				{
				chassis_move.gimbal_order.move_enable =0;
				chassis_move.gimbal_order.reset_enable =1;
				HAL_NVIC_SystemReset();
				
				}
				else
				{
					chassis_move.gimbal_order.reset_enable = 0;

				}
	
        //os delay
        //ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
     
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
   }
    
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis angle PID
    //���̽Ƕ�pidֵ

    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //in beginning�� chassis mode is raw 
    //���̿���״̬Ϊԭʼ
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_ZERO_FORCE;
    //get remote control point
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //��ȡ��̨�������ָ��
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    
    //initialize angle PID
    //��ʼ���Ƕ�PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //first order low-pass filter  replace ramp function
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //max and min speed
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //change to follow gimbal angle mode
    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
			//��Ϊ��̨ģʽ��ԽǶȱ�Ϊ0
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
			  chassis_move_transit->Chassis_Mode_Change_Sign = 1;
    }
		
		if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
		{
		    chassis_move_transit->Chassis_Mode_Change_Sign = 1;
		}
		if ((chassis_move_transit->last_chassis_mode != CHASSIS_SPIN_MODE) && chassis_move_transit->chassis_mode == CHASSIS_SPIN_MODE)
		{
		    chassis_move_transit->Chassis_Mode_Change_Sign = 1;
		}
		
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

   //���������̬�Ƕ�, �����������������������ⲿ�ִ���
	 //��ԽǶ��趨ֵ����ԽǶ�
    chassis_move_update->chassis_relative_angle = chassis_move_update->chassis_yaw_motor->relative_angle;
		//��ȡ���̹������ֵ
		chassis_move_update->gimbal_order.chassis_heat_MAX = JUDGE_usGetChassisPowerLimit();
		//��ȡ˲ʱ����
		chassis_move_update->gimbal_order.Instantaneous_power = JUDGE_fGetChassisPower();
		if(chassis_move_update->gimbal_order.Instantaneous_power>=50)
			chassis_move_update->gimbal_order.Instantaneous_power -= 5; 
		
		//��ȡʣ�ཹ������
    chassis_move_update->gimbal_order.residual_joule_energy = JUDGE_fGetRemainEnergy();
}
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
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel *  CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
    //���̿��ƣ����ü��̿ո���ʱ��ֱ�Ӹ����ٶ����ֵ
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
				//minΪ������ٶ�
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
				vy_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����,��ֹ���ڼ��̿��ƣ������µĿ���ֵͻ��
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
		//�����ٶ�����
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //get three control set-point, ��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
		
			//����
			 if (gimbal_control.auto_mode ==1)		 
			{
			chassis_move_control->vx_set = 0;
			chassis_move_control->vy_set = 0;
			//set control relative angle  set-point
			//���ÿ��������̨�Ƕ�
			chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
			//calculate ratation speed
			//������תPID���ٶ�
			chassis_move_control->wz_set = 0;
			//speed limit
			//�ٶ��޷�
			chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
			chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			
			}
      else
			{
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        //set control relative angle  set-point
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //������תPID���ٶ�
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
      }
			
		}
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {	//����
			 if (gimbal_control.auto_mode ==1)		 
			{
			chassis_move_control->vx_set = 0;
			chassis_move_control->vy_set = 0;
			//set control relative angle  set-point
			//���ÿ��������̨�Ƕ�
			chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
			//calculate ratation speed
			//������תPID���ٶ�
			chassis_move_control->wz_set = 0;
			//speed limit
			//�ٶ��޷�
			chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
			chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			
			}
			else
			{
			  chassis_move_control->vx_set = vy_set;
        chassis_move_control->vy_set = vx_set;
				chassis_move_control->chassis_relative_angle_set = rad_format(angle_set-chassis_move_control->chassis_relative_angle);
        chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0, chassis_move_control->chassis_relative_angle_set );
        chassis_move_control->vx_set = fp32_constrain(vy_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vx_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			}
    }
		else if (chassis_move_control->chassis_mode == CHASSIS_SPIN_MODE)
    {	//����
			 if (gimbal_control.auto_mode ==1)		 
			{
			chassis_move_control->vx_set = 0;
			chassis_move_control->vy_set = 0;
			//set control relative angle  set-point
			//���ÿ��������̨�Ƕ�
			chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
			//calculate ratation speed
			//������תPID���ٶ�
			chassis_move_control->wz_set = 2;
			//speed limit
			//�ٶ��޷�
			chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
			chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			
			}
			else
			{
			  chassis_move_control->vx_set = vy_set;
        chassis_move_control->vy_set = vx_set;
				chassis_move_control->chassis_relative_angle_set = rad_format(angle_set-chassis_move_control->chassis_relative_angle);
        chassis_move_control->wz_set = 2;
        chassis_move_control->vx_set = fp32_constrain(vy_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vx_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
			}
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_ZERO_FORCE)
    {
        chassis_move_control->wz_set = 0.0f;
        chassis_move_control->vx_set = 0.0f;
        chassis_move_control->vy_set = 0.0f;
    }
		
			chassis_move_control->gimbal_order.vx_set = -chassis_move_control->vx_set;
			chassis_move_control->gimbal_order.wz_set = -chassis_move_control->wz_set;
}

static void chassis_order_transmit(chassis_move_t *chassis_order)
{
	CAN_cmd_gimbal_command_0x712(chassis_order->gimbal_order);
	HAL_Delay(1);
	CAN_cmd_gimbal_command_0x710(chassis_order->gimbal_order);
  HAL_Delay(1);
	CAN_cmd_gimbal_command_0x711(chassis_order->gimbal_order);
}



//bool_t V_key_long_press_detection(void)
//{
//	int V_press_time;
//	bool_t V_long_press_long;
//	if(IF_KEY_PRESSED_V== 1)
//	{
//   V_press_time++;
//	}
//	else
//	{
//	V_press_time = 0;
//	}

//	if(V_press_time >= 1000)
//	{
//	V_long_press_long =! V_long_press_long;
//	}
//	if(V_long_press_long == 1)
//		return 1;
//	else
//	 return 0;
//	
//}
