  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ����ң������ֵ������������Ϊ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a chassis behaviour mode
    1. in chassis_behaviour.h , add a new behaviour name in chassis_behaviour
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // new add
    }chassis_behaviour_e,
    2. implement new function. chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx, vy, wz" param is chassis movement contorl input. 
        first param: 'vx' usually means  vertical speed,
            positive value means forward speed, negative value means backward speed.
        second param: 'vy' usually means horizotal speed,
            positive value means letf speed, negative value means right speed
        third param: 'wz' can be rotation speed set or angle set, 

        in this new function, you can assign speed to "vx","vy",and "wz",as your wish
    3.  in "chassis_behaviour_mode_set" function, add new logical judgement to assign CHASSIS_XXX_XXX to  "chassis_behaviour_mode" variable,
        and in the last of the function, add "else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" 
        choose a chassis control mode.
        four mode:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control relative angle
            between chassis and gimbal. you can name third param to 'xxx_angle_set' other than 'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control absolute angle calculated by gyro
            you can name third param to 'xxx_angle_set.
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy' are speed control, 'wz' is rotation speed control.
        CHASSIS_VECTOR_RAW : will use 'vx' 'vy' and 'wz'  to linearly calculate four wheel current set, 
            current set will be derectly sent to can bus.
    4. in the last of "chassis_behaviour_control_set" function, add
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }

        
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // ����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" ������������
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "judge.h"


/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);




/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
	bool_t rc_key_loge = 0;

bool_t rc_x_model;
bool_t rc_x_laxt_model;
bool_t model_change;
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
//	int rc_time;
    if (chassis_move_mode == NULL)
    {
        return;
    }

//if(rc_key_loge==0)
//{
//		if(IF_KEY_PRESSED_X)
//		{
//		rc_time++;
//			if(rc_time>=5)
//			{
//			rc_key_loge=1;
//			}
//		}
//		else
//		{
//		   rc_time=0;
//		}
//}
//else if(rc_key_loge == 1)
//	{
//	
//		if(IF_KEY_PRESSED_X)
//		{
//		rc_time++;
//			if(rc_time>=5)
//			{
//			rc_key_loge=0;
//			}
//		}
//		else
//		{
//		   rc_time=0;
//		}
//		
//	}


if(IF_KEY_PRESSED_X)
{
rc_x_model =1;
	
}
else
{
	rc_x_model =0;
	rc_x_laxt_model =  rc_x_model;
}

if (rc_x_laxt_model == 0 &&  rc_x_model==1 )
{
  model_change =! model_change;
	rc_x_laxt_model =  rc_x_model;
}



    //remote control  set chassis behaviour mode
    //ң��������ģʽ
		//���м������̨
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
    chassis_move_mode->chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
		chassis_move_mode->gimbal_order.move_enable = 1;
    }
		//��������
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
    chassis_move_mode->chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
		chassis_move_mode->gimbal_order.move_enable = 0;
    }
		//���ϲ�������̨
    else if ((switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]) || model_change==1) && chassis_move_mode->gimbal_order.move_enable == 1)
    {
    chassis_move_mode->chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
		chassis_move_mode->gimbal_order.move_enable = 1;
    }
		else if (IF_KEY_PRESSED_SHIFT)
		{
		chassis_move_mode->chassis_behaviour_mode = CHASSIS_SPIN;
		chassis_move_mode->gimbal_order.move_enable = 1;
		}
    //when gimbal in some mode, such as init mode, chassis must's move
    //����̨��ĳЩģʽ�£����ʼ���� ���̲���
    if (gimbal_cmd_to_chassis_stop())//|| JUDGE_usGetChassisPowerLimit() == 0
    {
    chassis_move_mode->chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
		chassis_move_mode->gimbal_order.move_enable = 0;
		
    }


    //add your own logic to enter the new mode
    //����Լ����߼��жϽ�����ģʽ


    //accord to beheviour mode, choose chassis control mode
    //������Ϊģʽѡ��һ�����̿���ģʽ
    if (chassis_move_mode->chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_ZERO_FORCE; 
    }
    else if (chassis_move_mode->chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_move_mode->chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)//�������ǲ���
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
		else if (chassis_move_mode->chassis_behaviour_mode == CHASSIS_SPIN)
		{
				 chassis_move_mode->chassis_mode = CHASSIS_SPIN_MODE;
		}
		
		
		
    
}


/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_move_rc_to_vector->chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		//������̨ģʽ�벻������̨ģʽ������ͬ
    else if (chassis_move_rc_to_vector->chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_move_rc_to_vector->chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		else if (chassis_move_rc_to_vector->chassis_behaviour_mode == CHASSIS_SPIN)
		{
		
		
		}
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}




/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = 0;
    static uint8_t swing_flag = 0;

    //judge if swing
    //�ж��Ƿ�Ҫҡ��
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
        }
    }
    else
    {
        swing_flag = 0;
    }

   
}



/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = -PI/2;
    static uint8_t swing_flag = 0;

    //judge if swing
    //�ж��Ƿ�Ҫҡ��
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
        }
    }
    else
    {
        swing_flag = 0;
    }
}
/**
  * @brief          ����С����ģʽ
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
		vx_set = 0;
		vy_set = 0;
    *angle_set = -PI/2;
    static uint8_t swing_flag = 0;
		
    //judge if swing
    //�ж��Ƿ�Ҫҡ��
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
        }
    }
    else
    {
        swing_flag = 0;
    }
}