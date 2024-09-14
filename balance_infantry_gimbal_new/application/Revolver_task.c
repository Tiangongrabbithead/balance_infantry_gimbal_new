#include "Revolver_task.h"
#include "cmsis_os.h"
#include "can.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "Gimbal_task.h"
#include "chassis_behaviour.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "arm_math.h"
#include "judge.h"
#include "tim.h"
#include "vision.h"
#include "stdint.h"
#include "stdlib.h"
#include "user_lib.h"
#include "detect_task.h"

Revolver_Control_t Revolver_control;
static void REVOLVER_Init(Revolver_Control_t *revolver_init);
static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback);
static void Revolver_control_loop(Revolver_Control_t *revolver_control);
static void REVOL_UpdateMotorAngleSum(Revolver_Control_t *revolver_sum);
static void Firc_Control_loop(Revolver_Control_t *fire_control);
static void REVOLVER_Rc_Switch(Revolver_Control_t *revolver_control);
int steering_mode=0;        //���ո�ģʽ 0Ϊ�رգ�1Ϊ��

int16_t Shoot_Can_Set_Current;

void Revolver_task(void const *pvParameters)
{

	vTaskDelay(REVOLVER_TASK_INIT_TIME);
	REVOLVER_Init(&Revolver_control);
	
	for(;;)
	{ 

		Revolver_Feedback_Update(&Revolver_control); 
		Firc_Control_loop(&Revolver_control);
		REVOLVER_Rc_Switch(&Revolver_control);
		Revolver_control_loop(&Revolver_control);
				if (toe_is_error(DBUS_TOE))
				{
				Revolver_control.Firc_L.give_current =0;
				Revolver_control.Firc_R.give_current =0;
				Shoot_Can_Set_Current =0;
				}
		
		CAN1_cmd_200(Revolver_control.Firc_L.give_current,Revolver_control.Firc_R.give_current,Shoot_Can_Set_Current,0);
//		CAN1_cmd_200(0,0,0,0);

  //CAN2_cmd_200(16000,-16000,Shoot_Can_Set_Current,0);
	//CAN2_cmd_200(0,0,Shoot_Can_Set_Current,0);
	   vTaskDelay(2);
	}
}



static void REVOLVER_Init(Revolver_Control_t *revolver_init)
{
		 //���ֵ����pidû�б�Ҫʹ����λ������
	 fp32 Revolver_speed_pid[3] = {REVOLVER_SPEED_PID_KP, REVOLVER_SPEED_PID_KI, REVOLVER_SPEED_PID_KD};
	 fp32 Revolver_position_pid[3] = {REVOLVER_POSITION_PID_KP, REVOLVER_POSITION_PID_KI, REVOLVER_POSITION_PID_KD};
	 const fp32 Firc_speed_pid[3] = {FIRC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FIRC_SPEED_PID_KD};
	 //��ʼ��PID
	 PID_init(&(revolver_init->revolver_motor_speed_pid), PID_POSITION, Revolver_speed_pid, 
	            REVOLVER_SPEED_PID_MAX_OUT, REVOLVER_SPEED_PID_MAX_IOUT);
	 PID_init(&(revolver_init->revolver_motor_position_pid), PID_POSITION, Revolver_position_pid, 
	            REVOLVER_POSITION_PID_MAX_OUT, REVOLVER_POSITION_PID_MAX_IOUT);
	 PID_init(&(revolver_init->Firc_R.firc_speed_pid), PID_POSITION, Firc_speed_pid, 
	            FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);
	 PID_init(&(revolver_init->Firc_L.firc_speed_pid), PID_POSITION, Firc_speed_pid, 
	            FIRC_SPEED_PID_MAX_OUT, FIRC_SPEED_PID_MAX_IOUT);

	 
   //ң����ָ��
   revolver_init->revolver_rc_ctrl=get_remote_control_point();
	 //���ָ��
	 revolver_init->revolver_motor_measure = get_trigger_motor_measure_point();
	 revolver_init->Firc_L.firc3508_motor_measure = get_motor_measure_point(0);
	 revolver_init->Firc_R.firc3508_motor_measure = get_motor_measure_point(1);
	 
	 revolver_init->actShoot=SHOOT_NORMAL;
	 revolver_init->Revolver_mode=REVOL_POSI_MODE;
	 revolver_init->revolver_buff_ramp = AN_BULLET/30;
	 Revolver_Feedback_Update(revolver_init);

}

static void Revolver_control_loop(Revolver_Control_t *revolver_control)
{

		static int16_t angle_out;
	  static uint32_t rc_keyZ_time=0;    							//����Z�İ���ʱ��
		static int16_t unfinish_time;
//		static bool_t if_stucked;

	  if(fabs(revolver_control->set_angle - revolver_control->angle)> AN_BULLET/2)
			{
				revolver_control->ifshoot_finsh = shoot_unfinish;
				unfinish_time++;
			}	
			else
			{
				unfinish_time = 0;
				revolver_control->ifshoot_finsh = shoot_finish;
				revolver_control->ifstuck_finish = stuck_finish;
			}
//		if(unfinish_time>500)
//		{
//			revolver_control->Revolver_mode = REVOL_REVERSAL_MODE;
//			revolver_control->ifstuck_finish = stuck_unfinish;
//			if_stucked = 1;
//		}
//		if(revolver_control->Revolver_mode == REVOL_REVERSAL_MODE&&revolver_control->last_Revolver_mode!=REVOL_REVERSAL_MODE)
//		{
//			revolver_control->set_angle += AN_BULLET*3/2;
//		}
		if(revolver_control->Revolver_mode == REVOL_POSI_MODE)
		{
			if(revolver_control->key_ShootFlag == shoot_ready&&revolver_control->ifshoot_finsh != shoot_unfinish)//&&abs(revolver_control->set_angle - revolver_control->last_angle)<1000)
			{		
					revolver_control->set_angle -= AN_BULLET;	
//				 if(if_stucked == 1)
//				{
//					revolver_control->set_angle-= AN_BULLET/2;
//					if_stucked = 0;
//				}
			}
		}
		//�������ֵ��������
		else if(revolver_control->Revolver_mode == REVOL_SPEED_MODE)
		{
			//������������
			if( revolver_control->ifshoot_finsh != shoot_unfinish&&(revolver_control->heating > 10 || revolver_control->heat_max ==0) )
				{
					//˫������
					revolver_control->set_angle-= AN_BULLET;
//					if(if_stucked == 1)
//					{
//						revolver_control->set_angle-= AN_BULLET/2;
//						if_stucked = 0;
//					}
				}
		}
					//б�º���	
			if(revolver_control->set_ramp_angle != revolver_control->set_angle)//����ת��ȥ
			{
				revolver_control->set_ramp_angle = RAMP_float(revolver_control->set_angle, revolver_control->set_ramp_angle, revolver_control->revolver_buff_ramp);
			}
			 //�ǶȻ����ٶȻ�����pid���ԣ������������ڲ������ۼ�ʱ�Ѿ���ɣ�
			angle_out = PID_calc(&revolver_control->revolver_motor_position_pid, revolver_control->angle, revolver_control->set_ramp_angle);
			revolver_control->given_current = PID_calc(&revolver_control->revolver_motor_speed_pid, revolver_control->speed, angle_out);
		
		if(switch_is_down(revolver_control->revolver_rc_ctrl->rc.s[RevChannel]))
		{
			revolver_control->given_current=0;
			
		  revolver_control->set_angle=revolver_control->angle;

		}
		
					/************************************�������*************************************/
			if(revolver_control->revolver_rc_ctrl->rc.ch[0] > 600)		
//			if ( revolver_control->revolver_rc_ctrl->key.v & Bombhatch)
			{				
				if(rc_keyZ_time<200)//���ոǿ���ʱ��Ϊ500����
				 {
					 rc_keyZ_time++;
					 if(rc_keyZ_time==200 && steering_mode==0)
					 {
						 steering_mode=1;  //��
					 }
					 else if(rc_keyZ_time==200 && steering_mode==1)
					 {
						 steering_mode=0;  //�ر�
					 }
				 }
			}
			else
			{
				rc_keyZ_time=0;
			}
			
			if(steering_mode==1)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1550);			
			}				
			else if(steering_mode==0)
			{
				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2300);
			}
			
		
		Shoot_Can_Set_Current=revolver_control->given_current;
}

/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */

static void Revolver_Feedback_Update(Revolver_Control_t *revolver_feedback)
{
	
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (revolver_feedback->revolver_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    revolver_feedback->speed = speed_fliter_3;

		//���̵������ֵ�ۼ�
    REVOL_UpdateMotorAngleSum(revolver_feedback);

    //���������Ƕ�
    revolver_feedback->angle = revolver_feedback->revolver_motor_measure->num*-8192 + revolver_feedback->revolver_motor_measure->ecd;
		
    //��갴��
    revolver_feedback->last_press_l = revolver_feedback->press_l;
    revolver_feedback->press_l = revolver_feedback->revolver_rc_ctrl->mouse.press_l;
    //������ʱ
    if (revolver_feedback->press_l)
    {
        if (revolver_feedback->press_l_time < PRESS_LONG_TIME)
        {
            revolver_feedback->press_l_time++;
        }
    }
    else
    {
        revolver_feedback->press_l_time = 0;
    }
		
		revolver_feedback->Firc_L.speed=revolver_feedback->Firc_L.firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;
		revolver_feedback->Firc_R.speed=revolver_feedback->Firc_R.firc3508_motor_measure->speed_rpm*Motor_RMP_TO_SPEED;

}

static void REVOL_UpdateMotorAngleSum(Revolver_Control_t *revolver_sum)
{
	
	
	//�ٽ�ֵ�жϷ�
	if (fabs(revolver_sum->revolver_motor_measure->ecd - revolver_sum->revolver_motor_measure->last_ecd) > 4095)//ת����Ȧ
	{		
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (revolver_sum->revolver_motor_measure->ecd < revolver_sum->revolver_motor_measure->last_ecd)//����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			revolver_sum->sum_ecd += 8191 -  revolver_sum->revolver_motor_measure->last_ecd + revolver_sum->revolver_motor_measure->ecd;
		}
		else
		{
			//������һȦ
				revolver_sum->sum_ecd -= 8191 - revolver_sum->revolver_motor_measure->ecd + revolver_sum->revolver_motor_measure->last_ecd;
		}
	}
	else      
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
			 revolver_sum->sum_ecd += revolver_sum->revolver_motor_measure->ecd  -  revolver_sum->revolver_motor_measure->last_ecd;
	}
	
	
	
}






//��ң�����󲦸˴�����ʱ���ܷ�����
static void REVOLVER_Rc_Switch(Revolver_Control_t *revolver_control)
{
	
		static int16_t speed_time;
	
		//�󲦸˲���һ�λ�����������ΪSTEP1  �󲦸�һֱ���ϻ򳤰����ΪSTEP2
		if ( (switch_is_up(revolver_control->revolver_rc_ctrl->rc.s[RevChannel])||revolver_control->revolver_rc_ctrl->mouse.press_l==1)&&(switch_is_down(revolver_control->revolver_rc_ctrl->rc.s[MODE_CHANNEL])!=1))
		{
			speed_time++;
			revolver_control->Revolver_Switch = REVOL_STEP1;
			
			if(speed_time>300)
			{
			revolver_control->Revolver_Switch = REVOL_STEP2;	
			}			
		}
		else 
		{
		revolver_control->Revolver_Switch = REVOL_STEP0;
		revolver_control->key_ShootFlag = shoot_unready;
		speed_time = 0;
		}
	
		//STEP2�����ٶȻ�
		if(revolver_control->Revolver_Switch == REVOL_STEP2&&revolver_control->ifstuck_finish != stuck_unfinish)
				revolver_control->Revolver_mode =	REVOL_SPEED_MODE;
		else if((revolver_control->Revolver_Switch == REVOL_STEP1||revolver_control->Revolver_Switch == REVOL_STEP0)&&revolver_control->ifstuck_finish != stuck_unfinish)
			  revolver_control->Revolver_mode =	REVOL_POSI_MODE;
		
		//���˲���һ�λ��������һ�η���1 ����Ϊ0
		if(revolver_control->Revolver_Switch == REVOL_STEP1 && revolver_control->Revolver_last_Switch == REVOL_STEP0)
		{
			if(revolver_control->heating> 10 || revolver_control->heat_max == 0)//abs(revolver_control->set_angle- revolver_control->angle)<1000) || heat_max == 0)
			{
				revolver_control->key_ShootFlag = shoot_ready;
			}
		}
		else
			revolver_control->key_ShootFlag = shoot_unready;

		revolver_control->Revolver_last_Switch = revolver_control->Revolver_Switch;
}
/**
 * @brief �������
 * @note  Ħ���ֱ��٣���̬����ת��
 */
void Temp_Fix_30S(void)
{
  float temp_scope = 35;//����仯��ΧΪ35���϶�
  float temp_low = 35;//��ʼ�¶��趨Ϊ35���϶�
  float res = 0;
  float temp_real;
  
  temp_real = ((float)Revolver_control.Firc_L.firc3508_motor_measure->temperate +
               (float)Revolver_control.Firc_R.firc3508_motor_measure->temperate)/2;
  
  if(temp_real >= temp_low)
    res = (temp_real - temp_low)/temp_scope * (-168);
  if(temp_real < temp_low)
    res = 0;
  if(temp_real > temp_low + temp_scope)
    res = -168;
  
  Revolver_control.Level_3_TEMP_fix_num = res;
}
void Temp_Fix_18S(void)
{
  float temp_scope = 35;//����仯��ΧΪ25���϶�
  float temp_low = 35;//��ʼ�¶��趨Ϊ35���϶�
  float res = 0;
  float temp_real;
  
  temp_real = ((float)Revolver_control.Firc_L.firc3508_motor_measure->temperate +
               (float)Revolver_control.Firc_R.firc3508_motor_measure->temperate)/2;
  
  if(temp_real >= temp_low)
    res = (temp_real - temp_low)/temp_scope * (-70);
  if(temp_real < temp_low)
    res = 0;
  if(temp_real > temp_low + temp_scope)
    res = -70;
  
   Revolver_control.Level_2_TEMP_fix_num = res;
}
void Temp_Fix_15S(void)
{
  float temp_scope = 35;//����仯��ΧΪ50���϶�
  float temp_low = 35;//��ʼ�¶��趨Ϊ35���϶�
  float res = 0;
  float temp_real;
  
  temp_real = ((float)Revolver_control.Firc_L.firc3508_motor_measure->temperate +
               (float)Revolver_control.Firc_R.firc3508_motor_measure->temperate)/2;
  
  if(temp_real >= temp_low)
    res = (temp_real - temp_low)/temp_scope * (-50);
  if(temp_real < temp_low)
    res = 0;
  if(temp_real > temp_low + temp_scope)
    res = -50;
  
   Revolver_control.Level_1_TEMP_fix_num = res;
}

uint8_t SpeedErr_cnt=0;	 
void SpeedAdapt(float real_S , float min_S, float max_S,float *fix , float up_num , float down_num)
{
  if(real_S < min_S && real_S > 8)
    SpeedErr_cnt++;
  else if(real_S >= min_S && real_S <= max_S )SpeedErr_cnt = 0;
  if(SpeedErr_cnt == 1)//����ƫ��
  {
    SpeedErr_cnt = 0;
    *fix += up_num;
  }
  if(real_S > max_S)//����ƫ��
    *fix -= down_num;
}

static void Firc_Control_loop(Revolver_Control_t *fire_control)
{
	
	float realspeed =JUDGE_usGetSpeedHeat();
	uint8_t speedlimit; 
	speedlimit =JUDGE_usGetSpeedLimit();
//	fire_control->v_fic_set = -8500;
	if(speedlimit==15)
  {

      SpeedAdapt(realspeed , 13.5 , 14.5 , &fire_control->Level_1_fix_num , 15 , 35);
      Temp_Fix_15S();
	}
    
	else if(speedlimit==18)
	{
      SpeedAdapt(realspeed , 16.5 , 17.5 , &fire_control->Level_2_fix_num , 15 , 35);
      Temp_Fix_18S();
	}

	else if(speedlimit==30)
	{
      SpeedAdapt(realspeed , 28.5 , 29.5 , &fire_control->Level_3_fix_num , 25 , 50);
      Temp_Fix_30S();//3508����¶Ȼ�Ӱ�����٣��¶�Խ������Խ��Խ��Խ�����̧ͷ����׼������֮���ٶ�����
	}

	
	

	if(speedlimit==15)
	{
          fire_control->v_fic_set = FRICTION_L1_SPEED/* + 
                                                fire_control->Level_1_fix_num*/;
	}
                                         
	else if(speedlimit==18)
	{
          fire_control->v_fic_set = FRICTION_L2_SPEED /*+ 
                                                fire_control->Level_2_fix_num*/;
	}
                                         
	else if(speedlimit==30)
	{
          fire_control->v_fic_set = FRICTION_L3_SPEED /*+ 
                                                fire_control->Level_3_fix_num*/;
	}
  else
	{
          fire_control->v_fic_set = FRICTION_L1_SPEED;//FRICTION_INIT_SPEED;
	}
	
	if(switch_is_down(fire_control->revolver_rc_ctrl->rc.s[0]))
	{
		fire_control->Firc_L.speed_set=0;
		fire_control->Firc_R.speed_set=0;
		fire_control->Firc_L.speed_ramp_set = 0;
		fire_control->Firc_R.speed_ramp_set = 0;
	}
	else
	{
		if(switch_is_down(fire_control->revolver_rc_ctrl->rc.s[RevChannel]))
		{
			fire_control->Firc_L.speed_set=0;
			fire_control->Firc_R.speed_set=0;
			fire_control->Firc_L.speed_ramp_set = 0;
			fire_control->Firc_R.speed_ramp_set = 0;
			Revolver_control.given_current = 0;

//			laser_off();
		}
		else
		{
			fire_control->Firc_L.speed_set=fire_control->v_fic_set;
			fire_control->Firc_R.speed_set=-fire_control->v_fic_set;
//			laser_on();
		
			
		}
		fire_control->Firc_L.speed_ramp_set = RAMP_float(fire_control->Firc_L.speed_set,fire_control->Firc_L.speed_ramp_set,10);
		fire_control->Firc_R.speed_ramp_set = RAMP_float(fire_control->Firc_R.speed_set,fire_control->Firc_R.speed_ramp_set,10);
		}

	
	fire_control->Firc_L.give_current=PID_calc(&fire_control->Firc_L.firc_speed_pid, fire_control->Firc_L.firc3508_motor_measure->speed_rpm,fire_control->Firc_L.speed_ramp_set);
	fire_control->Firc_R.give_current=PID_calc(&fire_control->Firc_R.firc_speed_pid, fire_control->Firc_R.firc3508_motor_measure->speed_rpm,fire_control->Firc_R.speed_ramp_set);
}


