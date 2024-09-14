#ifndef _REVOLVER_TASK_H
#define _REVOLVER_TASK_H

#include "can_receive.h"
#include "pid.h"
#include "stm32f4xx.h"
#include "main.h"
#include "user_lib.h"
#include "remote_control.h"
#include "FreeRTOS.h"

#define REVOLVER_TASK_INIT_TIME 200
#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define PRESS_LONG_TIME 300
#define RevChannel 1
#define MODE_CHANNEL 0  //选择底盘状态 开关通道号
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

#define 	REVOL_SPEED_GRID      8			//拨盘格数
#define  	AN_BULLET         36864//33903.370320855614973262032085562		//单个子弹电机位置增加值
#define   REVOL_CAN_OPEN    350  //摩擦轮实际速度超过这个值才允许拨盘转动,根据摩擦轮最小目标速度来改变


#define REVOL_STEP0    0		//失能标志
#define REVOL_STEP1    1		//SW1复位标志
#define REVOL_STEP2    2		//弹仓开关标志
#define Bombhatch KEY_PRESSED_OFFSET_Z   //弹舱盖开关按键
#define RC_KEY_LONG_TIME 500       //长按时间

#define REVOLVER_SPEED_PID_KP 400//780.0f
#define REVOLVER_SPEED_PID_KI 0
#define REVOLVER_SPEED_PID_KD 880.2//5
#define REVOLVER_SPEED_PID_MAX_OUT 16000.0f
#define REVOLVER_SPEED_PID_MAX_IOUT 5000.0f

#define REVOLVER_POSITION_PID_KP 0.0005//0.00080f//0.00072f
#define REVOLVER_POSITION_PID_KI 0
#define REVOLVER_POSITION_PID_KD 0.0006f//0.000001f
#define REVOLVER_POSITION_PID_MAX_OUT 10.0f
#define REVOLVER_POSITION_PID_MAX_IOUT 10.0f


#define FIRC_SPEED_PID_KP 30.5  ///4.5f   12
#define FIRC_SPEED_PID_KI 0.000f
#define FIRC_SPEED_PID_KD 3.0f  //2
#define FIRC_SPEED_PID_MAX_OUT 16384.0f
#define FIRC_SPEED_PID_MAX_IOUT 5000.0f

#define FRICTION_L1_SPEED 4700
#define FRICTION_L2_SPEED 5500
#define FRICTION_L3_SPEED 7600


//电机rmp 变化成 旋转速度的比例
#define one_heat 10.0f

typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
	REVOL_REVERSAL_MODE =2,
}eRevolverCtrlMode;
typedef enum
{
	shoot_unready  = 0,
	shoot_ready = 1,
}Key_ShootFlag;
typedef enum
{
	shoot_unfinish  = 0,
	shoot_finish = 1,
}Shoot_IFfinish;
typedef enum
{
	stuck_unfinish  = 0,
	stuck_finish = 1,
}Stuck_IFfinish;
typedef enum
{
	OPEN  = 0,
	CLOSE = 1,
}eCoverState;

#define INIT 0
#define NORMAL 1

typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;



typedef struct
{
  const motor_measure_t *firc3508_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
	fp32 speed_ramp_set;
  int16_t give_current;
	pid_type_def firc_speed_pid;

} Firc3508_Motor_t;


typedef struct
{
	  const RC_ctrl_t *revolver_rc_ctrl;
	  RC_ctrl_time_t RC_revolver_ctrl_time;
    const motor_measure_t *revolver_motor_measure;
	  pid_type_def revolver_motor_speed_pid;         //电机速度PID
	  pid_type_def revolver_motor_position_pid;         //电机位置PID
	

	eShootAction actShoot;
	  eShootAction last_actShoot;            
	  eRevolverCtrlMode Revolver_mode;
	  eRevolverCtrlMode last_Revolver_mode;
	
	  bool_t press_l;
    bool_t last_press_l;
    uint16_t press_l_time;
	
	  Key_ShootFlag key_ShootFlag;
		Shoot_IFfinish ifshoot_finsh;
		Stuck_IFfinish ifstuck_finish;
	   
	  uint16_t heat;
		uint16_t	heat_max;
		int16_t 	heating; 	
	  uint16_t key_ShootNum;
	
	  fp32 speed;
    fp32 speed_set;
    fp32 angle;
    int64_t set_angle;
	  int64_t set_ramp_angle;
		int64_t sum_ecd;
		fp32 heat_exceed;             //手动超热量标志位
		int64_t remaining_angle;
		
		fp32 revolver_buff_ramp;
    int16_t given_current;
		
		uint8_t	Revolver_Switch;
		uint8_t Revolver_last_Switch;
	
	  Firc3508_Motor_t Firc_R;
	  Firc3508_Motor_t Firc_L;
		float Level_1_fix_num;
		float Level_2_fix_num;
		float Level_3_fix_num;
		float Level_1_TEMP_fix_num;
		float Level_2_TEMP_fix_num;
		float Level_3_TEMP_fix_num;
    int16_t v_fic_set;
} Revolver_Control_t;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Revolver_PID_t;



extern void Revolver_task(void  const *pvParameters);
//TickType_t REVOL_uiGetRevolTime(void);







#endif
