#include "CUSUI.h"
//#include "masters_RTX.h"
#include "judge.h"
#include "can_receive.h"
#include "cmsis_os.h"
#include "remote_control.h"

#define TRUE 1
#define FALSE 0

#define YES TRUE
	#define NO  FALSE

RC_ctrl_t *UI_rc_ctrl;
UI_Info_t UI;
//extern UI_order UI_Measure;
extern int update_figure_flag,update_aim_flag,update_float_flag,update_supercapacitor_flag,update_int_flag;
extern bool global_clip,global_spin,global_auto_aim,global_shootlimit,global_block,global_bulletWarning;//圈圈
extern float global_supercapacitor_remain;//超电
//extern Chassis_Power_t		cap_measure;
void UI_Init(void)
{
//  Judge_IF_REF_ONL();
//		UI.IF_Init_Over = NO;
  if(!UI.IF_Init_Over)
  {
    
    update_figure_flag = ADD;
    update_aim_flag = ADD;	
    update_float_flag = ADD;
    update_supercapacitor_flag = ADD;
    
		
    update_int_flag = ADD;
    
    UI.IF_Init_Over = YES;

  }
}



  void UI_DataUpdate(void)
{
//  if(Report_UI_User_cmd_CLIP())
//    ShootNum_Reset();
  
  UI.User.AUTO =          UI_rc_ctrl->mouse.press_l==1;
//  UI.User.CLIP =             Report_UI_User_cmd_CLIP();
//  UI.User.SPIN =             UI_Measure.Spin;
////  UI.User.Shoot_heat_limit = Report_UI_User_cmd_SHOOTLIMIT();
////  UI.User.BlockMuch =        Report_UI_User_cmd_BlockMuch();
////  UI.User.Bullet_Warning =   Report_UI_User_cmd_BulletWarning();

//  UI.User.Vcap_show =        cap_measure.Cap_voltage;
}



void CUSUI_TASK(void  *argumt)
{
  uint32_t currentTime;
  for(;;)   
	{	
    currentTime = xTaskGetTickCount();//当前系统时间
    
    Startjudge_task();

    vTaskDelayUntil(&currentTime, 100);//绝对延时
  }
}

void Startjudge_task(void)
{
  static int i = -1,J = 0;
  i++;
  J++;
  UI_Init();
  UI_DataUpdate();
  
  switch(i)
  {
    case 0:
        Client_graphic_Init();   //文字
        break;
    
    case 1:
        global_clip     = UI.User.CLIP;
        global_spin     = UI.User.SPIN;
        global_auto_aim = UI.User.AUTO;
        global_shootlimit = UI.User.Shoot_heat_limit;
        global_block    = UI.User.BlockMuch;
        global_bulletWarning = UI.User.Bullet_Warning;
        Client_graphic_Info_update();//圆圈
        update_figure_flag = MODIFY;
        break;  
    case 2:
      _lowshortstem_aim_4();
      break;
    case 3:
       _lowlong_aim_();
      break;
    
    case 4:
        global_supercapacitor_remain = (UI.User.Vcap_show - 13)/(23-13) * 100;
        Client_supercapacitor_update();
        update_supercapacitor_flag = MODIFY;
        break;
    
    case 5:
        _high_aim_();
        break;
        
    case 6:
        _lowshort_aim_2();
        break;
    
    case 7:
        _lowshort_aim_3();
        break;
    
    
    default:
        i = -1;
        break;
  }
  
  if(J == 100)
  {
    J = 0;
    UI.IF_Init_Over = NO;
  }
}

