#ifndef __CUSUI_H
#define __CUSUI_H

//#include "system.h"
#include "main.h"
#include "struct_typedef.h"
#include "can_receive.h"
#define YES TRUE
	#define NO  FALSE
	
	typedef struct
{
  UI_order UI_send_measure;
	fp32 V_Cap_measure;
}UI_Measure_t;

typedef struct
{
  uint8_t SPIN;
  uint8_t CLIP;
  uint8_t AUTO;
  uint8_t BlockMuch;
  uint8_t Shoot_heat_limit;
  uint8_t Bullet_Warning;
	UI_Measure_t UI_measure;
  float Vcap_show;
	
}User_CMD_t;


typedef struct
{
  bool_t IF_Init_Over;
  User_CMD_t User;
}UI_Info_t;
extern void CUSUI_TASK(void *argumt);



void Startjudge_task(void);


#endif


