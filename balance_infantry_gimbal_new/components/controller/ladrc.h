#ifndef LADRC_H
#define LADRC_H
#include "struct_typedef.h"

typedef struct
{   
	fp32 time_cons;//时间常数
	
	fp32 wo;//观测器带宽
	fp32 b0;//输出增益
	fp32 z1;
	fp32 z2;
	
    
  fp32 wc;//控制器带宽
	
  fp32 max_out;  //最大输出
	//输入
  fp32 set;//设定值
  fp32 fdb;//反馈值
	fp32 gyro;//角速度
	fp32 err;
	
	fp32 u;
	
	
} ladrc_type_def;



extern void LADRC_init(ladrc_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out);
extern fp32 LADRC_calc(ladrc_type_def *ladrc, fp32 ref, fp32 set, fp32 speed);
#endif
