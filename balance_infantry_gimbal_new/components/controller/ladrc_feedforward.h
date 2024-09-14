#ifndef LADRC_FEEDFORWARD_H
#define LADRC_FEEDFORWARD_H
#include "struct_typedef.h"

typedef struct
{   
	fp32 y[2];
	fp32 u[2];
} differ_type_def;

typedef struct
{   
	fp32 time_cons;//时间常数
	
	fp32 wo;//观测器带宽
	fp32 b0;//输出增益
	fp32 z1;
	fp32 z2;
	
	
	differ_type_def differ1;
	differ_type_def differ2;
    
  fp32 wc;//控制器带宽
	
  fp32 max_out;  //最大输出
	
	
	fp32 dif1;
	fp32 dif2; 
	fp32 w;//前馈带宽
	fp32 gain;//前馈增益
	
	//输入
  fp32 set;//设定值
	fp32 set_last;//上次设定值
  fp32 fdb;//反馈值
	fp32 gyro;//角速度
	fp32 err;
	
	fp32 u;	
} ladrc_fdw_type_def;

extern void LADRC_FDW_init(ladrc_fdw_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain);
extern fp32 LADRC_FDW_calc(ladrc_fdw_type_def *ladrc, fp32 ref, fp32 set, fp32 speed);

#endif
