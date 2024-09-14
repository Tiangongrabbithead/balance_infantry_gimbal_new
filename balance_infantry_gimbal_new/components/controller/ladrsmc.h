#ifndef LADRSMC_H
#define LADRSMC_H
#include "struct_typedef.h"

typedef struct
{   fp32 time_cons;//时间常数
	
	fp32 wo;//观测器带宽
	fp32 b0;//输出增益
	fp32 z1;
	fp32 z2;
	
    
  fp32 wc;//控制器带宽
  fp32 K;//控制相关参数
	fp32 s;
	
  fp32 max_out;  //最大输出
	//输入
  fp32 set;//设定值
  fp32 fdb;//反馈值
	fp32 gyro;//角速度
	
	
  fp32 smcout;//输出
	fp32 u;
	fp32 err;
	
	
} ladrsmc_type_def;



extern void LADRSMC_init(ladrsmc_type_def *ladrsmc,fp32 wc, fp32 K,fp32 b0 ,fp32 wo,fp32 max_out);
extern fp32 LADRSMC_calc(ladrsmc_type_def *ladrsmc, fp32 ref, fp32 set, fp32 speed);
#endif
