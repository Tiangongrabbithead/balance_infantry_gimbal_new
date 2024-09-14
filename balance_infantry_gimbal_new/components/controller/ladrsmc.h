#ifndef LADRSMC_H
#define LADRSMC_H
#include "struct_typedef.h"

typedef struct
{   fp32 time_cons;//ʱ�䳣��
	
	fp32 wo;//�۲�������
	fp32 b0;//�������
	fp32 z1;
	fp32 z2;
	
    
  fp32 wc;//����������
  fp32 K;//������ز���
	fp32 s;
	
  fp32 max_out;  //������
	//����
  fp32 set;//�趨ֵ
  fp32 fdb;//����ֵ
	fp32 gyro;//���ٶ�
	
	
  fp32 smcout;//���
	fp32 u;
	fp32 err;
	
	
} ladrsmc_type_def;



extern void LADRSMC_init(ladrsmc_type_def *ladrsmc,fp32 wc, fp32 K,fp32 b0 ,fp32 wo,fp32 max_out);
extern fp32 LADRSMC_calc(ladrsmc_type_def *ladrsmc, fp32 ref, fp32 set, fp32 speed);
#endif
