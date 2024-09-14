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
	fp32 time_cons;//ʱ�䳣��
	
	fp32 wo;//�۲�������
	fp32 b0;//�������
	fp32 z1;
	fp32 z2;
	
	
	differ_type_def differ1;
	differ_type_def differ2;
    
  fp32 wc;//����������
	
  fp32 max_out;  //������
	
	
	fp32 dif1;
	fp32 dif2; 
	fp32 w;//ǰ������
	fp32 gain;//ǰ������
	
	//����
  fp32 set;//�趨ֵ
	fp32 set_last;//�ϴ��趨ֵ
  fp32 fdb;//����ֵ
	fp32 gyro;//���ٶ�
	fp32 err;
	
	fp32 u;	
} ladrc_fdw_type_def;

extern void LADRC_FDW_init(ladrc_fdw_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain);
extern fp32 LADRC_FDW_calc(ladrc_fdw_type_def *ladrc, fp32 ref, fp32 set, fp32 speed);

#endif
