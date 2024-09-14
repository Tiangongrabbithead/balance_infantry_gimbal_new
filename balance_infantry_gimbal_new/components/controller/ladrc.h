#ifndef LADRC_H
#define LADRC_H
#include "struct_typedef.h"

typedef struct
{   
	fp32 time_cons;//ʱ�䳣��
	
	fp32 wo;//�۲�������
	fp32 b0;//�������
	fp32 z1;
	fp32 z2;
	
    
  fp32 wc;//����������
	
  fp32 max_out;  //������
	//����
  fp32 set;//�趨ֵ
  fp32 fdb;//����ֵ
	fp32 gyro;//���ٶ�
	fp32 err;
	
	fp32 u;
	
	
} ladrc_type_def;



extern void LADRC_init(ladrc_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out);
extern fp32 LADRC_calc(ladrc_type_def *ladrc, fp32 ref, fp32 set, fp32 speed);
#endif
