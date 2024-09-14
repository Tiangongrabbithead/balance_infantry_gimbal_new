#include "math.h"
#include "ladrc_feedforward.h"
#include "user_lib.h"
#include "arm_math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
	

void LADRC_FDW_init(ladrc_fdw_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out,fp32 w,fp32 gain)
{
    ladrc->wc = wc;
	ladrc->wo = wo;
	ladrc->b0 = b0;
	
	
	ladrc->gain = gain;
	ladrc->w = w;
	
	
	ladrc->max_out = max_out;
	ladrc->fdb = 0.0f;
	ladrc->u = 0.0f;
	ladrc->set = 0.0f;
	ladrc->gyro = 0.0f;
	ladrc->z1 = 0;
	ladrc->z2 = 0;
	ladrc->time_cons = 0.002;//采样率
}

fp32 differentiator(differ_type_def *differ ,fp32 bandwidth,fp32 time_cons,fp32 input)
{ 
  //使用梯形法离散化
 
  differ->u[0] = differ->u[1];	
  differ->u[1] = input;
  differ->y[0] = differ->y[1];
  differ->y[1] = (bandwidth*(differ->u[1]-differ->u[0])-(bandwidth*time_cons/2-1)*differ->y[0])/(1+(bandwidth*time_cons)/2);
  
  return differ->y[1];
}

fp32 LADRC_FDW_calc(ladrc_fdw_type_def *ladrc, fp32 ref, fp32 set, fp32 gyro)
{
	
	ladrc->err = rad_format(ladrc->set - ladrc->fdb);
	ladrc->set_last = ladrc->set;
	ladrc->set = set;
  ladrc->fdb = ref;
	ladrc->gyro = gyro;
	
	if(fabsf(ladrc->set-ladrc->set_last)>3.0f)//如果发生了跳变,防止产生过大的前馈量
	{
		 ladrc->differ1.y[0]=0;
		 ladrc->differ1.y[1]=0;
		 ladrc->differ1.u[0]=ladrc->set;
		 ladrc->differ1.u[1]=ladrc->set;
	 
		 ladrc->differ2.y[0]=0;
		 ladrc->differ2.y[1]=0;
		 ladrc->differ2.u[0]=0;
		 ladrc->differ2.u[1]=0;
	}
	
	ladrc->dif1 = differentiator(&ladrc->differ1 ,ladrc->w,ladrc->time_cons,set);
	ladrc->dif2 = differentiator(&ladrc->differ2 ,ladrc->w,ladrc->time_cons,ladrc->dif1);
	
	//带前馈的ladrc算法，先把前馈增益置0，调好控制器，再调节前馈器，前馈增益一般不大于1.
	
	//零阶保持法离散化积分器
	ladrc->z2 += ladrc->time_cons*(ladrc->wo*ladrc->wo)*(ladrc->gyro-ladrc->z1);
	ladrc->z1 += ladrc->time_cons*((ladrc->b0*ladrc->u) + ladrc->z2 + (2*ladrc->wo)*(ladrc->gyro-ladrc->z1));
	ladrc->u = (ladrc->wc*ladrc->wc*ladrc->err + 2*ladrc->wc*(ladrc->gain * ladrc->dif1-ladrc->z1) - ladrc->z2 + ladrc->gain * ladrc->dif2)/ladrc->b0;
	LimitMax(ladrc->u, ladrc->max_out);
   
	return ladrc->u;
}




