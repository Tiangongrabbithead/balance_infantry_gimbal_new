#include "math.h"
#include "ladrc.h"
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
	
	
	
	
	
void LADRC_init(ladrc_type_def *ladrc,fp32 wc, fp32 b0 ,fp32 wo,fp32 max_out)
{
    ladrc->wc = wc;
	ladrc->wo = wo;
	ladrc->b0 = b0;
	
	
	ladrc->max_out = max_out;
	ladrc->fdb = 0.0f;
	ladrc->u = 0.0f;
	ladrc->set = 0.0f;
	ladrc->gyro = 0.0f;
	ladrc->z1 = 0;
	ladrc->z2 = 0;
	ladrc->time_cons = 0.002;//采样率
}


fp32 LADRC_calc(ladrc_type_def *ladrc, fp32 ref, fp32 set, fp32 gyro)
{
	
	fp32 err;
	err = set - ref;
	
	
	ladrc->set = set;
  ladrc->fdb = ref;
	ladrc->err = rad_format(err);
	ladrc->gyro = gyro;
	
	//零阶保持法离散化积分器
	ladrc->z2 += ladrc->time_cons*(ladrc->wo*ladrc->wo)*(ladrc->gyro-ladrc->z1);
	ladrc->z1 += ladrc->time_cons*((ladrc->b0*ladrc->u) + ladrc->z2 + (2*ladrc->wo)*(ladrc->gyro-ladrc->z1));
	ladrc->u = (ladrc->wc*ladrc->wc*ladrc->err - 2*ladrc->wc*ladrc->z1 - ladrc->z2)/ladrc->b0;
	LimitMax(ladrc->u, ladrc->max_out);
   
	return ladrc->u;
}




