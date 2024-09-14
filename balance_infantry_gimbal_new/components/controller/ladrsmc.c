#include "main.h"
#include "ladrsmc.h"
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
	
	
	
	
	
void LADRSMC_init(ladrsmc_type_def *ladrsmc,fp32 wc, fp32 K,fp32 b0 ,fp32 wo,fp32 max_out)
{
    ladrsmc->wc = wc;
	ladrsmc->K = K;
	ladrsmc->wo = wo;
	ladrsmc->b0 = b0;
	
	
	ladrsmc->max_out = max_out;
	ladrsmc->fdb = 0.0f;
	ladrsmc->u = 0.0f;
	ladrsmc->set = 0.0f;
	ladrsmc->smcout=0;
	ladrsmc->gyro = 0.0f;
	ladrsmc->s = 0.0f;
	ladrsmc->z1 = 0;
	ladrsmc->z2 = 0;
	ladrsmc->time_cons = 0.002;//采样率
}


fp32 LADRSMC_calc(ladrsmc_type_def *ladrsmc, fp32 ref, fp32 set, fp32 gyro)
{
	fp32 err;
	err = set - ref;
	
	
	ladrsmc->set = set;
  ladrsmc->fdb = ref;
	ladrsmc->err = rad_format(err);
	ladrsmc->gyro = gyro;
	
	//零阶保持法离散化积分器
	ladrsmc->z2 += ladrsmc->time_cons*(ladrsmc->wo*ladrsmc->wo)*(ladrsmc->gyro-ladrsmc->z1);
	ladrsmc->z1 += ladrsmc->time_cons*((ladrsmc->b0*ladrsmc->u) + ladrsmc->z2 + (2*ladrsmc->wo)*(ladrsmc->gyro-ladrsmc->z1));
	
	
	ladrsmc->s = ladrsmc->wc*(ladrsmc->fdb-set)+ladrsmc->z1;
	ladrsmc->smcout = -ladrsmc->wc*ladrsmc->z1-ladrsmc->K*ladrsmc->s;
	
	ladrsmc->u = (ladrsmc->smcout - ladrsmc->z2)/ladrsmc->b0;
    LimitMax(ladrsmc->u, ladrsmc->max_out);
	return ladrsmc->u;
}



