#include "program.h"

/** ===================================================================
 **     Funtion Name :void WorkInit(void)
 **     Description :执行初始化函数
 **     Parameters  :void
 **     Returns     :void
 ** ===================================================================*/
void WorkInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_Delay(10);
	UserADC1_Init();	
#if LPF_ENABLE == 1
    for (char i = 0; i < 5; i++)
    {
        low_filter_init(&lpf[i], 200e3, 30e3);
    }
#endif
}

/** ===================================================================
 **     Funtion Name :void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 **     Description :TIM2弱回调函数,放置不断执行的函数
 **									-weak-
 **     Parameters  :TIM_HandleTypeDef *htim
 **     Returns     :void
 ** ===================================================================*/
//刷新计数 5mS计数一次，在中断里累加
uint16_t timer2cnt = 0; //65535
/*-weak-*/void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		timer2cnt++;//计时
		StateM();//状态机，运行状态

//    BBMode();//判断buck与mix模态
		DRMode();//判断充放电方向
		BuckBoostVLoopCtlPID();//PID模态回路计算
	}
}

