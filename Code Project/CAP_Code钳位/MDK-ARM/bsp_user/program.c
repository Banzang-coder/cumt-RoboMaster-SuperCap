#include "program.h"
/** ===================================================================
 **     Funtion Name :void WorkInit(void)
 **     Description :执行初始化函数
 **     Parameters  :void
 **     Returns     :void
 ** ===================================================================*/
uint16_t adcProtcnt = 0;
void WorkInit(void)
{
	UserADC1_Init();	
	while(1)
	{
		if(adcProtcnt > 500)break;
	}
	HAL_TIM_Base_Start_IT(&htim2);
	

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
		if(timer2cnt % 1000 == 0)
		{
			LED_Toggle(GREEN);

		}
		StateM();//状态机，运行状态

		if(timer2cnt % 5 == 0)
		{
			Communication_Onlinecnt++;
			CAN_SendData(&hcan, 0x601, 8);
			printf("%f,%f,%f,%f,%f,%d\r\n",in.P,full_bridge_in.I,in.I,in.V,volt_ratio,FLAGS.Cloop);
		}
		
	}
}

