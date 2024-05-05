#include "program.h"
/** ===================================================================
 **     Funtion Name :void WorkInit(void)
 **     Description :ִ�г�ʼ������
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
 **     Description :TIM2���ص�����,���ò���ִ�еĺ���
 **									-weak-
 **     Parameters  :TIM_HandleTypeDef *htim
 **     Returns     :void
 ** ===================================================================*/
//ˢ�¼��� 5mS����һ�Σ����ж����ۼ�
uint16_t timer2cnt = 0; //65535
/*-weak-*/void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		timer2cnt++;//��ʱ
		if(timer2cnt % 1000 == 0)
		{
			LED_Toggle(GREEN);

		}
		StateM();//״̬��������״̬

		if(timer2cnt % 5 == 0)
		{
			Communication_Onlinecnt++;
			CAN_SendData(&hcan, 0x601, 8);
			printf("%f,%f,%f,%f,%f,%d\r\n",in.P,full_bridge_in.I,in.I,in.V,volt_ratio,FLAGS.Cloop);
		}
		
	}
}

