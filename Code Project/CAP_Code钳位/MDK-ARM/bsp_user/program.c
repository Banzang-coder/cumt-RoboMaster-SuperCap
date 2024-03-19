#include "program.h"

/** ===================================================================
 **     Funtion Name :void WorkInit(void)
 **     Description :ִ�г�ʼ������
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
		StateM();//״̬��������״̬

//    BBMode();//�ж�buck��mixģ̬
		DRMode();//�жϳ�ŵ緽��
		BuckBoostVLoopCtlPID();//PIDģ̬��·����
	}
}

