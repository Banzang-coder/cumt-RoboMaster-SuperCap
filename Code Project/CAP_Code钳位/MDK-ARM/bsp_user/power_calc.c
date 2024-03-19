#include "power_calc.h"
#include "drv_pid.h"

struct ParameterBridge dcdcinfo;

/** ===================================================================
 **     Funtion Name :void UserADC1_Init(void)
 ** 
 **     Description :ADC1�û���ʼ��������DMA/IT/��ѯ
 **                   
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
 
/*----------ADC��ʼ��----------*/
uint32_t UserADC1_ConvertedValue[ADC1_CHANNEL_FRE][ADC1_CHANNEL_CNT]={0};
void UserADC1_Init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_Delay(10);
	//ADC_Enable(&hadc1);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)UserADC1_ConvertedValue,ADC1_CHANNEL_FRE*ADC1_CHANNEL_CNT);
  //HAL_ADC_Start_IT(&hadc1);
}


/** ===================================================================
 **     Funtion Name :void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
 **  									-weak-
 **     Description :ADC����������ص�����,���Լ���ADC��������
 **									 �ڸú����н����˲������Լ��㣬������̹ر��ж�
 **                   
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

float adc_raw_data[ADC1_CHANNEL_CNT];
uint32_t usefor_adc[ADC1_CHANNEL_CNT];
/*-weak-*/void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for (uint8_t j = 0; j < ADC1_CHANNEL_CNT; j++) 
	{
    for (uint8_t i = 0; i < ADC1_CHANNEL_FRE; i++) 
		{
      adc_raw_data[j] += UserADC1_ConvertedValue[i][j];
    }
    adc_raw_data[j] /= ADC1_CHANNEL_FRE; // ȡƽ��ֵ
  }
	  for (uint8_t i = 0; i < ADC1_CHANNEL_CNT; i++) 
	  {
      usefor_adc[i] = adc_raw_data[i];
    }
	
#if board_version == 0
	dcdcinfo.V_bat =    	  (float)adc_raw_data[0]* 0.0121f   + 0.0407f;
	dcdcinfo.V_cap =     		(float)adc_raw_data[1]* 0.0120f   + 0.0283f;
	dcdcinfo.I_cap_in =     (float)adc_raw_data[2]* (-0.0053f)   +10.7935f;
	dcdcinfo.I_cap_out = 		(float)adc_raw_data[3]* 0.0053f   -10.8435f;
	dcdcinfo.I_bat =  			(float)adc_raw_data[4]* 0.0053f   -10.7935f;
#endif
	  for (uint8_t i = 0; i < ADC1_CHANNEL_CNT; i++) 
		{
      adc_raw_data[i] = 0;
    }
/*--IN3_VBAT,IN4_VCAP,IN11_IBAT,IN12_ICAPout,IN13_ICAPin--*/
		control_calc();
}



/*
 ** ===================================================================
 **     Funtion Name : void control_calc(float plim)
 **     Description :  �����·���ĵ����
 **     Parameters  :��
 **                 none
 **     Returns     :��
 **                 none
 ** ===================================================================
 */
CONTROL_STRUCT in, chassis, full_bridge_in, cap; //�����룬���������dcdc���룬dcdc���
void control_calc(void)
{
	in.I = dcdcinfo.I_bat;
	in.V = dcdcinfo.V_bat;
	in.P = in.V * in.I;
	
	chassis.I = dcdcinfo.I_bat - dcdcinfo.I_cap_in;
	chassis.V = dcdcinfo.V_bat;
	chassis.P = chassis.V * chassis.I;
	
	full_bridge_in.I = dcdcinfo.I_cap_in;
	full_bridge_in.V = dcdcinfo.V_bat;
	full_bridge_in.P = full_bridge_in.V * full_bridge_in.I;
	
	cap.I = dcdcinfo.I_cap_out;
	cap.V = dcdcinfo.V_cap;
	cap.P = cap.V * cap.I;
	
}

/*
 ** ===================================================================
 **     Funtion Name : void BuckBoostVLoopCtlPID(void)
 **     Description :  BUCK-BOOSTģʽ�»�·����
 **                    BUCK-BOOST mode loop calculate
 **     �������ο���ѹ�������ѹ�Ĺ�ϵ���жϻ�·������BUCKģʽ������BUCK-BOOSTģʽ
 **     detect output reference voltage's relationship with input voltage,
 **     judge loop control mode is buck-boost mode or buck mode
 **     BUCKģʽ�£�BOOST�Ŀ��عܹ����ڹ̶�ռ�ձȣ�����BUCK��ռ�ձȿ��������ѹ
 **     in BUCK mode, boost side mosfet is work in fixed duty, buck side mosfet control output voltage
 **     BUCK-BOOSTģʽ�£�BUCK�Ŀ��عܹ����ڹ̶�ռ�ձȣ�����BOOST��ռ�ձȿ��������ѹ
 **     in BUCK-BOOST mode, buck side mosfet is work in fixed duty, boost side mosfet control output voltage
 **     Parameters  :��
 **                 none
 **     Returns     :��
 **                 none
 ** ===================================================================
 */

BUCKBOOST_STRUCT buckboost;
volatile fp32 pid_final_buck = 0;
volatile fp32 pid_final_boost = 0;
float testpower = 30;


 void BuckBoostVLoopCtlPID(void)
{
    if (FLAGS.BBModeChange)
    {
			IncrementalPID_Clear(&currentout_Iloop); 
			IncrementalPID_Clear(&voltageout_Vloop); 
    }
    switch (FLAGS.BBFlag)
    {
			case NA0: 
			{				//init
				IncrementalPID_Clear(&currentout_Iloop); 
				IncrementalPID_Clear(&voltageout_Vloop); 
			break;
			}
			case Buck:  //BUCK mode
			{
				if (FLAGS.BBModeChange)
					{
							IncrementalPID_Init(&currentout_Iloop,0.0f,0.004f,0.0005f,0,0.01,-0.01,0.95,0.05); 
							IncrementalPID_Init(&voltageout_Vloop,0.0f,0.1f,1.0f,0,0.01,-0.01,0.95,0.05); 
							FLAGS.BBModeChange = 0;
					}
					IncrementalPID_Compute(&currentout_Iloop,testpower/full_bridge_in.V,full_bridge_in.I);
					IncrementalPID_Compute(&voltageout_Vloop,tarvoltage,cap.V);
					
					if (currentout_Iloop.output < voltageout_Vloop.output)       //���������ѹ��Ϊ���������ʻ�Ϊ������ǰ��
					{
							pid_final_buck = currentout_Iloop.output;
							FLAGS.Cloop = current_loop;
					}
					else
					{
							pid_final_buck = voltageout_Vloop.output;
							FLAGS.Cloop = voltage_loop;
					}
					pid_final_boost = 0.05f;
					CtrValue.BuckDuty = pid_final_buck * HRTIMA_Period;
					CtrValue.BoostDuty = pid_final_boost * HRTIMD_Period; //BOOST duty fixed PWM 0.95
					
					/*max out & min out restrict*/
					//soft start related
					if (CtrValue.BuckDuty > CtrValue.BuckMaxDuty)
					{
							CtrValue.BuckDuty = CtrValue.BuckMaxDuty;
					}
					//avoiding a voltage spike
					if (CtrValue.BuckDuty < (cap.V / 33.0f * HRTIMA_Period))
					{
							CtrValue.BuckDuty = cap.V / 33.0f * HRTIMA_Period;
					}
					break;
				}
			case Mix:   //Mix mode
			{
				if (FLAGS.BBModeChange)
					{
							IncrementalPID_Init(&currentout_Iloop,0.0f,-0.05f,0.0f,0.1,0.01f,-0.01f,0.50f,0.04f); 
							IncrementalPID_Init(&voltageout_Vloop,0.01f,0.2f,0.1f,0.1,0.01f,-0.01f,0.50f,0.04f); 
							FLAGS.BBModeChange = 0;
					}
					IncrementalPID_Compute(&currentout_Iloop,testpower/full_bridge_in.V,full_bridge_in.I);
					IncrementalPID_Compute(&voltageout_Vloop,tarvoltage,cap.V);
					
					if (currentout_Iloop.output < voltageout_Vloop.output)       //���������ѹ��Ϊ���������ʻ�Ϊ������ǰ��
					{
							pid_final_boost = currentout_Iloop.output;
							FLAGS.Cloop = current_loop;
					}
					else
					{
							pid_final_boost = voltageout_Vloop.output;
							FLAGS.Cloop = voltage_loop;
					}
					pid_final_buck = 0.83f;
					CtrValue.BoostDuty = pid_final_boost * HRTIMA_Period;
					CtrValue.BuckDuty = pid_final_buck * HRTIMD_Period; //buck duty fix 0.75

//					if (FLAGS.BBModeChange)
//					{
//							CtrValue.BoostDuty = (1.003f - 0.8f * in.V / cap.V) * HRTIMA_Period;
//							FLAGS.BBModeChange = 0;
//					}
					if (CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
					{
							CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
					}
					break;
			}
			default:
			{
					break;
			}
    }
    /*PWMENFlag is pwm enable flag, when this reg unset, PWM get to minimum period*/
    if (FLAGS.PWMENFlag == 0)
    {
        CtrValue.BuckDuty = MIN_BUCK_DUTY;
        CtrValue.BoostDuty = MIN_BOOST_DUTY;
    }
    /*update PWM regs*/
	 hhrtim1.Instance->sTimerxRegs[0].CMP1xR = HRTIMA_Period - CtrValue.BuckDuty;  //ͨ���޸ıȽ�ֵCMP���Ӷ��޸�ռ�ձ�
   hhrtim1.Instance->sTimerxRegs[3].CMP1xR = CtrValue.BoostDuty;  //ͨ���޸ıȽ�ֵCMP���Ӷ��޸�ռ�ձ�
}
