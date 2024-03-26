#include "power_calc.h"
#include "drv_pid.h"

struct ParameterBridge dcdcinfo;
struct ParameterBridge dcdctest;

/** ===================================================================
 **     Funtion Name :void UserADC1_Init(void)
 ** 
 **     Description :ADC1用户初始化函数，DMA/IT/轮询
 **                   
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
 
/*----------ADC初始化----------*/
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
 **     Description :ADC采样完成弱回调函数,线性计算ADC采样数据
 **									 在该函数中进行滤波与线性计算，计算过程关闭中断
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
    adc_raw_data[j] /= ADC1_CHANNEL_FRE; // 取平均值
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
		adcProtcnt++;
}



/*
 ** ===================================================================
 **     Funtion Name : void control_calc(float plim)
 **     Description :  计算回路中四点参数
 **     Parameters  :无	
 **                 none
 **     Returns     :无
 **                 none
 ** ===================================================================
 */
CONTROL_STRUCT in, chassis, full_bridge_in, cap; //总输入，底盘输出，dcdc输入，dcdc输出
void control_calc(void)
{
#if LPF_ENABLE == 1
    lpf[0].Input = dcdcinfo.I_bat;
    lpf[1].Input = dcdcinfo.V_bat;
    lpf[2].Input = dcdcinfo.I_cap_in;
    lpf[3].Input = dcdcinfo.I_cap_out;
    lpf[4].Input = dcdcinfo.V_cap;
	
    for (char i = 0; i < 5; i++)
    {
        low_filter_calc(&lpf[i]);
    }
		
		dcdctest.I_bat = lpf[0].Output;
		dcdctest.V_bat = lpf[1].Output;
		dcdctest.I_cap_in = lpf[2].Output;
		dcdctest.I_cap_out = lpf[3].Output;
		dcdctest.V_cap = lpf[4].Output;
		
		in.I = dcdctest.I_bat;
		in.V = dcdctest.V_bat;
		in.P = in.V * in.I;
		if(in.P<0){in.P = 0;}
		
		chassis.I = dcdctest.I_bat - dcdctest.I_cap_in;
		chassis.V = dcdctest.V_bat;
		chassis.P = chassis.V * chassis.I;
		
		full_bridge_in.I = dcdctest.I_cap_in;
		full_bridge_in.V = dcdctest.V_bat;
		full_bridge_in.P = full_bridge_in.V * full_bridge_in.I;
		
		cap.I = dcdctest.I_cap_out;
		cap.V = dcdctest.V_cap;
		cap.P = cap.V * cap.I;
	
#else
		in.I = dcdcinfo.I_bat;
		in.V = dcdcinfo.V_bat;
		in.P = in.V * in.I;
		if(in.P<0){in.P = 0;}
		
		chassis.I = dcdcinfo.I_bat - dcdcinfo.I_cap_in;
		chassis.V = dcdcinfo.V_bat;
		chassis.P = chassis.V * chassis.I;
		
		full_bridge_in.I = dcdcinfo.I_cap_in;
		full_bridge_in.V = dcdcinfo.V_bat;
		full_bridge_in.P = full_bridge_in.V * full_bridge_in.I;
		
		cap.I = dcdcinfo.I_cap_out;
		cap.V = dcdcinfo.V_cap;
		cap.P = cap.V * cap.I;
#endif
}

/*
 ** ===================================================================
 **     Funtion Name : void BuckBoostVLoopCtlPID(void)
 **     Description :  BUCK-BOOST模式下环路计算
 **                    BUCK-BOOST mode loop calculate
 **     检测输出参考电压与输入电压的关系，判断环路工作于BUCK模式，还是BUCK-BOOST模式
 **     detect output reference voltage's relationship with input voltage,
 **     judge loop control mode is buck-boost mode or buck mode
 **     BUCK模式下，BOOST的开关管工作在固定占空比，控制BUCK的占空比控制输出电压
 **     in BUCK mode, boost side mosfet is work in fixed duty, buck side mosfet control output voltage
 **     BUCK-BOOST模式下，BUCK的开关管工作在固定占空比，控制BOOST的占空比控制输出电压
 **     in BUCK-BOOST mode, buck side mosfet is work in fixed duty, boost side mosfet control output voltage
 **     Parameters  :无
 **                 none
 **     Returns     :无
 **                 none
 ** ===================================================================
 */

#define shift_ratio 0.83f
float testpower = 30.0f;
float testvoltage = 25.50f;

//CtrValue.Pcharge
volatile fp32 volt_ratio = 0;
volatile fp32 volt_ratio_filter = 0;
volatile fp32 pid_final_buck = 0;
volatile fp32 pid_final_boost = 0;
//		volt_ratio += 0.03f*(testpower/cap.V - cap.I);
void BuckBoostVLoopCtlPID(void)
{
	if(FLAGS.DRModeChange)
	{
		IncrementalPID_ClearWithoutOutput(&currentout_Iloop); 
		IncrementalPID_ClearWithoutOutput(&voltageout_Vloop); 
	}
	switch(FLAGS.DRFlag)
	{
		case NA1:
		{
        if (powerin_Ploop.output >= 0)
        {
            FLAGS.DRFlag = Charge;     //charge mode
        }
        else
        {
            FLAGS.DRFlag = Discharge;  //Discharge mode
        }
        break;
		}
		case Charge:
		{
			if(FLAGS.DRModeChange)
			{
			IncrementalPID_Init(&currentout_Iloop, 0.001f, 0.008f, 0.03f,volt_ratio,
                         1.2f, -1.2f,
                         1.2f, 0.10f); 
			IncrementalPID_Init(&voltageout_Vloop, 0.00f, 0.03f, 0.00f,volt_ratio,
                         1.2f, -1.2f,
                         1.2f, 0.10f);
			FLAGS.DRModeChange = 0;
			}
			IncrementalPID_Compute(&powerin_Ploop,CtrValue.Poref, in.P);
			IncrementalPID_Compute(&currentout_Iloop,powerin_Ploop.output, full_bridge_in.I);
			IncrementalPID_Compute(&voltageout_Vloop,CtrValue.Voref, cap.V);
			
			if (currentout_Iloop.output > voltageout_Vloop.output)
			{
				volt_ratio = voltageout_Vloop.output;
				FLAGS.Cloop = voltage_loop;
			}
			else
			{
				volt_ratio = currentout_Iloop.output;
				FLAGS.Cloop = current_loop;
			}
			
			/*---pid输出判断，占空比钳位，输出已在pid限幅---*/
			if(volt_ratio >=shift_ratio)
			{
				pid_final_buck = shift_ratio;
				pid_final_boost = volt_ratio - shift_ratio + 0.05f;
			}else if (volt_ratio <shift_ratio)
			{
				pid_final_buck = volt_ratio;
				pid_final_boost = 0.05f;
			}
					
			CtrValue.BuckDuty = pid_final_buck * HRTIMA_Period;
			CtrValue.BoostDuty = pid_final_boost * HRTIMA_Period;
			
			if (CtrValue.BuckDuty < (cap.V / 33.0f * HRTIMA_Period))
			{
				 CtrValue.BuckDuty = cap.V / 33.0f * HRTIMA_Period;
			}
			
			/*---PWM输出限幅,缓启动---*/
			if(CtrValue.BuckDuty > CtrValue.BuckMaxDuty)
			{
				CtrValue.BuckDuty = CtrValue.BuckMaxDuty;
			}
			if(CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
			{
				CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
			}
			if(CtrValue.BuckDuty < MIN_BUCK_DUTY)
			{
				CtrValue.BuckDuty = MIN_BUCK_DUTY;
			}
			if(CtrValue.BoostDuty < MIN_BOOST_DUTY)
			{
				CtrValue.BoostDuty = MIN_BOOST_DUTY;
			}
			break;
		}
		case Discharge:
		{
			if(FLAGS.DRModeChange)
			{
			IncrementalPID_Init(&currentout_Iloop, 0.001f, 0.008f, 0.03f,volt_ratio,
                         1.2f, -1.2f,
                         1.2f, 0.10f); 
			IncrementalPID_Init(&voltageout_Vloop, 0.00f, -0.03f, 0.00f,volt_ratio,
                         1.2f, -1.2f,
                         1.2f, 0.10f);
			FLAGS.DRModeChange = 0;
			}
			
			IncrementalPID_Compute(&powerin_Ploop,CtrValue.Poref, in.P);
			IncrementalPID_Compute(&currentout_Iloop,powerin_Ploop.output, full_bridge_in.I);
			IncrementalPID_Compute(&voltageout_Vloop,limitoutvoltage, in.V);
			
			if (currentout_Iloop.output > voltageout_Vloop.output)
			{
				volt_ratio = currentout_Iloop.output;
				FLAGS.Cloop = current_loop;
			}
			else
			{
				volt_ratio = voltageout_Vloop.output;
				FLAGS.Cloop = voltage_loop;
			}
			
			/*---pid输出判断，占空比钳位，输出已在pid限幅---*/
			if(volt_ratio >=shift_ratio)
			{
				pid_final_buck = shift_ratio;
				pid_final_boost = volt_ratio - shift_ratio + 0.05f;
			}else if (volt_ratio <shift_ratio)
			{
				pid_final_buck = volt_ratio;
				pid_final_boost = 0.05f;
			}
					
			CtrValue.BuckDuty = pid_final_buck * HRTIMA_Period;
			CtrValue.BoostDuty = pid_final_boost * HRTIMA_Period;
			
			if (CtrValue.BuckDuty < (cap.V / 33.0f * HRTIMA_Period))
			{
				 CtrValue.BuckDuty = cap.V / 33.0f * HRTIMA_Period;
			}
			
			/*---PWM输出限幅,缓启动---*/
			if(CtrValue.BuckDuty > CtrValue.BuckMaxDuty)
			{
				CtrValue.BuckDuty = CtrValue.BuckMaxDuty;
			}
			if(CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
			{
				CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
			}
			if(CtrValue.BuckDuty < MIN_BUCK_DUTY)
			{
				CtrValue.BuckDuty = MIN_BUCK_DUTY;
			}
			if(CtrValue.BoostDuty < MIN_BOOST_DUTY)
			{
				CtrValue.BoostDuty = MIN_BOOST_DUTY;
			}
			break;
		}	
	}
		/*---pwm使能控制---*/
    if (FLAGS.PWMENFlag == 0)
    {
        CtrValue.BuckDuty = MIN_BUCK_DUTY;
        CtrValue.BoostDuty = MIN_BOOST_DUTY;
    }
    /*update PWM regs*/
	 hhrtim1.Instance->sTimerxRegs[0].CMP1xR = CtrValue.BuckDuty;  //通过修改比较值CMP，从而修改占空比
   hhrtim1.Instance->sTimerxRegs[3].CMP1xR = CtrValue.BoostDuty;  //通过修改比较值CMP，从而修改占空比
}
