#include "function.h"


struct _Ctr_value CtrValue = { 0.0f, 0.0f, 30.0f, 0.0f, 0 ,MIN_BUCK_DUTY, 0, 0, 0 }; //控制参数
struct _FLAG FLAGS = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //控制标志位




/** ===================================================================
 **     Funtion Name :void BBMode(void)
 **     Description :运行模式判断
 **      Buck模式：输出参考电压<0.8倍输入电压
 **      MIX模式：1.2倍输入电压>输出参考电压>0.8倍输入电压
 **      当进入MIX（buck-boost）模式后，退出到 Buck时需要滞缓，防止在临界点来回振荡
 **     Parameters  :
 **     Returns     :
 ** ===================================================================
*/

void BBMode(void)
{
    // 上一次模式状态量
    uint8_t PreBBFlag = 0;

    // 暂存当前的模式状态量
    PreBBFlag = FLAGS.BBFlag;
    switch (FLAGS.BBFlag)
    {
    // NA
    case NA0:
    {
        /*init control value*/
        CtrValue.Voref = tarvoltage;
        CtrValue.Poref = 30.0f;
        // power ref will change in CAN_receive file, HAL_FDCAN_RxFifo0Callback function

        if (cap.V < (in.V * 0.8f)) // vout < 0.8 * vin
        {
            FLAGS.BBFlag = Buck; // buck mode
        }
        else
        {
            FLAGS.BBFlag = Mix; // buck-boost mode
        }
        break;
    }
    // Buck模式
    case Buck:
    {
        if (pid_final_buck >= 0.83f) // vout > 0.85 * vin
        {
            FLAGS.BBFlag = Mix; // buck-boost mode
        }
        break;
    }
    // Mix模式
    case Mix:
    {
        if (pid_final_boost <= 0.05f) // vout < 0.8 * vin
        {
            FLAGS.BBFlag = Buck; // buck mode
        }
        break;
    }
    }
    // when mode changes, set reg
    if (PreBBFlag != FLAGS.BBFlag)
        FLAGS.BBModeChange = 1;
		else
			  FLAGS.BBModeChange = 0;

}

/** ===================================================================
 **     Function Name :void DRMode(void)
 **     Description:currnt direction judge
 **     Charge mode:battery charges cap
 **     Discharge mode:cap discharge power to chassis
 **     when get into Discharge mode,exits need to put in a delay,in order to avoid oscillating
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void DRMode(void)
{
    //last dr mode
    uint8_t PreDRFlag = 0;

    //save current dr mode
    PreDRFlag = FLAGS.DRFlag;

    /*full_bridge_in.I = c_in.value_elec - c_chassis.value_elec;*/
    switch (FLAGS.DRFlag)
    {
    //PID set - ref, when over power, error is below 0
    //NA
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
        // charge mode
    case Charge:
    {
			LED_ON(RED);
			LED_OFF(BLUE);
        if (powerin_Ploop.output < 0)
        {
            FLAGS.DRFlag = Discharge;  //Discharge mode
        }
        break;
    }
        //discharge mode
    case Discharge:
    {
			LED_OFF(RED);
			LED_ON(BLUE);
        if (powerin_Ploop.output >= 0)
        {
            FLAGS.DRFlag = Charge;     //charge mode
        }
        break;
    }
    }
    //when mode changes,set reg
    if (PreDRFlag != FLAGS.DRFlag)
        FLAGS.DRModeChange = 1;
//    else
//        FLAGS.DRModeChange = 0;

}

/** ===================================================================
 **     Funtion Name :void LED_xx(enum _LED_Color LED_Color)
 **     Description :LED切换
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

void LED_ON(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);}//设置GPIO引脚电平}
	else if (LED_Color == RED)  {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, GPIO_PIN_RESET);}
	else if (LED_Color == BLUE) {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);}
}
	
void LED_OFF(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);}
	else if (LED_Color == RED)  {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, GPIO_PIN_SET);}
	else if (LED_Color == BLUE) {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);}
}

void LED_Toggle(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);}
	else if (LED_Color == RED)  {HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5) ;}
	else if (LED_Color == BLUE) {HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6) ;}
}


/** ===================================================================
 **     Function Name :void StateM(void)
 **     Description :   状态机函数
 **                     state machine function
 **     初始化状态
 **     initial state
 **     等外启动状态
 **     wait state
 **     启动状态
 **     rise state
 **     运行状态
 **     run state
 **     故障状态
 **     error state
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
//软启动状态标志位
SState_M STState = SSInit;

void StateM(void)
{
    //判断状态类型
    //judge state type
    switch (FLAGS.SMFlag)
    {
    //初始化状态
    case Init:
        StateMInit();
        break;
        //等待状态
    case Wait:
        StateMWait();
        break;
        //软启动状态
    case Rise:
        StateMRise();
        break;
        //运行状态
    case Run:
        StateMRun();
        break;
        //故障状态
    case Err:
        StateMErr();
        break;
    }
}


/** ===================================================================
 **     Function Name :void StateMInit(void)
 **     Description :   初始化状态函数，参数初始化
 **                     initial function, parameter initialed
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void StateMInit(void)
{
    //相关参数初始化
    //initial relative parameter
    ValueInit();
    //状态机跳转至等待软启状态
    //go to wait state waiting for rise state
    FLAGS.SMFlag = Wait;
}

/** ===================================================================
 **     Function Name :void StateMWait(void)
 **     Description :   等待状态机，等待1S后无故障则软启
 **                     wait state, wait for 1s to start when there is no error
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void StateMWait(void)
{
    //计数器定义
    //counter
    static uint16_t CntS = 0;

    //关PWM
    //shutdown PWM
    FLAGS.PWMENFlag = 0;
    //计数器累加
    //counter add
    CntS++;
    //等待1ms*200，进入启动状态
    //wait for 1ms*200, entering rise state
    if (CntS > 256)
    {
        CntS = 256;

        if (FLAGS.ErrFlag == F_NOERR)
        {
            //计数器清0
            //counter reset to zero
            CntS = 0;
            //状态标志位跳转至等待状态
            //state flag go 2 wait state
            FLAGS.SMFlag = Rise;
            //软启动子状态跳转至初始化状态
            //soft start(rise state) go 2 initial state
            STState = SSInit;
        }
    }
}

/*
 ** ===================================================================
 **     Function Name : void StateMRise(void)
 **     Description :软启阶段
 **     软启初始化
 **     软启等待
 **     开始软启
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */

#define MAX_SSCNT       100 //等待100ms
//wait for 100ms
void StateMRise(void)
{
    //计时器
    //counter
    static uint16_t Cnt = 0;
    //最大占空比限制计数器
    //maxim duty restrict counter
    static uint16_t BuckMaxDutyCnt = 0;
    static uint16_t BoostMaxDutyCnt = 0;

    //判断软启状态
    //judge soft start state
    switch (STState)
    {
    //初始化状态
    //initial state
    case SSInit:
    {
        //关闭PWM
        //shutdown PWM
        FLAGS.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD2); //关闭
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
        //软启中将运行限制占空比启动，从最小占空比开始启动
        //soft starting will restrict operating duty, start at minimum duty
        CtrValue.BuckMaxDuty = MIN_BUCK_DUTY;
        CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
        //环路计算变量初始化
        //control loop calculate parameter initial
					IncrementalPID_Clear(&currentout_Iloop); 
					IncrementalPID_Clear(&voltageout_Vloop); 
					IncrementalPID_Clear(&powerin_Ploop); 
        //跳转至软启等待状态
        //jump 2 soft start waiting state
        STState = SSWait;

        break;
    }
        //等待软启动状态
        //soft start waiting state
    case SSWait:
    {
        //计数器累加
        //counter add
        Cnt++;
        //等待100ms
        //wait for 100ms
        if (Cnt > MAX_SSCNT)
        {
            //计数器清0
            //counter reset 2 0
            Cnt = 0;
            //环路计算变量初始化
            //control loop calculate parameter initial
						IncrementalPID_Clear(&currentout_Iloop); 
						IncrementalPID_Clear(&voltageout_Vloop); 
						IncrementalPID_Clear(&powerin_Ploop); 
            //限制启动占空比
            //restrict operation duty
            if (cap.V <= 5.0f)
            {
                CtrValue.BuckDuty = cap.V / in.V * HRTIMA_Period;
                CtrValue.BuckMaxDuty = MIN_BUCK_DUTY;
                CtrValue.BoostDuty = MIN_BOOST_DUTY;
                CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
            }
            else
            {
								volt_ratio = cap.V/in.V + (1.0f + 0.05f - 0.83f);
                CtrValue.BuckDuty = volt_ratio * HRTIMA_Period;
                CtrValue.BuckMaxDuty = MAX_BUCK_DUTY;
                CtrValue.BoostDuty = MIN_BOOST_DUTY;
                CtrValue.BoostMaxDuty = MAX_BOOST_DUTY;
            }
            STState = SSRun;    //跳转至软启状态
                                //jump 2 soft start state
        }
        break;
    }
        //软启动状态
        //soft start state
    case SSRun:
    {
        if (FLAGS.PWMENFlag == 0)
				{
					IncrementalPID_Clear(&currentout_Iloop); 
					IncrementalPID_Clear(&voltageout_Vloop); 
					IncrementalPID_Clear(&powerin_Ploop); 
				/*------------------------------------------------*/
					/*-----定时器启动！----*/
					/*-----相关函数初始化----*/
          User_HRTIM_init();
												                         /* p      i     d  */
					IncrementalPID_Init(&currentout_Iloop, 0.001f, 0.008f, 0.03f,volt_ratio,
                         1.2f, -1.2f,
                         1.2f, 0.10f); 
					IncrementalPID_Init(&voltageout_Vloop, 0.00f, 0.03f, 0.00f,volt_ratio,
                         1.2f, -1.2f,
                         1.2f, 0.10f);
					IncrementalPID_Init(&powerin_Ploop, 0.0f, 0.0005f, 0.0f,0.0f,
                         1.2f, -1.2f,
                         10.00f, -10.00f);	
					CAN_Filter_Init(&hcan);
	        UART2_Open(RxBuffer,sizeof(RxBuffer));
				/*-----低通滤波-----*/	
#if LPF_ENABLE == 1
					for (char i = 0; i < 5; i++)
					{
						low_filter_init(&lpf[i], 200e3, 30e3);
					}
#endif
					
				/*------------------------------------------------*/
				
        //发波标志位置位
        //PWM start flag
					FLAGS.PWMENFlag = 1;
				}
        //最大占空比限制逐渐增加
        //maximum PWM duty add gradually
        BuckMaxDutyCnt++;
        BoostMaxDutyCnt++;
        //最大占空比限制累加
        //PWM maximum restrict add gradually
        CtrValue.BuckMaxDuty = CtrValue.BuckMaxDuty + BuckMaxDutyCnt * 5;
        CtrValue.BoostMaxDuty = CtrValue.BoostMaxDuty + BoostMaxDutyCnt * 5;
        //累加到最大值
        //add to the maximum value
        if (CtrValue.BuckMaxDuty > MAX_BUCK_DUTY)
        {
            CtrValue.BuckMaxDuty = MAX_BUCK_DUTY;
        }
        if (CtrValue.BoostMaxDuty > MAX_BOOST_DUTY)
        {
            CtrValue.BoostMaxDuty = MAX_BOOST_DUTY;
        }

        if (CtrValue.BuckMaxDuty == MAX_BUCK_DUTY && CtrValue.BoostMaxDuty == MAX_BOOST_DUTY)
        {
            //状态机跳转至运行状态
            //state machine jump 2 run state
            FLAGS.SMFlag = Run;
            //软启动子状态跳转至初始化状态
            //soft start state jump 2 initial state
            STState = SSInit;
        }
        break;
    }
    default:
        break;
    }
}

/*
 ** ===================================================================
 **     Function Name :void StateMRun(void)
 **     Description :正常运行，主处理函数在中断中运行
 **                  normal operation, main function is run in interrupt
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
void StateMRun(void)
{
		DRMode();//判断充放电方向
	  BBMode();
		BuckBoostVLoopCtlPID();//PID模态回路计算
		DetectMErr();
}

/*
 ** ===================================================================
 **     Function Name :void StateMErr(void)
 **     Description :故障状态
 **                  error state
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
void StateMErr(void)
{
    //关闭PWMF
    //Shutdown PWM
    FLAGS.PWMENFlag = 0;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭

    //若故障消除跳转至等待重新软启
    //restart when error state clear up
    if (FLAGS.ErrFlag == F_NOERR)
        FLAGS.SMFlag = Wait;
}

/*
 ** ===================================================================
 **     Function Name :void ShortOff(void)
 **     Description :短路保护，可以重启10次
 **                  shortage protect, can restart 10 times
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_SHORT_I     14.0f//短路电流判据
//shortage current judgment
#define MIN_SHORT_V     0.8f//短路电压判据
//shortage voltage judgment
void ShortOff(void)
{
    static int32_t RSCnt = 0;
    static uint8_t RSNum = 0;

    //当output current大于 15A，且output voltage小于0.2V时，可判定为发生短路保护
    //when output current is larger than 11A and output voltage less than 0.2V,
    //can trigger shortage protect
    if ((cap.I > MAX_SHORT_I) && (cap.V < MIN_SHORT_V))
    {
        //关闭PWM
        //shutdown PWM
        FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
        //故障标志位
        //error flag
        setRegBits(FLAGS.ErrFlag, F_SW_SHORT);
        //跳转至故障状态
        //jump 2 error state
        FLAGS.SMFlag = Err;
    }
    //输出短路保护恢复
    //recover for shortage protect
    //当发生输出短路保护，关机后等待4S后清除故障信息，进入等待状态等待重启
    //when triggered shortage protect, shutdown for 4s to wait for error state clean up,
    //entering wait state
    if (getRegBits(FLAGS.ErrFlag, F_SW_SHORT))
    {
        //等待故障清楚计数器累加
        //wait for error flag cleaning counter
        RSCnt++;
        //等待2S
        //wait for 2s
        if (RSCnt > 400)
        {
            //计数器清零
            //counter reset
            RSCnt = 0;
            //短路重启只重启10次，10次后不重启
            //shortage protect only reset for 10 times,
            //after 10 times system shutdown
            if (RSNum > 10)
            {
                //确保不清除故障，不重启
                //ensuring not clean error flag
                RSNum = 11;
                //关闭PWM
                //shutdown PWM
                FLAGS.PWMENFlag = 0;
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
            }
            else
            {
                //短路重启计数器累加
                //shortage protect counter add
                RSNum++;
                //清除过流保护故障标志位
                //clean shortage protect flag
                clrRegBits(FLAGS.ErrFlag, F_SW_SHORT);
            }
        }
    }
}

/*
 ** ===================================================================
 **     Function Name :void SwOCP(void)
 **     Description :软件过流保护，可重启
 **                  software shortage protect, can restart
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_OCP_VAL     13.0f//13A过流保护点
void SwOCP(void)
{
    //过流保护判据保持计数器定义
    static uint16_t OCPCnt = 0;
    //故障清楚保持计数器定义
    static uint16_t RSCnt = 0;
    //保留保护重启计数器
    static uint16_t RSNum = 0;

    //当output current大于*A，且保持500ms
    if ((cap.I > MAX_OCP_VAL) && (cap.I < -MAX_OCP_VAL) && (FLAGS.SMFlag == Run))
    {
        //条件保持计时
        OCPCnt++;
        //条件保持50ms，则认为过流发生
        if (OCPCnt > 10)
        {
            //计数器清0
            OCPCnt = 0;
            //关闭PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
            //故障标志位
            setRegBits(FLAGS.ErrFlag, F_SW_IOUT_OCP);
            //跳转至故障状态
            FLAGS.SMFlag = Err;
        }
    }
    else
        //计数器清0
        OCPCnt = 0;

    //输出过流后恢复
    //当发生输出软件过流保护，关机后等待4S后清除故障信息，进入等待状态等待重启
    if (getRegBits(FLAGS.ErrFlag, F_SW_IOUT_OCP))
    {
        //等待故障清楚计数器累加
        RSCnt++;
        //等待2S
        if (RSCnt > 400)
        {
            //计数器清零
            RSCnt = 0;
            //过流重启计数器累加
            RSNum++;
            //过流重启只重启10次，10次后不重启（严重故障）
            if (RSNum > 10)
            {
                //确保不清除故障，不重启
                RSNum = 11;
                //关闭PWM
                FLAGS.PWMENFlag = 0;
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
            }
            else
            {
                //清除过流保护故障标志位
                clrRegBits(FLAGS.ErrFlag, F_SW_IOUT_OCP);
            }
        }
    }
}

/*
 ** ===================================================================
 **     Function Name :void SwOVP(void)
 **     Description :软件输出过压保护，不重启
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_VOUT_OVP_VAL    25.0f//24V过压保护
void VoutSwOVP(void)
{
    //过压保护判据保持计数器定义
    static uint16_t OVPCnt = 0;

    //当output voltage大于25V，且保持100ms
    if (cap.V > MAX_VOUT_OVP_VAL)
    {
        //条件保持计时
        OVPCnt++;
        //条件保持10ms
        if (OVPCnt > 2)
        {
            //计时器清零
            OVPCnt = 0;
            //关闭PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
            //故障标志位
            setRegBits(FLAGS.ErrFlag, F_SW_VOUT_OVP);
            //跳转至故障状态
            FLAGS.SMFlag = Err;
        }
    }
    else
        OVPCnt = 0;
}

/*
 ** ===================================================================
 **     Function Name :void VinSwUVP(void)
 **     Description :输入软件欠压保护，低压输入保护,可恢复
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MIN_UVP_VAL    18.0f//18V欠压保护
#define MIN_UVP_VAL_RE 20.5f//20.5V欠压保护恢复
void VinSwUVP(void)
{
    //过压保护判据保持计数器定义
    static uint16_t UVPCnt = 0;
    static uint16_t RSCnt = 0;

    //当input voltage小于20V，且保持200ms
    if ((in.V < MIN_UVP_VAL) && (FLAGS.SMFlag != Init) && (!FLAGS.BBModeChange))
    {
        //条件保持计时
        UVPCnt++;
        //条件保持10ms
        if (UVPCnt > 2)
        {
            //计时器清零
            UVPCnt = 0;
            RSCnt = 0;
            //关闭PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
            //故障标志位
            setRegBits(FLAGS.ErrFlag, F_SW_VIN_UVP);
            //跳转至故障状态
            FLAGS.SMFlag = Err;
        }
    }
    else
        UVPCnt = 0;

    //输入欠压保护恢复
    //当发生输入欠压保护，等待输入电压恢复至正常水平后清楚故障标志位，重启
    if (getRegBits(FLAGS.ErrFlag, F_SW_VIN_UVP))
    {
        if (in.V > MIN_UVP_VAL_RE)
        {
            //等待故障清楚计数器累加
            RSCnt++;
            //等待1S
            if (RSCnt > 200)
            {
                RSCnt = 0;
                UVPCnt = 0;
                //清楚故障标志位
                clrRegBits(FLAGS.ErrFlag, F_SW_VIN_UVP);
            }
        }
        else
            RSCnt = 0;
    }
    else
        RSCnt = 0;
}

/*
 ** ===================================================================
 **     Funtion Name :void VinSwOVP(void)
 **     Description :软件输入过压保护，不重启
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_VIN_OVP_VAL    30.0f//28V过压保护
void VinSwOVP(void)
{
    //过压保护判据保持计数器定义
    static uint16_t OVPCnt = 0;

    //当input voltage大于30V，且保持100ms
    if (in.V > MAX_VIN_OVP_VAL)
    {
        //条件保持计时
        OVPCnt++;
        //条件保持10ms
        if (OVPCnt > 500)
        {
            //计时器清零
            OVPCnt = 0;
            //关闭PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
            //故障标志位
            setRegBits(FLAGS.ErrFlag, F_SW_VIN_OVP);
            //跳转至故障状态
            FLAGS.SMFlag = Err;
        }
    }
    else
        OVPCnt = 0;
}

/*
 ** ===================================================================
 **     Function Name :void DetectMErr(void)
 **     Description :故障检测执行函数
 **                  error detect
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
void DetectMErr(void)
{
	ShortOff();
//	SwOCP();
//	VoutSwOVP();
//	VinSwUVP();
//	VinSwOVP();
}

/** ===================================================================
 **     Function Name :void ValInit(void)
 **     Description :   相关参数初始化函数
 **                     relate parameter initial function
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void ValueInit(void)
{
    //关闭PWM
    //Shutdown PWM
    FLAGS.PWMENFlag = 0;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭
    //清除故障标志位
    //reset error flag
    FLAGS.ErrFlag = 0;
    //初始化参考量
    //initial reference parameter
    CtrValue.Voref = 0;
    CtrValue.Ioref = 0;
    CtrValue.Poref = 30.0f;
    //限制占空比
    //restrict PWM duty
    CtrValue.BuckDuty = MIN_BUCK_DUTY;
    CtrValue.BuckMaxDuty = MIN_BUCK_DUTY;
    CtrValue.BoostDuty = MIN_BOOST_DUTY;
    CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
    //环路计算变量初始化
    //control loop initial
		IncrementalPID_Clear(&currentout_Iloop); 
		IncrementalPID_Clear(&voltageout_Vloop); 
		IncrementalPID_Clear(&powerin_Ploop); 
}

/** ===================================================================
 **     Funtion Name :void hrtim_init(void)
 **     Description :HRTIME初始化
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void User_HRTIM_init(void)
{
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER|HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //开启子定时器A

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //通道打开
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道打开

}


