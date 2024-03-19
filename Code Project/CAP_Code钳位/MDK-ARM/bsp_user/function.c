#include "function.h"


struct _Ctr_value CtrValue = { 0.0f, 0.0f, 0.0f, MIN_BUCK_DUTY, 0, 0, 0 }; //控制参数
struct _FLAG FLAGS = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //控制标志位




/** ===================================================================
 **     Funtion Name :void BBMode(void)
 **     Description :运行模式判断

				!!通过对shift_ratio = 0.83f 的判断 来决定是进入buck还是mix以达到无缝切换(使用占空比钳位)!!

 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

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
        CtrValue.Poref = tarpower;
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
        if (powerin_loop.out >= 0)
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
        if (powerin_loop.out < 0)
        {
            FLAGS.DRFlag = Discharge;  //Discharge mode
        }
        break;
    }
        //discharge mode
    case Discharge:
    {
        if (powerin_loop.out >= 0)
        {
            FLAGS.DRFlag = Charge;     //charge mode
        }
        break;
    }
    }
    //when mode changes,set reg
    if (PreDRFlag != FLAGS.DRFlag)
        FLAGS.DRModeChange = 1;
    else
        FLAGS.DRModeChange = 0;

}

/** ===================================================================
 **     Funtion Name :void LED_xx(enum _LED_Color LED_Color)
 **     Description :LED切换
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

void LED_ON(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);}//设置GPIO引脚电平}
	else if (LED_Color == RED)  {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, GPIO_PIN_SET);}
	else if (LED_Color == BLUE) {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);}
}
	
void LED_OFF(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);}
	else if (LED_Color == RED)  {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, GPIO_PIN_RESET);}
	else if (LED_Color == BLUE) {HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);}
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
                CtrValue.BuckDuty = cap.V / in.V * HRTIMA_Period;
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
					/*-----定时器启动！----*/
          User_HRTIM_init();
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
        if (CtrValue.BuckMaxDuty > MAX_BUCK_DUTY * 0.3)
        {
//            HAL_GPIO_WritePin(L_OD_GPIO_Port, L_OD_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(R_OD_GPIO_Port, R_OD_Pin, GPIO_PIN_SET);
        }
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
		BBMode();//判断buck与mix模态
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
    //关闭PWM
    //Shutdown PWM
    FLAGS.PWMENFlag = 0;
//    HAL_GPIO_WritePin(R_OD_GPIO_Port, R_OD_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(L_OD_GPIO_Port, L_OD_Pin, GPIO_PIN_RESET);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //关闭
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //关闭

    //若故障消除跳转至等待重新软启
    //restart when error state clear up
    if (FLAGS.ErrFlag == F_NOERR)
        FLAGS.SMFlag = Wait;
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
    CtrValue.Poref = 0;
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
}

/** ===================================================================
 **     Funtion Name :void hrtim_init(void)
 **     Description :HRTIME初始化
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void User_HRTIM_init(void)
{
	
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //通道打开
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //通道打开

	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A); //开启子定时器A
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D); //开启子定时器A
}


