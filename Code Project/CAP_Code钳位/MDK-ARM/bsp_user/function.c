#include "function.h"


struct _Ctr_value CtrValue = { 0.0f, 0.0f, 0.0f, MIN_BUCK_DUTY, 0, 0, 0 }; //���Ʋ���
struct _FLAG FLAGS = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //���Ʊ�־λ




/** ===================================================================
 **     Funtion Name :void BBMode(void)
 **     Description :����ģʽ�ж�

				!!ͨ����shift_ratio = 0.83f ���ж� �������ǽ���buck����mix�Դﵽ�޷��л�(ʹ��ռ�ձ�ǯλ)!!

 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

void BBMode(void)
{
    // ��һ��ģʽ״̬��
    uint8_t PreBBFlag = 0;

    // �ݴ浱ǰ��ģʽ״̬��
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
    // Buckģʽ
    case Buck:
    {
        if (pid_final_buck >= 0.83f) // vout > 0.85 * vin
        {
            FLAGS.BBFlag = Mix; // buck-boost mode
        }
        break;
    }
    // Mixģʽ
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
 **     Description :LED�л�
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

void LED_ON(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);}//����GPIO���ŵ�ƽ}
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
 **     Description :   ״̬������
 **                     state machine function
 **     ��ʼ��״̬
 **     initial state
 **     ��������״̬
 **     wait state
 **     ����״̬
 **     rise state
 **     ����״̬
 **     run state
 **     ����״̬
 **     error state
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
//������״̬��־λ
SState_M STState = SSInit;

void StateM(void)
{
    //�ж�״̬����
    //judge state type
    switch (FLAGS.SMFlag)
    {
    //��ʼ��״̬
    case Init:
        StateMInit();
        break;
        //�ȴ�״̬
    case Wait:
        StateMWait();
        break;
        //������״̬
    case Rise:
        StateMRise();
        break;
        //����״̬
    case Run:
        StateMRun();
        break;
        //����״̬
    case Err:
        StateMErr();
        break;
    }
}


/** ===================================================================
 **     Function Name :void StateMInit(void)
 **     Description :   ��ʼ��״̬������������ʼ��
 **                     initial function, parameter initialed
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void StateMInit(void)
{
    //��ز�����ʼ��
    //initial relative parameter
    ValueInit();
    //״̬����ת���ȴ�����״̬
    //go to wait state waiting for rise state
    FLAGS.SMFlag = Wait;
}

/** ===================================================================
 **     Function Name :void StateMWait(void)
 **     Description :   �ȴ�״̬�����ȴ�1S���޹���������
 **                     wait state, wait for 1s to start when there is no error
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void StateMWait(void)
{
    //����������
    //counter
    static uint16_t CntS = 0;

    //��PWM
    //shutdown PWM
    FLAGS.PWMENFlag = 0;
    //�������ۼ�
    //counter add
    CntS++;
    //�ȴ�1ms*200����������״̬
    //wait for 1ms*200, entering rise state
    if (CntS > 256)
    {
        CntS = 256;

        if (FLAGS.ErrFlag == F_NOERR)
        {
            //��������0
            //counter reset to zero
            CntS = 0;
            //״̬��־λ��ת���ȴ�״̬
            //state flag go 2 wait state
            FLAGS.SMFlag = Rise;
            //��������״̬��ת����ʼ��״̬
            //soft start(rise state) go 2 initial state
            STState = SSInit;
        }
    }
}

/*
 ** ===================================================================
 **     Function Name : void StateMRise(void)
 **     Description :�����׶�
 **     ������ʼ��
 **     �����ȴ�
 **     ��ʼ����
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */

#define MAX_SSCNT       100 //�ȴ�100ms
//wait for 100ms
void StateMRise(void)
{
    //��ʱ��
    //counter
    static uint16_t Cnt = 0;
    //���ռ�ձ����Ƽ�����
    //maxim duty restrict counter
    static uint16_t BuckMaxDutyCnt = 0;
    static uint16_t BoostMaxDutyCnt = 0;

    //�ж�����״̬
    //judge soft start state
    switch (STState)
    {
    //��ʼ��״̬
    //initial state
    case SSInit:
    {
        //�ر�PWM
        //shutdown PWM
        FLAGS.PWMENFlag = 0;
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD2); //�ر�
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
        //�����н���������ռ�ձ�����������Сռ�ձȿ�ʼ����
        //soft starting will restrict operating duty, start at minimum duty
        CtrValue.BuckMaxDuty = MIN_BUCK_DUTY;
        CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
        //��·���������ʼ��
        //control loop calculate parameter initial
				IncrementalPID_Clear(&currentout_Iloop); 
				IncrementalPID_Clear(&voltageout_Vloop); 
        //��ת�������ȴ�״̬
        //jump 2 soft start waiting state
        STState = SSWait;

        break;
    }
        //�ȴ�������״̬
        //soft start waiting state
    case SSWait:
    {
        //�������ۼ�
        //counter add
        Cnt++;
        //�ȴ�100ms
        //wait for 100ms
        if (Cnt > MAX_SSCNT)
        {
            //��������0
            //counter reset 2 0
            Cnt = 0;
            //��·���������ʼ��
            //control loop calculate parameter initial
						IncrementalPID_Clear(&currentout_Iloop); 
						IncrementalPID_Clear(&voltageout_Vloop); 
            //��������ռ�ձ�
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
            STState = SSRun;    //��ת������״̬
                                //jump 2 soft start state
        }
        break;
    }
        //������״̬
        //soft start state
    case SSRun:
    {
        if (FLAGS.PWMENFlag == 0)
				{
					IncrementalPID_Clear(&currentout_Iloop); 
					IncrementalPID_Clear(&voltageout_Vloop); 
					/*-----��ʱ��������----*/
          User_HRTIM_init();
        //������־λ��λ
        //PWM start flag
					FLAGS.PWMENFlag = 1;
				}

        //���ռ�ձ�����������
        //maximum PWM duty add gradually
        BuckMaxDutyCnt++;
        BoostMaxDutyCnt++;
        //���ռ�ձ������ۼ�
        //PWM maximum restrict add gradually
        CtrValue.BuckMaxDuty = CtrValue.BuckMaxDuty + BuckMaxDutyCnt * 5;
        CtrValue.BoostMaxDuty = CtrValue.BoostMaxDuty + BoostMaxDutyCnt * 5;
        if (CtrValue.BuckMaxDuty > MAX_BUCK_DUTY * 0.3)
        {
//            HAL_GPIO_WritePin(L_OD_GPIO_Port, L_OD_Pin, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(R_OD_GPIO_Port, R_OD_Pin, GPIO_PIN_SET);
        }
        //�ۼӵ����ֵ
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
            //״̬����ת������״̬
            //state machine jump 2 run state
            FLAGS.SMFlag = Run;
            //��������״̬��ת����ʼ��״̬
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
 **     Description :�������У������������ж�������
 **                  normal operation, main function is run in interrupt
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
void StateMRun(void)
{
		BBMode();//�ж�buck��mixģ̬
}

/*
 ** ===================================================================
 **     Function Name :void StateMErr(void)
 **     Description :����״̬
 **                  error state
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
void StateMErr(void)
{
    //�ر�PWM
    //Shutdown PWM
    FLAGS.PWMENFlag = 0;
//    HAL_GPIO_WritePin(R_OD_GPIO_Port, R_OD_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(L_OD_GPIO_Port, L_OD_Pin, GPIO_PIN_RESET);
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�

    //������������ת���ȴ���������
    //restart when error state clear up
    if (FLAGS.ErrFlag == F_NOERR)
        FLAGS.SMFlag = Wait;
}

/** ===================================================================
 **     Function Name :void ValInit(void)
 **     Description :   ��ز�����ʼ������
 **                     relate parameter initial function
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void ValueInit(void)
{
    //�ر�PWM
    //Shutdown PWM
    FLAGS.PWMENFlag = 0;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
    //������ϱ�־λ
    //reset error flag
    FLAGS.ErrFlag = 0;
    //��ʼ���ο���
    //initial reference parameter
    CtrValue.Voref = 0;
    CtrValue.Ioref = 0;
    CtrValue.Poref = 0;
    //����ռ�ձ�
    //restrict PWM duty
    CtrValue.BuckDuty = MIN_BUCK_DUTY;
    CtrValue.BuckMaxDuty = MIN_BUCK_DUTY;
    CtrValue.BoostDuty = MIN_BOOST_DUTY;
    CtrValue.BoostMaxDuty = MIN_BOOST_DUTY;
    //��·���������ʼ��
    //control loop initial
		IncrementalPID_Clear(&currentout_Iloop); 
		IncrementalPID_Clear(&voltageout_Vloop); 
}

/** ===================================================================
 **     Funtion Name :void hrtim_init(void)
 **     Description :HRTIME��ʼ��
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void User_HRTIM_init(void)
{
	
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //ͨ����
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����

	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A); //�����Ӷ�ʱ��A
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_D); //�����Ӷ�ʱ��A
}


