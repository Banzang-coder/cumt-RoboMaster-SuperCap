#include "function.h"


struct _Ctr_value CtrValue = { 0.0f, 0.0f, 30.0f, 0.0f, 0 ,MIN_BUCK_DUTY, 0, 0, 0 }; //���Ʋ���
struct _FLAG FLAGS = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //���Ʊ�־λ




/** ===================================================================
 **     Funtion Name :void BBMode(void)
 **     Description :����ģʽ�ж�
 **      Buckģʽ������ο���ѹ<0.8�������ѹ
 **      MIXģʽ��1.2�������ѹ>����ο���ѹ>0.8�������ѹ
 **      ������MIX��buck-boost��ģʽ���˳��� Buckʱ��Ҫ�ͻ�����ֹ���ٽ��������
 **     Parameters  :
 **     Returns     :
 ** ===================================================================
*/

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
 **     Description :LED�л�
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/

void LED_ON(enum _LED_Color LED_Color)
{
	if      (LED_Color == GREEN){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);}//����GPIO���ŵ�ƽ}
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
					IncrementalPID_Clear(&powerin_Ploop); 
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
						IncrementalPID_Clear(&powerin_Ploop); 
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
								volt_ratio = cap.V/in.V + (1.0f + 0.05f - 0.83f);
                CtrValue.BuckDuty = volt_ratio * HRTIMA_Period;
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
					IncrementalPID_Clear(&powerin_Ploop); 
				/*------------------------------------------------*/
					/*-----��ʱ��������----*/
					/*-----��غ�����ʼ��----*/
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
				/*-----��ͨ�˲�-----*/	
#if LPF_ENABLE == 1
					for (char i = 0; i < 5; i++)
					{
						low_filter_init(&lpf[i], 200e3, 30e3);
					}
#endif
					
				/*------------------------------------------------*/
				
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
		DRMode();//�жϳ�ŵ緽��
	  BBMode();
		BuckBoostVLoopCtlPID();//PIDģ̬��·����
		DetectMErr();
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
    //�ر�PWMF
    //Shutdown PWM
    FLAGS.PWMENFlag = 0;
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
    HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�

    //������������ת���ȴ���������
    //restart when error state clear up
    if (FLAGS.ErrFlag == F_NOERR)
        FLAGS.SMFlag = Wait;
}

/*
 ** ===================================================================
 **     Function Name :void ShortOff(void)
 **     Description :��·��������������10��
 **                  shortage protect, can restart 10 times
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_SHORT_I     14.0f//��·�����о�
//shortage current judgment
#define MIN_SHORT_V     0.8f//��·��ѹ�о�
//shortage voltage judgment
void ShortOff(void)
{
    static int32_t RSCnt = 0;
    static uint8_t RSNum = 0;

    //��output current���� 15A����output voltageС��0.2Vʱ�����ж�Ϊ������·����
    //when output current is larger than 11A and output voltage less than 0.2V,
    //can trigger shortage protect
    if ((cap.I > MAX_SHORT_I) && (cap.V < MIN_SHORT_V))
    {
        //�ر�PWM
        //shutdown PWM
        FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
        //���ϱ�־λ
        //error flag
        setRegBits(FLAGS.ErrFlag, F_SW_SHORT);
        //��ת������״̬
        //jump 2 error state
        FLAGS.SMFlag = Err;
    }
    //�����·�����ָ�
    //recover for shortage protect
    //�����������·�������ػ���ȴ�4S�����������Ϣ������ȴ�״̬�ȴ�����
    //when triggered shortage protect, shutdown for 4s to wait for error state clean up,
    //entering wait state
    if (getRegBits(FLAGS.ErrFlag, F_SW_SHORT))
    {
        //�ȴ���������������ۼ�
        //wait for error flag cleaning counter
        RSCnt++;
        //�ȴ�2S
        //wait for 2s
        if (RSCnt > 400)
        {
            //����������
            //counter reset
            RSCnt = 0;
            //��·����ֻ����10�Σ�10�κ�����
            //shortage protect only reset for 10 times,
            //after 10 times system shutdown
            if (RSNum > 10)
            {
                //ȷ����������ϣ�������
                //ensuring not clean error flag
                RSNum = 11;
                //�ر�PWM
                //shutdown PWM
                FLAGS.PWMENFlag = 0;
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
            }
            else
            {
                //��·�����������ۼ�
                //shortage protect counter add
                RSNum++;
                //��������������ϱ�־λ
                //clean shortage protect flag
                clrRegBits(FLAGS.ErrFlag, F_SW_SHORT);
            }
        }
    }
}

/*
 ** ===================================================================
 **     Function Name :void SwOCP(void)
 **     Description :�������������������
 **                  software shortage protect, can restart
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_OCP_VAL     13.0f//13A����������
void SwOCP(void)
{
    //���������оݱ��ּ���������
    static uint16_t OCPCnt = 0;
    //����������ּ���������
    static uint16_t RSCnt = 0;
    //������������������
    static uint16_t RSNum = 0;

    //��output current����*A���ұ���500ms
    if ((cap.I > MAX_OCP_VAL) && (cap.I < -MAX_OCP_VAL) && (FLAGS.SMFlag == Run))
    {
        //�������ּ�ʱ
        OCPCnt++;
        //��������50ms������Ϊ��������
        if (OCPCnt > 10)
        {
            //��������0
            OCPCnt = 0;
            //�ر�PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
            //���ϱ�־λ
            setRegBits(FLAGS.ErrFlag, F_SW_IOUT_OCP);
            //��ת������״̬
            FLAGS.SMFlag = Err;
        }
    }
    else
        //��������0
        OCPCnt = 0;

    //���������ָ�
    //�����������������������ػ���ȴ�4S�����������Ϣ������ȴ�״̬�ȴ�����
    if (getRegBits(FLAGS.ErrFlag, F_SW_IOUT_OCP))
    {
        //�ȴ���������������ۼ�
        RSCnt++;
        //�ȴ�2S
        if (RSCnt > 400)
        {
            //����������
            RSCnt = 0;
            //���������������ۼ�
            RSNum++;
            //��������ֻ����10�Σ�10�κ����������ع��ϣ�
            if (RSNum > 10)
            {
                //ȷ����������ϣ�������
                RSNum = 11;
                //�ر�PWM
                FLAGS.PWMENFlag = 0;
                HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
            }
            else
            {
                //��������������ϱ�־λ
                clrRegBits(FLAGS.ErrFlag, F_SW_IOUT_OCP);
            }
        }
    }
}

/*
 ** ===================================================================
 **     Function Name :void SwOVP(void)
 **     Description :��������ѹ������������
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_VOUT_OVP_VAL    25.0f//24V��ѹ����
void VoutSwOVP(void)
{
    //��ѹ�����оݱ��ּ���������
    static uint16_t OVPCnt = 0;

    //��output voltage����25V���ұ���100ms
    if (cap.V > MAX_VOUT_OVP_VAL)
    {
        //�������ּ�ʱ
        OVPCnt++;
        //��������10ms
        if (OVPCnt > 2)
        {
            //��ʱ������
            OVPCnt = 0;
            //�ر�PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
            //���ϱ�־λ
            setRegBits(FLAGS.ErrFlag, F_SW_VOUT_OVP);
            //��ת������״̬
            FLAGS.SMFlag = Err;
        }
    }
    else
        OVPCnt = 0;
}

/*
 ** ===================================================================
 **     Function Name :void VinSwUVP(void)
 **     Description :�������Ƿѹ��������ѹ���뱣��,�ɻָ�
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MIN_UVP_VAL    18.0f//18VǷѹ����
#define MIN_UVP_VAL_RE 20.5f//20.5VǷѹ�����ָ�
void VinSwUVP(void)
{
    //��ѹ�����оݱ��ּ���������
    static uint16_t UVPCnt = 0;
    static uint16_t RSCnt = 0;

    //��input voltageС��20V���ұ���200ms
    if ((in.V < MIN_UVP_VAL) && (FLAGS.SMFlag != Init) && (!FLAGS.BBModeChange))
    {
        //�������ּ�ʱ
        UVPCnt++;
        //��������10ms
        if (UVPCnt > 2)
        {
            //��ʱ������
            UVPCnt = 0;
            RSCnt = 0;
            //�ر�PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
            //���ϱ�־λ
            setRegBits(FLAGS.ErrFlag, F_SW_VIN_UVP);
            //��ת������״̬
            FLAGS.SMFlag = Err;
        }
    }
    else
        UVPCnt = 0;

    //����Ƿѹ�����ָ�
    //����������Ƿѹ�������ȴ������ѹ�ָ�������ˮƽ��������ϱ�־λ������
    if (getRegBits(FLAGS.ErrFlag, F_SW_VIN_UVP))
    {
        if (in.V > MIN_UVP_VAL_RE)
        {
            //�ȴ���������������ۼ�
            RSCnt++;
            //�ȴ�1S
            if (RSCnt > 200)
            {
                RSCnt = 0;
                UVPCnt = 0;
                //������ϱ�־λ
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
 **     Description :��������ѹ������������
 **     Parameters  : none
 **     Returns     : none
 ** ===================================================================
 */
#define MAX_VIN_OVP_VAL    30.0f//28V��ѹ����
void VinSwOVP(void)
{
    //��ѹ�����оݱ��ּ���������
    static uint16_t OVPCnt = 0;

    //��input voltage����30V���ұ���100ms
    if (in.V > MAX_VIN_OVP_VAL)
    {
        //�������ּ�ʱ
        OVPCnt++;
        //��������10ms
        if (OVPCnt > 500)
        {
            //��ʱ������
            OVPCnt = 0;
            //�ر�PWM
            FLAGS.PWMENFlag = 0;
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TD1); //�ر�
            HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TD2); //�ر�
            //���ϱ�־λ
            setRegBits(FLAGS.ErrFlag, F_SW_VIN_OVP);
            //��ת������״̬
            FLAGS.SMFlag = Err;
        }
    }
    else
        OVPCnt = 0;
}

/*
 ** ===================================================================
 **     Function Name :void DetectMErr(void)
 **     Description :���ϼ��ִ�к���
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
    CtrValue.Poref = 30.0f;
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
		IncrementalPID_Clear(&powerin_Ploop); 
}

/** ===================================================================
 **     Funtion Name :void hrtim_init(void)
 **     Description :HRTIME��ʼ��
 **     Parameters  :
 **     Returns     :
 ** ===================================================================*/
void User_HRTIM_init(void)
{
	HAL_HRTIM_WaveformCounterStart(&hhrtim1,HRTIM_TIMERID_MASTER|HRTIM_TIMERID_TIMER_A|HRTIM_TIMERID_TIMER_D); //�����Ӷ�ʱ��A

	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2); //ͨ����
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1|HRTIM_OUTPUT_TD2); //ͨ����

}


