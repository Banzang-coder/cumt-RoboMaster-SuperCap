#ifndef _FUNCTION_
#define _FUNCTION_
/*ԭʼ��*/
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include "main.h"
#include "gpio.h"
#include "hrtim.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "stdio.h"
#include "can.h"

/*΢��*/
#include <stdio.h>
#include <stdlib.h>

/*�����*/
#include "arm_math.h" /*Ӱ������ٶȣ�ֻ����Ҫ�ĵط���*/
#include "math.h"
/*�Լ��Ŀ�*/
#include "filter.h"
#include "power_calc.h"
#include "struct_typedef.h"
#include "drv_pid.h"
#include "program.h"
#include "drv_can.h"
#include "drv_usart.h"

/*--------------------��������--------------------*/
#define     F_NOERR      	  	0x0000//�޹���
#define     F_SW_VIN_UVP  		0x0001//����Ƿѹ
#define     F_SW_VIN_OVP    	0x0002//�����ѹ
#define     F_SW_VOUT_UVP  		0x0004//���Ƿѹ
#define     F_SW_VOUT_OVP    	0x0008//�����ѹ
#define     F_SW_IOUT_OCP    	0x0010//�������
#define     F_SW_SHORT  	  	0x0020//�����·

/*--------------------����ռ�ձ�--------------------*/
#define MAX_BUCK_DUTY   (uint16_t)(0.95 * HRTIMD_Period)    //buck���ռ�ձ�95%
#define MIN_BUCK_DUTY	(uint16_t)(0.05 * HRTIMD_Period)    //buck��Сռ�ձ�5%
#define MAX_BOOST_DUTY  (uint16_t)(0.65 * HRTIMA_Period)    //boost���ռ�ձ�40%
#define MIN_BOOST_DUTY  (uint16_t)(0.05 * HRTIMA_Period)    //boost��Сռ�ձ�5%

#define HRTIMD_Period 46080
#define HRTIMA_Period 46080
#define HRTIMMaster_Period 46080

#define MIN_REG_VALUE   (uint16_t)1800                      //HRTIM reg mini value
#define MAX_REG_VALUE   (uint32_t)43776                     //HRTIM reg max value

#define tarvoltage 26.0f
#define tarpower 30.0f
#define limitoutvoltage 27.0f
#define maxNchargeV 33.0f
struct _Ctr_value
{
    fp32 Voref;             //�ο���ѹ
    fp32 Ioref;             //�ο�����
    fp32 Poref;             //�ο�����
		fp32 JudgeChassisPower;           //����ϵͳ���̹���
		uint8_t BufferPower;
    int32_t BuckMaxDuty;    //Buck���ռ�ձ�_cnt
    int32_t BoostMaxDuty;   //Boost���ռ�ձ�_cnt
    int32_t BuckDuty;       //Buck����ռ�ձ�_cnt
    int32_t BoostDuty;      //Boost����ռ�ձ�_cnt
};

//��־λ����
struct _FLAG
{
    uint16_t SMFlag;        //״̬����־λ
    uint16_t CtrFlag;       //���Ʊ�־λ
    uint16_t ErrFlag;       //���ϱ�־λ
    uint8_t BBFlag;         //����ģʽ��־λ��Buckģʽ��MIX���ģʽ
    uint8_t DRFlag;         //current direction flag,charge,discharge
    uint8_t Cloop;          //loop control mode
    uint8_t PWMENFlag;      //������־λ
    uint8_t BBModeChange;   //����ģʽ�л���־λ
    uint8_t DRModeChange;   //current direction change flag
		uint8_t ChargePNFlag;
		uint8_t CAPusableFlag;
};

//״̬��ö����
typedef enum
{
    Posdir,   //��ʼ��
		Negdir
} CHARGE_M;

//״̬��ö����
typedef enum
{
    Init,   //��ʼ��
    Wait,   //���еȴ�
    Rise,   //����
    Run,    //��������
    Err     //����
} STATE_M;

//״̬��ö����
typedef enum
{
    NA0,    //undefine
    Buck,   //Buck mode
    Mix     //MIX mode
} BB_M;

//״̬��ö����
typedef enum
{
    NA1,        //undefine
    Charge,     //charge mode
    Discharge   //discharge mode
} DR_M;

//״̬��ö����
typedef enum
{
    current_loop,
    voltage_loop
} CL_M;

//������ö�ٱ���
typedef enum
{
    SSInit, //soft start initl
    SSWait, //soft start waitting
    SSRun   //start soft start
} SState_M;

typedef enum _LED_Color
{
	GREEN,BLUE,RED
}LED_Color;

void LED_Toggle(enum _LED_Color LED_Color);
void LED_OFF(enum _LED_Color LED_Color);
void LED_ON(enum _LED_Color LED_Color);
void BBMode(void);
void DRMode(void);
void StateM(void);
void StateMInit(void);
void StateMWait(void);
void StateMRise(void);
void StateMRun(void);
void StateMErr(void);
void ValueInit(void);
void DetectMErr(void);
void ShortOff(void);
void SwOCP(void);
void VoutSwOVP(void);
void VinSwUVP(void);
void VinSwOVP(void);
void User_HRTIM_init(void);


extern struct _Ctr_value CtrValue;
extern struct _FLAG FLAGS;

#define setRegBits(reg, mask)   (reg |= (unsigned int)(mask))
#define clrRegBits(reg, mask)  	(reg &= (unsigned int)(~(unsigned int)(mask)))
#define getRegBits(reg, mask)   (reg & (unsigned int)(mask))
#define getReg(reg)           	(reg)

#endif /*_FUNCTION_*/
