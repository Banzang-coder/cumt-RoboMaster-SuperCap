#ifndef _FUNCTION_
#define _FUNCTION_
/*原始库*/
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

/*微库*/
#include <stdio.h>
#include <stdlib.h>

/*外设库*/
#include "arm_math.h" /*影响编译速度，只在需要的地方加*/
#include "math.h"
/*自己的库*/
#include "filter.h"
#include "power_calc.h"
#include "struct_typedef.h"
#include "drv_pid.h"
#include "program.h"
#include "drv_can.h"
#include "drv_usart.h"

/*--------------------故障类型--------------------*/
#define     F_NOERR      	  	0x0000//无故障
#define     F_SW_VIN_UVP  		0x0001//输入欠压
#define     F_SW_VIN_OVP    	0x0002//输入过压
#define     F_SW_VOUT_UVP  		0x0004//输出欠压
#define     F_SW_VOUT_OVP    	0x0008//输出过压
#define     F_SW_IOUT_OCP    	0x0010//输出过流
#define     F_SW_SHORT  	  	0x0020//输出短路

/*--------------------极限占空比--------------------*/
#define MAX_BUCK_DUTY   (uint16_t)(0.95 * HRTIMD_Period)    //buck最大占空比95%
#define MIN_BUCK_DUTY	(uint16_t)(0.05 * HRTIMD_Period)    //buck最小占空比5%
#define MAX_BOOST_DUTY  (uint16_t)(0.65 * HRTIMA_Period)    //boost最大占空比40%
#define MIN_BOOST_DUTY  (uint16_t)(0.05 * HRTIMA_Period)    //boost最小占空比5%

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
    fp32 Voref;             //参考电压
    fp32 Ioref;             //参考电流
    fp32 Poref;             //参考功率
		fp32 JudgeChassisPower;           //裁判系统底盘功率
		uint8_t BufferPower;
    int32_t BuckMaxDuty;    //Buck最大占空比_cnt
    int32_t BoostMaxDuty;   //Boost最大占空比_cnt
    int32_t BuckDuty;       //Buck控制占空比_cnt
    int32_t BoostDuty;      //Boost控制占空比_cnt
};

//标志位定义
struct _FLAG
{
    uint16_t SMFlag;        //状态机标志位
    uint16_t CtrFlag;       //控制标志位
    uint16_t ErrFlag;       //故障标志位
    uint8_t BBFlag;         //运行模式标志位，Buck模式，MIX混合模式
    uint8_t DRFlag;         //current direction flag,charge,discharge
    uint8_t Cloop;          //loop control mode
    uint8_t PWMENFlag;      //启动标志位
    uint8_t BBModeChange;   //工作模式切换标志位
    uint8_t DRModeChange;   //current direction change flag
		uint8_t ChargePNFlag;
		uint8_t CAPusableFlag;
};

//状态机枚举量
typedef enum
{
    Posdir,   //初始化
		Negdir
} CHARGE_M;

//状态机枚举量
typedef enum
{
    Init,   //初始化
    Wait,   //空闲等待
    Rise,   //软启
    Run,    //正常运行
    Err     //故障
} STATE_M;

//状态机枚举量
typedef enum
{
    NA0,    //undefine
    Buck,   //Buck mode
    Mix     //MIX mode
} BB_M;

//状态机枚举量
typedef enum
{
    NA1,        //undefine
    Charge,     //charge mode
    Discharge   //discharge mode
} DR_M;

//状态机枚举量
typedef enum
{
    current_loop,
    voltage_loop
} CL_M;

//软启动枚举变量
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
