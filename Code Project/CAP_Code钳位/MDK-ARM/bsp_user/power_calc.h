#ifndef _POWER_CALC_
#define _POWER_CALC_

#ifdef __cplusplus
extern "C"
{
#endif
	
#include "function.h"
#include "struct_typedef.h"

#define HRTIMD_Period 46080
#define HRTIMA_Period 46080
#define HRTIMMaster_Period 46080

#define ADC1_CHANNEL_CNT 5 	//采样通道数
#define ADC1_CHANNEL_FRE 5	//单个通道采样次数，用来取平均值
#define board_version (0)   //板子序号，用哪个板子就是那个
#define LPF_ENABLE    (1)   //低通滤波使能
	
typedef struct
{
    fp32 V;
    fp32 I;
    fp32 P;

} CONTROL_STRUCT;

 struct ParameterBridge
{
  float V_cap;
  float V_bat;
  float I_bat;
  float	I_cap_out;
  float	I_cap_in;
};

volatile typedef struct
{
  uint8_t data_update;    //1更新
  uint8_t loop_mode;     	//环路模式0：cv 1：cc 2：cp
  float efficiency;      	//总效率 ，热耗散
  uint8_t power_dir;     	//能量方向0正向充电1反向放电
  uint16_t max_power_lim; //裁判系统最大功率
  float power_pass;				//裁判系统花去功率板与板子测量误差经过pid结果用于传递
  float powerbuffer ;			//缓冲能量
} BUCKBOOST_STRUCT;

void BuckBoostVLoopCtlPID(void);
void UserADC1_Init(void);
void control_calc(void);

extern CONTROL_STRUCT in, chassis, full_bridge_in, cap;
extern uint16_t pid_seesee;
extern struct ParameterBridge dcdcinfo;
extern volatile fp32 pid_final_buck;
extern volatile fp32 pid_final_boost;

#ifdef __cplusplus
}
#endif

#endif /*_POWER_CALC_*/
