#ifndef _POWER_CALC_
#define _POWER_CALC_

#ifdef __cplusplus
extern "C"
{
#endif
	
#include "function.h"
#include "struct_typedef.h"

#define ADC1_CHANNEL_CNT 5 	//����ͨ����
#define ADC1_CHANNEL_FRE 5	//����ͨ����������������ȡƽ��ֵ
#define board_version (0)   //������ţ����ĸ����Ӿ����Ǹ�
#define LPF_ENABLE    (0)   //��ͨ�˲�ʹ��
	
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
  uint8_t data_update;    //1����
  uint8_t loop_mode;     	//��·ģʽ0��cv 1��cc 2��cp
  float efficiency;      	//��Ч�� ���Ⱥ�ɢ
  uint8_t power_dir;     	//��������0������1����ŵ�
  uint16_t max_power_lim; //����ϵͳ�����
  float power_pass;				//����ϵͳ��ȥ���ʰ�����Ӳ�������pid������ڴ���
  float powerbuffer ;			//��������
} BUCKBOOST_STRUCT;

void BuckBoostVLoopCtlPID(void);
void UserADC1_Init(void);
void control_calc(void);

extern CONTROL_STRUCT in, chassis, full_bridge_in, cap;
extern struct ParameterBridge dcdcinfo;
extern volatile fp32 pid_final_buck;
extern volatile fp32 pid_final_boost;
extern volatile fp32 volt_ratio;
extern volatile fp32 volt_ratio_filter;

extern struct ParameterBridge dcdctest;

#ifdef __cplusplus
}
#endif

#endif /*_POWER_CALC_*/
