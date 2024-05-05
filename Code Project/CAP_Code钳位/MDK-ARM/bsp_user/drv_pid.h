#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "struct_typedef.h"

enum PID_MODE
{
    PID_POSITION = 0, PID_DELTA = 1
};

typedef struct
{
//    uint8_t mode;   //PIDģʽ��PID_POSITIONλ��ʽ��PID_DELTA����ʽ
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;   //������
    fp32 min_out;   //�����
    fp32 max_iout;  //���������
    fp32 min_iout;  //��С�������
    fp32 set;       //�趨ֵ
    fp32 fdb;

    fp32 out;
    fp32 pout;
    fp32 iout;
    fp32 dout;
    fp32 dbuf[2];
    fp32 error[2];  //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;

// ����PID�����ṹ��
typedef struct {
    float Kp;          // ����ϵ��
    float Ki;          // ����ϵ��
    float Kd;          // ΢��ϵ��
		float setvalue;
		float realvalue;
    float integral;    // �������ۻ�ֵ
    float integral_max; // ���������ֵ
    float integral_min; // ��������Сֵ
    float prev_error;  // ��һ�ε����
    float prev_output; // ��һ�ε����
    float output;      // ��ǰ�����
    float output_max;  // ������ֵ
    float output_min;  // �����Сֵ
} IncrementalPID;

extern void PID_init(pid_type_def *pid, const fp32 p, const fp32 i, const fp32 d, fp32 max_out,fp32 max_iout, fp32 min_out, fp32 min_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);



extern void IncrementalPID_Init(IncrementalPID *pid, float kp, float ki, float kd,float output_init,
                         float integral_max, float integral_min,
                         float output_max, float output_min);
extern void IncrementalPID_Clear(IncrementalPID *pid);
extern float IncrementalPID_Compute(IncrementalPID *pid, float setpoint, float measured_value);
extern void IncrementalPID_UpdateControlOutput(float *control_output, float output_increment);
extern void IncrementalPID_ClearWithoutOutput(IncrementalPID *pid);

extern pid_type_def voltageout_loop;
extern pid_type_def currentout_loop;
extern pid_type_def powerin_loop;
extern pid_type_def powerbuf_loop;
extern pid_type_def power_referee_loop;

extern IncrementalPID currentout_Iloop;
extern IncrementalPID voltageout_Vloop;
extern IncrementalPID powerin_Ploop;
extern IncrementalPID Nchargelimit;
#ifdef __cplusplus
}
#endif

#endif
