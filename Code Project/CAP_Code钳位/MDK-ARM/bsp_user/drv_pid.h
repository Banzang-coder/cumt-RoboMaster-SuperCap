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
//    uint8_t mode;   //PID模式，PID_POSITION位置式和PID_DELTA增量式
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;   //最大输出
    fp32 min_out;   //最输出
    fp32 max_iout;  //最大积分输出
    fp32 min_iout;  //最小积分输出
    fp32 set;       //设定值
    fp32 fdb;

    fp32 out;
    fp32 pout;
    fp32 iout;
    fp32 dout;
    fp32 dbuf[2];
    fp32 error[2];  //误差项 0最新 1上一次 2上上次

} pid_type_def;

// 定义PID参数结构体
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
		float setvalue;
		float realvalue;
    float integral;    // 积分项累积值
    float integral_max; // 积分项最大值
    float integral_min; // 积分项最小值
    float prev_error;  // 上一次的误差
    float prev_output; // 上一次的输出
    float output;      // 当前的输出
    float output_max;  // 输出最大值
    float output_min;  // 输出最小值
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
