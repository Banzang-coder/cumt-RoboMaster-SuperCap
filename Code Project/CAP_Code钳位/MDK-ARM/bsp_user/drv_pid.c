#include "drv_pid.h"

#define LimitMax(input, max) \
  {                          \
    if (input > max)         \
    {                        \
      input = max;           \
    }                        \
    else if (input < -max)   \
    {                        \
      input = -max;          \
    }                        \
  }

pid_type_def voltageout_loop;
pid_type_def currentout_loop;
pid_type_def powerin_loop;
pid_type_def powerbuf_loop;
pid_type_def power_referee_loop;

/**
 * @brief          pid struct data init
 * @param[out]     pid: PID struct data point
 * @param[in]      mode: PID_POSITION: normal pid
 * @param[in]      PID: 0: kp, 1: ki, 2:kd
 * @param[in]      max_out: pid max out
 * @param[in]      max_iout: pid max iout
 * @retval         none
 */
void PID_init(pid_type_def *pid, const fp32 p, const fp32 i, const fp32 d,
			fp32 min_iout, fp32 max_iout, 
				fp32 min_out,fp32 max_out)
{
    if (pid == NULL || pid == NULL)
    {
        return;
    }
//    pid->mode = mode;
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->min_out = min_out;
    pid->min_iout = min_iout;
    pid->dbuf[0] = pid->dbuf[1] = 0.0f;
    pid->error[0] = pid->error[1] = 0.0f;
    pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}

/**
 * @brief          pid calculate
 * @param[out]     pid: PID struct data point
 * @param[in]      ref: feedback data
 * @param[in]      set: set point
 * @retval         pid out
 */
#define N_filter 24000

fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->pout = pid->Kp * pid->error[0];
    pid->iout += pid->Ki * pid->error[0];
    pid->dbuf[1] = pid->dbuf[0];
    pid->dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->dout = pid->Kd * pid->dbuf[0];
    LimitMax(pid->iout, pid->max_iout);
    pid->out = pid->pout + pid->iout + pid->dout;

    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    else if (pid->out < pid->min_out)
    {
        pid->out = pid->min_out;
    }

    return pid->out;
}

/**
 * @brief          pid out clear
 * @param[out]     pid: PID struct data point
 * @retval         none
 */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = 0.0f;
    pid->dbuf[0] = pid->dbuf[1] = 0.0f;
    pid->out = pid->pout = pid->iout = pid->dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**
 * @brief          ����ʽPID
 */

// ��ʼ������ʽPID������
void IncrementalPID_Init(IncrementalPID *pid, float kp, float ki, float kd,float output_init,
                         float integral_max, float integral_min,
                         float output_max, float output_min) 
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->prev_error = 0.0f;
    pid->prev_output = output_init;
    pid->output = output_init; // ��ʼ�����Ϊ0
    pid->output_max = output_max;
    pid->output_min = output_min;
}

// �������ʽPID�������Ĳ���
void IncrementalPID_Clear(IncrementalPID *pid) 
{
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->output = 0.0f;
}
	
// ��������ʽPID�������������޷�
float IncrementalPID_Compute(IncrementalPID *pid, float setpoint, float measured_value)
{
    // �������
    float error = setpoint - measured_value;

    // ���������
    pid->integral = pid->Ki * error;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    // ��������޷�
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }

    // ��������ʽPID�������
    float output_increment = pid->Kp * (error - pid->prev_error) + pid->integral - pid->Kd * (pid->output - pid->prev_output);

    // ������һ�ε��������
    pid->prev_error = error;
    pid->prev_output = pid->output; // ����Ϊ��ǰ�����

    // ���µ�ǰ�����
    pid->output += output_increment;

    // ��������������޷�
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }

    return pid->output; // ���ص�ǰ��PID���ֵ
}

// Ӧ������ʽPID���������ϵͳ
void IncrementalPID_UpdateControlOutput(float *control_output, float output_increment) 
{
    // �ڴ˴����ݿ���ϵͳ�����ͣ�������ơ��¶ȿ��Ƶȣ��������
    // ���磬������������ӵ����п������
    // control_output += output_increment;
}

// ����ʽPID����������������һ�ε������
void IncrementalPID_ClearWithoutOutput(IncrementalPID *pid) 
	{
    pid->Kp = 0.0f;
    pid->Ki = 0.0f;
    pid->Kd = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

IncrementalPID currentout_Iloop;
IncrementalPID voltageout_Vloop;
IncrementalPID powerin_Ploop;
IncrementalPID Nchargelimit;
