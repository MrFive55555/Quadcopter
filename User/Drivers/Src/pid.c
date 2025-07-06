/**
 * @file     pid.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 19:33:39
 */

#include "pid.h"

/* Your code here */
void pid_init(pid_data_struct *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;

    pid->setpoint = 0;
    pid->integral = 0;
    pid->last_error = 0;
}
float pid_update(pid_data_struct *pid, float measurement, float dt)
{
    float error = pid->setpoint - measurement;
    pid->integral += error * dt;

    // 积分限幅
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if (output > pid->output_limit)
        output = pid->output_limit;
    if (output < -pid->output_limit)
        output = -pid->output_limit;

    return output;
}
void pid_reset(pid_data_struct *pid)
{
    pid->setpoint = 0;
    pid->integral = 0;
    pid->last_error = 0;
    // 不重置 kp, ki, kd, output_limit, integral_limit
}