/**
 * @file     pid.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 19:33:39
 */

#include "pid.h"
static pid_data_struct pid_data[PID_NUM];
pid_data_struct* pid_get_data(void){
    return pid_data;
}
void pid_init(pid_enum index, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid_data[index].kp = kp;
    pid_data[index].ki = ki;
    pid_data[index].kd = kd;
    pid_data[index].integral_limit = integral_limit;
    pid_data[index].output_limit = output_limit;

    pid_data[index].target = 0;
    pid_data[index].integral = 0;
    pid_data[index].last_error = 0;
}
float pid_update(pid_enum index, float measurement, float dt)
{
    float error = pid_data[index].target - measurement;
    pid_data[index].integral += error * dt;

    // 积分限幅
    if (pid_data[index].integral > pid_data[index].integral_limit)
        pid_data[index].integral = pid_data[index].integral_limit;
    if (pid_data[index].integral < -pid_data[index].integral_limit)
        pid_data[index].integral = -pid_data[index].integral_limit;

    float derivative = (error - pid_data[index].last_error) / dt;
    pid_data[index].last_error = error;

    float output = pid_data[index].kp * error + pid_data[index].ki * pid_data[index].integral + pid_data[index].kd * derivative;

    // 输出限幅
    if (output > pid_data[index].output_limit)
        output = pid_data[index].output_limit;
    if (output < -pid_data[index].output_limit)
        output = -pid_data[index].output_limit;

    return output;
}
void pid_set_target(pid_enum index, float target)
{
    pid_data[index].target = target;
}