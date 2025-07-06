#ifndef PID_H
#define PID_H

/**
 * @file     pid.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 19:33:47
 */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float setpoint; // 目标值
    float integral;
    float last_error;

    float output_limit;   // 输出限幅（如 ±1000）
    float integral_limit; // 积分限幅
} pid_data_struct;
void pid_init(pid_data_struct *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);

float pid_update(pid_data_struct *pid, float measurement, float dt);
void pid_reset(pid_data_struct *pid);
#endif /* PID_H */