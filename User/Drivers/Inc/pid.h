#ifndef PID_H
#define PID_H

/**
 * @file     pid.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 19:33:47
 */
typedef enum
{
    PID_PITCH = 0,
    PID_ROLL,
    PID_YAW,
    PID_PITCH_RATE,
    PID_ROLL_RATE,
    PID_YAW_RATE,
    PID_HEIGHT,
    PID_NUM
} pid_enum;
typedef struct
{
    float kp;
    float ki;
    float kd;

    float target; // 目标值
    float integral;
    float last_error;

    float output_limit;   // 输出限幅（如 ±1000）
    float integral_limit; // 积分限幅
} pid_data_struct;
void pid_init(pid_enum index, float kp, float ki, float kd,
              float integral_limit, float output_limit);
pid_data_struct *pid_get_data(void);
float pid_update(pid_enum index, float measurement, float dt);
void pid_set_target(pid_enum index, float target);
#endif /* PID_H */