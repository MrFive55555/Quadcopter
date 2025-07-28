#ifndef ATTITUDE_H
#define ATTITUDE_H

/**
 * @file     attitude.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 15:25:47
 */
#include <math.h>
#include <string.h>
#include "bmi088.h"
#include "spl06.h"
#include "pid.h"
/**
 * code segments
 */
#define ATTITUDE_USING_QUATERNION 1
/**
 * global variables
 */
#define ATTITUDE_DT (0.000625f * 4) // 采样周期，单位：s
#define ATTITUDE_DT_50HZ 0.02f      // 50Hz采样周期，单位：s
#define ATTITUDE_PI 3.1415926f
#define ATTITUDE_GYRO_SENS (2000.0f / 32768.0f) // ±2000°/s，对应比例
#define ATTITUDE_ACC_SENS (1.0f / 10920.0f)     // ±3g, based on datasheet sensitivity 10920 LSB/g
#define ATTITUDE_ALPHA 0.98f                    // 互补滤波系数
#define ATTITUDE_ANGLE_LIMIT 90.0f              // 角度限制，单位：°
#define ATTITUDE_GYRO_STATIC_THRESH 0.5f        // 陀螺仪静止阈值，单位：°/s
typedef enum
{
    ACCEL_PITCH = 0,
    ACCEL_ROLL,
    GYRO_PITCH,
    GYRO_ROLL,
    PITCH,
    ROLL,
    YAW,
} attitude_angle_enum;
typedef struct
{
    // 0: ACCEL_PITCH, 1: ACCEL_ROLL, 2: GYRO_PITCH, 3: GYRO_ROLL, 4: PITCH, 5: ROLL, 6: YAW
    float angle[7]; // 姿态角度数据
    float pitch_out;
    float roll_out;
    // float yaw_out;
    float pitch_rate_out; // 角速度输出
    float roll_rate_out;  // 角速度输出
    uint32_t height_out;  // height control value
    uint32_t pwm[4];      // motor pwm value
    uint32_t throttle;
} attitude_data_struct;
void attitude_enable_pwm(uint8_t enable);
void attitude_init(attitude_data_struct *attitude_data, uint32_t throttle);
void attitude_reset(attitude_data_struct *attitude_data);
void attitude_motor_control(attitude_data_struct *attitude_data, bmi088_data_struct *bmi088_data, pid_data_struct *pid_pitch, pid_data_struct *pid_roll, pid_data_struct *pid_pitch_rate, pid_data_struct *pid_roll_rate);
void attitude_height_control(attitude_data_struct *attitude_data, spl06_data_struct *spl06_data, pid_data_struct *pid_height);
#ifdef ATTITUDE_USING_QUATERNION
typedef struct
{
    float q[4];          // w,x,y,z
    float integral_e[3]; // ex,ey,ez
    float mahony_kp;
    float mahony_ki;
} attitude_quaternion_struct;
void attitude_mahony_angle_calculate(bmi088_data_struct *bmi088_data, attitude_quaternion_struct *quaternion, attitude_data_struct *attitude_data);
#else
void attitude_angle_calculate(attitude_data_struct *attitude_data, bmi088_data_struct *bmi088_data);
#endif /* ATTITUDE_USING_QUATERNION */

#endif /* ATTITUDE_H */