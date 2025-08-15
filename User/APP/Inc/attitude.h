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
    float yaw_out;
    float pitch_rate_out; // 角速度输出
    float roll_rate_out;  // 角速度输出
    float yaw_rate_out;   // 角速度输出
    uint32_t height_out;  // height control value
    uint32_t pwm[4];      // motor pwm value
    int16_t throttle;
} attitude_data_struct;
attitude_data_struct *attitude_get_data(void);
void attitude_init(float mahony_kp, float mahony_ki);
void attitude_enable_pwm(uint8_t enable);
void attitude_target_set(int16_t *target);
void attitude_throttle_add(int16_t throttle);
void attitude_motor_control(bmi088_data_struct *bmi088_data);
void attitude_height_control(spl06_data_struct *spl06_data);
#ifdef ATTITUDE_USING_QUATERNION
typedef struct
{
    float q[4];          // w,x,y,z
    float integral_e[3]; // ex,ey,ez
    float mahony_kp;
    float mahony_ki;
} attitude_quaternion_struct;
void attitude_mahony_angle_calculate(bmi088_data_struct *bmi088_data);
#else
void attitude_angle_calculate(bmi088_data_struct *bmi088_data);
#endif /* ATTITUDE_USING_QUATERNION */

#endif /* ATTITUDE_H */