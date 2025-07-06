/**
 * @file     attitude.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 15:25:38
 */

/* Your code here */
#include "attitude.h"
/**
 * global variables
 */
static uint32_t constrain(uint32_t value, uint32_t min, uint32_t max);
void attitude_enable_pwm(uint8_t enable)
{
    if (enable)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            HAL_TIM_PWM_Start(&htim2, i * 4);
        }
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            HAL_TIM_PWM_Stop(&htim2, i * 4);
        }
    }
}
void attitude_init(attitude_data_struct *attitude_data, uint32_t throttle)
{
    attitude_data->throttle = throttle;
}
void attitude_angle_calculate(attitude_data_struct *attitude_data, bmi088_data_struct *bmi088_data)
{
    // 1. 单位换算
    for (uint8_t i = 0; i < 3; i++)
    {
        // x y zd
        bmi088_data->accl[i] = ((float)bmi088_data->accl_sample[i]) * ATTITUDE_ACC_SENS; // g
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        // x y z
        bmi088_data->gyro[i] = ((float)bmi088_data->gyro_sample[i]) * ATTITUDE_GYRO_SENS; // °/s
    }

    // 2. 加速度解算角度（单位：°）
    // accl 0pitch ,1roll
    attitude_data->angle[ACCEL_PITCH] = atan2(-bmi088_data->accl[X], sqrt(bmi088_data->accl[Y] * bmi088_data->accl[Y] + bmi088_data->accl[Z] * bmi088_data->accl[Z])) * 180.0f / ATTITUDE_PI;
    // attitude_data->angle[ACCEL_PITCH] = atan2(bmi088_data->accl[X], bmi088_data->accl[Z]) * 180.0f / ATTITUDE_PI;
    attitude_data->angle[ACCEL_ROLL] = atan2(bmi088_data->accl[Y], bmi088_data->accl[Z]) * 180.0f / ATTITUDE_PI;

    // 3. 陀螺仪积分（角速度 × 时间 = 角度）
    // gyro 2pitch , 3roll
    attitude_data->angle[GYRO_PITCH] += bmi088_data->gyro[Y] * ATTITUDE_DT;
    attitude_data->angle[GYRO_ROLL] += bmi088_data->gyro[X] * ATTITUDE_DT;
    attitude_data->angle[YAW] += bmi088_data->gyro[Z] * ATTITUDE_DT;

    // 4. 互补滤波融合
    attitude_data->angle[PITCH] = ATTITUDE_ALPHA * attitude_data->angle[GYRO_PITCH] + (1 - ATTITUDE_ALPHA) * attitude_data->angle[ACCEL_PITCH];
    attitude_data->angle[ROLL] = ATTITUDE_ALPHA * attitude_data->angle[GYRO_ROLL] + (1 - ATTITUDE_ALPHA) * attitude_data->angle[ACCEL_ROLL];

    // 5.constrain angle value
    for (uint8_t i = 4; i < 6; i++)
    {
        if (attitude_data->angle[i] > ATTITUDE_ANGLE_LIMIT)
            attitude_data->angle[i] = ATTITUDE_ANGLE_LIMIT;
        else if (attitude_data->angle[i] < -ATTITUDE_ANGLE_LIMIT)
            attitude_data->angle[i] = -ATTITUDE_ANGLE_LIMIT;
    }

    // 6. recover gyro angle
    if (fabs(bmi088_data->gyro[X]) < ATTITUDE_GYRO_STATIC_THRESH && fabs(bmi088_data->gyro[Y]) < ATTITUDE_GYRO_STATIC_THRESH)
    {
        attitude_data->angle[GYRO_PITCH] += (attitude_data->angle[ACCEL_PITCH] - attitude_data->angle[GYRO_PITCH]) * ATTITUDE_DT;
        attitude_data->angle[GYRO_ROLL] += (attitude_data->angle[ACCEL_ROLL] - attitude_data->angle[GYRO_ROLL]) * ATTITUDE_DT;
    }
}
void attitude_motor_control(attitude_data_struct *attitude_data, bmi088_data_struct *bmi088_data, pid_data_struct *pid_pitch, pid_data_struct *pid_roll, pid_data_struct *pid_pitch_rate, pid_data_struct *pid_roll_rate)
{
    attitude_data->pitch_out = pid_update(pid_pitch, attitude_data->angle[PITCH], ATTITUDE_DT);
    attitude_data->roll_out = pid_update(pid_roll, attitude_data->angle[ROLL], ATTITUDE_DT);
    pid_pitch_rate->setpoint = attitude_data->pitch_out; // setpoint for pitch rate
    pid_roll_rate->setpoint = attitude_data->roll_out;   // setpoint for roll rate
    attitude_data->pitch_rate_out = pid_update(pid_pitch_rate, bmi088_data->gyro[Y], ATTITUDE_DT);
    attitude_data->roll_rate_out = pid_update(pid_roll_rate, bmi088_data->gyro[X], ATTITUDE_DT);
    //attitude_data->yaw_out = 0; // disable yaw control for now
    attitude_data->pwm[0] = attitude_data->height_out + attitude_data->throttle - attitude_data->pitch_rate_out - attitude_data->roll_rate_out;
    attitude_data->pwm[1] = attitude_data->height_out + attitude_data->throttle + attitude_data->pitch_rate_out - attitude_data->roll_rate_out;
    attitude_data->pwm[2] = attitude_data->height_out + attitude_data->throttle - attitude_data->pitch_rate_out + attitude_data->roll_rate_out;
    attitude_data->pwm[3] = attitude_data->height_out + attitude_data->throttle + attitude_data->pitch_rate_out + attitude_data->roll_rate_out;
    for (uint8_t i = 0; i < 4; i++)
    {
        attitude_data->pwm[i] = constrain(attitude_data->pwm[i], 0, 2000); // constrain pwm value
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        // HAL_TIM_PWM_Start(&htim2, i * 4);
        __HAL_TIM_SET_COMPARE(&htim2, i * 4, attitude_data->pwm[i]);
    }
}
void attitude_height_control(attitude_data_struct *attitude_data, spl06_data_struct *spl06_data, pid_data_struct *pid_height)
{
    attitude_data->height_out -= pid_update(pid_height, spl06_data->pressure, ATTITUDE_DT_50HZ);
    attitude_data->height_out = constrain(attitude_data->height_out, 0, 400); // constrain height control value
}
static uint32_t constrain(uint32_t value, uint32_t min, uint32_t max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}
void attitude_reset(attitude_data_struct *attitude_data)
{
    memset(attitude_data, 0, sizeof(attitude_data_struct));
}