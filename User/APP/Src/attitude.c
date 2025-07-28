/**
 * @file     attitude.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 15:25:38
 */

/* Your code here */
#include "attitude.h"
static uint32_t attitude_constrain(uint32_t value, uint32_t min, uint32_t max);
static void attitude_sensor_convert(bmi088_data_struct *bmi088_data);
#if ATTITUDE_USING_QUATERNION
static float attitude_inv_sqrt(float number);
static void attitude_quaternion_to_euler(attitude_quaternion_struct *quaternion, attitude_data_struct *attitude_data);
void attitude_mahony_angle_calculate(bmi088_data_struct *bmi088_data, attitude_quaternion_struct *quaternion, attitude_data_struct *attitude_data)
{
    float norm;
    float vx, vy, vz; // v: estimated gravity vector
    float ex, ey, ez; // error
    float accel_norm_sq;
    // 0. 单位换算
    attitude_sensor_convert(bmi088_data);
    accel_norm_sq = bmi088_data->accl[X] * bmi088_data->accl[X] + bmi088_data->accl[Y] * bmi088_data->accl[Y] + bmi088_data->accl[Z] * bmi088_data->accl[Z];
    // 1. 如果加速度计的读数有效 (模长不为0且不过大)
    if (accel_norm_sq > 0.01f)
    {
        // 2. 将加速度计向量归一化，得到实际测量的重力方向
        norm = attitude_inv_sqrt(accel_norm_sq);
        bmi088_data->accl[X] *= norm;
        bmi088_data->accl[Y] *= norm;
        bmi088_data->accl[Z] *= norm;

        // 3. 将四元数代表的姿态中的重力向量(0,0,1)旋转到机体坐标系下
        //    v = R_body_to_world' * [0, 0, 1]'
        vx = 2.0f * (quaternion->q[1] * quaternion->q[3] - quaternion->q[0] * quaternion->q[2]);
        vy = 2.0f * (quaternion->q[0] * quaternion->q[1] + quaternion->q[2] * quaternion->q[3]);
        vz = quaternion->q[0] * quaternion->q[0] - quaternion->q[1] * quaternion->q[1] - quaternion->q[2] * quaternion->q[2] + quaternion->q[3] * quaternion->q[3];

        // 4. 计算测量值和估计值之间的误差
        //    误差向量 e 是两个向量的叉积
        ex = (bmi088_data->accl[Y] * vz - bmi088_data->accl[Z] * vy);
        ey = (bmi088_data->accl[Z] * vx - bmi088_data->accl[X] * vz);
        ez = (bmi088_data->accl[X] * vy - bmi088_data->accl[Y] * vx);

        // 5. 对误差进行PI控制器计算，修正陀螺仪的角速度
        if (quaternion->mahony_ki > 0.0f)
        {
            quaternion->integral_e[0] += ex * ATTITUDE_DT;
            quaternion->integral_e[1] += ey * ATTITUDE_DT;
            quaternion->integral_e[2] += ez * ATTITUDE_DT;
        }
        else
        {
            quaternion->integral_e[0] = 0.0f;
            quaternion->integral_e[1] = 0.0f;
            quaternion->integral_e[2] = 0.0f;
        }
        bmi088_data->gyro[X] += quaternion->mahony_kp * ex + quaternion->mahony_ki * quaternion->integral_e[0];
        bmi088_data->gyro[Y] += quaternion->mahony_kp * ey + quaternion->mahony_ki * quaternion->integral_e[1];
        bmi088_data->gyro[Z] += quaternion->mahony_kp * ez + quaternion->mahony_ki * quaternion->integral_e[2];
    }

    // 6. 使用修正后的角速度，通过一阶龙格-库塔法更新四元数
    //    dq/dt = 0.5 * q * omega
    quaternion->q[0] += (-quaternion->q[1] * bmi088_data->gyro[X] - quaternion->q[2] * bmi088_data->gyro[Y] - quaternion->q[3] * bmi088_data->gyro[Z]) * 0.5f * ATTITUDE_DT;
    quaternion->q[1] += (quaternion->q[0] * bmi088_data->gyro[X] + quaternion->q[2] * bmi088_data->gyro[Z] - quaternion->q[3] * bmi088_data->gyro[Y]) * 0.5f * ATTITUDE_DT;
    quaternion->q[2] += (quaternion->q[0] * bmi088_data->gyro[Y] - quaternion->q[1] * bmi088_data->gyro[Z] + quaternion->q[3] * bmi088_data->gyro[X]) * 0.5f * ATTITUDE_DT;
    quaternion->q[3] += (quaternion->q[0] * bmi088_data->gyro[Z] + quaternion->q[1] * bmi088_data->gyro[Y] - quaternion->q[2] * bmi088_data->gyro[X]) * 0.5f * ATTITUDE_DT;

    // 7. 将四元数归一化，防止计算误差累积
    norm = attitude_inv_sqrt(quaternion->q[0] * quaternion->q[0] + quaternion->q[1] * quaternion->q[1] + quaternion->q[2] * quaternion->q[2] + quaternion->q[3] * quaternion->q[3]);
    quaternion->q[0] *= norm;
    quaternion->q[1] *= norm;
    quaternion->q[2] *= norm;
    quaternion->q[3] *= norm;

    // 8. 将四元数转换为欧拉角
    attitude_quaternion_to_euler(quaternion, attitude_data);
}
static float attitude_inv_sqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5f;

    x2 = number * 0.5f;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));
    return y;
}
static void attitude_quaternion_to_euler(attitude_quaternion_struct *quaternion, attitude_data_struct *attitude_data)
{
    float sq_q1 = quaternion->q[1] * quaternion->q[1];
    float sq_q2 = quaternion->q[2] * quaternion->q[2];
    float sq_q3 = quaternion->q[3] * quaternion->q[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (quaternion->q[0] * quaternion->q[1] + quaternion->q[2] * quaternion->q[3]);
    float cosr_cosp = 1.0f - 2.0f * (sq_q1 + sq_q2);
    attitude_data->angle[ROLL] = atan2f(sinr_cosp, cosr_cosp) * (180.0f / ATTITUDE_PI);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (quaternion->q[0] * quaternion->q[2] - quaternion->q[3] * quaternion->q[1]);
    if (fabsf(sinp) >= 1)
        attitude_data->angle[PITCH] = copysignf(ATTITUDE_PI / 2, sinp) * (180.0f / ATTITUDE_PI); // Use 90 degrees if out of range
    else
        attitude_data->angle[PITCH] = asinf(sinp) * (180.0f / ATTITUDE_PI);

    // Yaw (z-axis rotation)
    // float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    // float cosy_cosp = 1.0f - 2.0f * (sq_q2 + sq_q3);
    // attitude_data->angle[YAW] = atan2f(siny_cosp, cosy_cosp) * (180.0f / ATTITUDE_PI);
}
#else
void attitude_angle_calculate(attitude_data_struct *attitude_data, bmi088_data_struct *bmi088_data)
{
    // 1. 单位换算
    attitude_sensor_convert(bmi088_data);
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
#endif /* ATTITUDE_USING_QUATERNION */
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
void attitude_reset(attitude_data_struct *attitude_data)
{
    memset(attitude_data, 0, sizeof(attitude_data_struct));
}
static void attitude_sensor_convert(bmi088_data_struct *bmi088_data)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        // x y z
        bmi088_data->accl[i] = ((float)bmi088_data->accl_sample[i]) * ATTITUDE_ACC_SENS; // g
    }
    for (uint8_t i = 0; i < 3; i++)
    {
#if ATTITUDE_USING_QUATERNION
        bmi088_data->gyro[i] = ((float)bmi088_data->gyro_sample[i]) * ATTITUDE_GYRO_SENS * ATTITUDE_PI / 180.0f; // rad/s
#else
        bmi088_data->gyro[i] = ((float)bmi088_data->gyro_sample[i]) * ATTITUDE_GYRO_SENS; // °/s
#endif
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
    // attitude_data->yaw_out = 0; // disable yaw control for now
    attitude_data->pwm[0] = attitude_data->height_out + attitude_data->throttle - attitude_data->pitch_rate_out - attitude_data->roll_rate_out;
    attitude_data->pwm[1] = attitude_data->height_out + attitude_data->throttle + attitude_data->pitch_rate_out - attitude_data->roll_rate_out;
    attitude_data->pwm[2] = attitude_data->height_out + attitude_data->throttle - attitude_data->pitch_rate_out + attitude_data->roll_rate_out;
    attitude_data->pwm[3] = attitude_data->height_out + attitude_data->throttle + attitude_data->pitch_rate_out + attitude_data->roll_rate_out;
    for (uint8_t i = 0; i < 4; i++)
    {
        attitude_data->pwm[i] = attitude_constrain(attitude_data->pwm[i], 0, 2000); // constrain pwm value
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
    attitude_data->height_out = attitude_constrain(attitude_data->height_out, 0, 400); // constrain height control value
}
static uint32_t attitude_constrain(uint32_t value, uint32_t min, uint32_t max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}