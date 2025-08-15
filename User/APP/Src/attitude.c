/**
 * @file     attitude.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-09 15:25:38
 */

/* Your code here */
#include "attitude.h"
/**
 * global variables of attitude
 */
static float attitude_inv_sqrt(float number);
static void attitude_quaternion_to_euler(void);
static attitude_data_struct attitude_data;
static uint32_t attitude_constrain(uint32_t value, uint32_t min, uint32_t max);
#if ATTITUDE_USING_QUATERNION
static attitude_quaternion_struct attitude_quaternion;
void attitude_mahony_angle_calculate(bmi088_data_struct *bmi088_data)
{
    float norm;
    float vx, vy, vz; // v: estimated gravity vector
    float ex, ey, ez; // error
    float accel_norm_sq;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    accel_x = bmi088_data->accl[X];
    accel_y = bmi088_data->accl[Y];
    accel_z = bmi088_data->accl[Z];
    gyro_x = bmi088_data->gyro[X] * ATTITUDE_PI / 180.0f; // 转换为弧度 rad/s
    gyro_y = bmi088_data->gyro[Y] * ATTITUDE_PI / 180.0f;
    gyro_z = bmi088_data->gyro[Z] * ATTITUDE_PI / 180.0f;
    accel_norm_sq = accel_x * accel_x + accel_y * accel_y + accel_z * accel_z;
    // 1. 如果加速度计的读数有效 (模长不为0且不过大)
    if (accel_norm_sq > 0.01f)
    {
        // 2. 将加速度计向量归一化，得到实际测量的重力方向
        norm = attitude_inv_sqrt(accel_norm_sq);
        accel_x *= norm;
        accel_y *= norm;
        accel_z *= norm;

        // 3. 将四元数代表的姿态中的重力向量(0,0,1)旋转到机体坐标系下
        //    v = R_body_to_world' * [0, 0, 1]'
        vx = 2.0f * (attitude_quaternion.q[1] * attitude_quaternion.q[3] - attitude_quaternion.q[0] * attitude_quaternion.q[2]);
        vy = 2.0f * (attitude_quaternion.q[0] * attitude_quaternion.q[1] + attitude_quaternion.q[2] * attitude_quaternion.q[3]);
        vz = attitude_quaternion.q[0] * attitude_quaternion.q[0] - attitude_quaternion.q[1] * attitude_quaternion.q[1] - attitude_quaternion.q[2] * attitude_quaternion.q[2] + attitude_quaternion.q[3] * attitude_quaternion.q[3];

        // 4. 计算测量值和估计值之间的误差
        //    误差向量 e 是两个向量的叉积
        ex = (accel_y * vz - accel_z * vy);
        ey = (accel_z * vx - accel_x * vz);
        ez = (accel_x * vy - accel_y * vx);

        // 5. 对误差进行PI控制器计算，修正陀螺仪的角速度
        if (attitude_quaternion.mahony_ki > 0.0f)
        {
            attitude_quaternion.integral_e[0] += ex * ATTITUDE_DT;
            attitude_quaternion.integral_e[1] += ey * ATTITUDE_DT;
            attitude_quaternion.integral_e[2] += ez * ATTITUDE_DT;
        }
        else
        {
            attitude_quaternion.integral_e[0] = 0.0f;
            attitude_quaternion.integral_e[1] = 0.0f;
            attitude_quaternion.integral_e[2] = 0.0f;
        }
        gyro_x += attitude_quaternion.mahony_kp * ex + attitude_quaternion.mahony_ki * attitude_quaternion.integral_e[0];
        gyro_y += attitude_quaternion.mahony_kp * ey + attitude_quaternion.mahony_ki * attitude_quaternion.integral_e[1];
        gyro_z += attitude_quaternion.mahony_kp * ez + attitude_quaternion.mahony_ki * attitude_quaternion.integral_e[2];
    }

    // 6. 使用修正后的角速度，通过一阶龙格-库塔法更新四元数
    //    dq/dt = 0.5 * q * omega
    attitude_quaternion.q[0] += (-attitude_quaternion.q[1] * gyro_x - attitude_quaternion.q[2] * gyro_y - attitude_quaternion.q[3] * gyro_z) * 0.5f * ATTITUDE_DT;
    attitude_quaternion.q[1] += (attitude_quaternion.q[0] * gyro_x + attitude_quaternion.q[2] * gyro_z - attitude_quaternion.q[3] * gyro_y) * 0.5f * ATTITUDE_DT;
    attitude_quaternion.q[2] += (attitude_quaternion.q[0] * gyro_y - attitude_quaternion.q[1] * gyro_z + attitude_quaternion.q[3] * gyro_x) * 0.5f * ATTITUDE_DT;
    attitude_quaternion.q[3] += (attitude_quaternion.q[0] * gyro_z + attitude_quaternion.q[1] * gyro_y - attitude_quaternion.q[2] * gyro_x) * 0.5f * ATTITUDE_DT;

    // 7. 将四元数归一化，防止计算误差累积
    norm = attitude_inv_sqrt(attitude_quaternion.q[0] * attitude_quaternion.q[0] + attitude_quaternion.q[1] * attitude_quaternion.q[1] + attitude_quaternion.q[2] * attitude_quaternion.q[2] + attitude_quaternion.q[3] * attitude_quaternion.q[3]);
    attitude_quaternion.q[0] *= norm;
    attitude_quaternion.q[1] *= norm;
    attitude_quaternion.q[2] *= norm;
    attitude_quaternion.q[3] *= norm;

    // 8. 将四元数转换为欧拉角
    attitude_quaternion_to_euler();
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
static void attitude_quaternion_to_euler(void)
{
    float sq_q1 = attitude_quaternion.q[1] * attitude_quaternion.q[1];
    float sq_q2 = attitude_quaternion.q[2] * attitude_quaternion.q[2];
    float sq_q3 = attitude_quaternion.q[3] * attitude_quaternion.q[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (attitude_quaternion.q[0] * attitude_quaternion.q[1] + attitude_quaternion.q[2] * attitude_quaternion.q[3]);
    float cosr_cosp = 1.0f - 2.0f * (sq_q1 + sq_q2);
    attitude_data.angle[ROLL] = atan2f(sinr_cosp, cosr_cosp) * (180.0f / ATTITUDE_PI);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (attitude_quaternion.q[0] * attitude_quaternion.q[2] - attitude_quaternion.q[3] * attitude_quaternion.q[1]);
    if (fabsf(sinp) >= 1)
        attitude_data.angle[PITCH] = copysignf(ATTITUDE_PI / 2, sinp) * (180.0f / ATTITUDE_PI); // Use 90 degrees if out of range
    else
        attitude_data.angle[PITCH] = asinf(sinp) * (180.0f / ATTITUDE_PI);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (attitude_quaternion.q[0] * attitude_quaternion.q[3] + attitude_quaternion.q[1] * attitude_quaternion.q[2]);
    float cosy_cosp = 1.0f - 2.0f * (sq_q2 + sq_q3);
    attitude_data.angle[YAW] = atan2f(siny_cosp, cosy_cosp) * (180.0f / ATTITUDE_PI);
}
#else
void attitude_angle_calculate(bmi088_data_struct *bmi088_data)
{
    // 2. 加速度解算角度（单位：°）
    // accl 0pitch ,1roll
    attitude_data.angle[ACCEL_PITCH] = atan2(-bmi088_data->accl[X], sqrt(bmi088_data->accl[Y] * bmi088_data->accl[Y] + bmi088_data->accl[Z] * bmi088_data->accl[Z])) * 180.0f / ATTITUDE_PI;
    // attitude_data.angle[ACCEL_PITCH] = atan2(bmi088_data->accl[X], bmi088_data->accl[Z]) * 180.0f / ATTITUDE_PI;
    attitude_data.angle[ACCEL_ROLL] = atan2(bmi088_data->accl[Y], bmi088_data->accl[Z]) * 180.0f / ATTITUDE_PI;

    // 3. 陀螺仪积分（角速度 × 时间 = 角度）
    // gyro 2pitch , 3roll
    attitude_data.angle[GYRO_PITCH] += bmi088_data->gyro[Y] * ATTITUDE_DT;
    attitude_data.angle[GYRO_ROLL] += bmi088_data->gyro[X] * ATTITUDE_DT;
    attitude_data.angle[YAW] += bmi088_data->gyro[Z] * ATTITUDE_DT;

    // 4. 互补滤波融合
    attitude_data.angle[PITCH] = ATTITUDE_ALPHA * attitude_data.angle[GYRO_PITCH] + (1 - ATTITUDE_ALPHA) * attitude_data.angle[ACCEL_PITCH];
    attitude_data.angle[ROLL] = ATTITUDE_ALPHA * attitude_data.angle[GYRO_ROLL] + (1 - ATTITUDE_ALPHA) * attitude_data.angle[ACCEL_ROLL];

    // 5.constrain angle value
    for (uint8_t i = 4; i < 6; i++)
    {
        if (attitude_data.angle[i] > ATTITUDE_ANGLE_LIMIT)
            attitude_data.angle[i] = ATTITUDE_ANGLE_LIMIT;
        else if (attitude_data.angle[i] < -ATTITUDE_ANGLE_LIMIT)
            attitude_data.angle[i] = -ATTITUDE_ANGLE_LIMIT;
    }

    // 6. recover gyro angle
    if (fabs(bmi088_data->gyro[X]) < ATTITUDE_GYRO_STATIC_THRESH && fabs(bmi088_data->gyro[Y]) < ATTITUDE_GYRO_STATIC_THRESH)
    {
        attitude_data.angle[GYRO_PITCH] += (attitude_data.angle[ACCEL_PITCH] - attitude_data.angle[GYRO_PITCH]) * ATTITUDE_DT;
        attitude_data.angle[GYRO_ROLL] += (attitude_data.angle[ACCEL_ROLL] - attitude_data.angle[GYRO_ROLL]) * ATTITUDE_DT;
    }
}
#endif /* ATTITUDE_USING_QUATERNION */
attitude_data_struct *attitude_get_data(void)
{
    return &attitude_data;
}
void attitude_init(float mahony_kp, float mahony_ki)
{
#ifdef ATTITUDE_USING_QUATERNION
    attitude_quaternion.q[0] = 1.0f;
    attitude_quaternion.mahony_kp = mahony_kp;
    attitude_quaternion.mahony_ki = mahony_ki;
#else
    return;
#endif /* ATTITUDE_USING_QUATERNION */
}
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
/**
 * target[0-3]=roll,pitch,throttle,yaw
 * tartget value is 1000 to 2000;
 * 1 degeree / 5 value
 * then range is-50 to 50.
 */
void attitude_target_set(int16_t *target)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (target[i] >= 1750)
        {
            target[i] = (target[i] - 1750) / 5; // Convert to degrees
        }
        else if (target[i] <= 1250)
        {
            target[i] = (target[i] - 1250) / 5;
        }
        else
        {
            target[i] = 0;
        }
    }
    pid_set_target(PID_ROLL, target[0]);
    pid_set_target(PID_PITCH, target[1]);
    attitude_throttle_add(target[2]);
    pid_set_target(PID_YAW, target[3]);
}
void attitude_throttle_add(int16_t throttle)
{
    attitude_data.throttle += throttle;
    if (attitude_data.throttle > 2000)
        attitude_data.throttle = 2000;
    else if (attitude_data.throttle < 0)
        attitude_data.throttle = 0;
}
void attitude_motor_control(bmi088_data_struct *bmi088_data)
{
    attitude_data.pitch_out = pid_update(PID_PITCH, attitude_data.angle[PITCH], ATTITUDE_DT);
    attitude_data.roll_out = pid_update(PID_ROLL, attitude_data.angle[ROLL], ATTITUDE_DT);
    attitude_data.yaw_out = pid_update(PID_YAW, attitude_data.angle[YAW], ATTITUDE_DT);
    pid_set_target(PID_PITCH_RATE, attitude_data.pitch_out);
    pid_set_target(PID_ROLL_RATE, attitude_data.roll_out);
    pid_set_target(PID_YAW_RATE, attitude_data.yaw_out);
    attitude_data.pitch_rate_out = pid_update(PID_PITCH_RATE, bmi088_data->gyro[Y], ATTITUDE_DT);
    attitude_data.roll_rate_out = pid_update(PID_ROLL_RATE, bmi088_data->gyro[X], ATTITUDE_DT);
    attitude_data.yaw_rate_out = pid_update(PID_YAW_RATE, bmi088_data->gyro[Z], ATTITUDE_DT);
    attitude_data.pwm[0] = attitude_data.height_out + attitude_data.throttle - attitude_data.pitch_rate_out - attitude_data.roll_rate_out + attitude_data.yaw_rate_out;
    attitude_data.pwm[1] = attitude_data.height_out + attitude_data.throttle + attitude_data.pitch_rate_out - attitude_data.roll_rate_out - attitude_data.yaw_rate_out;
    attitude_data.pwm[2] = attitude_data.height_out + attitude_data.throttle - attitude_data.pitch_rate_out + attitude_data.roll_rate_out - attitude_data.yaw_rate_out;
    attitude_data.pwm[3] = attitude_data.height_out + attitude_data.throttle + attitude_data.pitch_rate_out + attitude_data.roll_rate_out + attitude_data.yaw_rate_out;
    for (uint8_t i = 0; i < 4; i++)
    {
        attitude_data.pwm[i] = attitude_constrain(attitude_data.pwm[i], 0, 2000); // constrain pwm value
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        __HAL_TIM_SET_COMPARE(&htim2, i * 4, attitude_data.pwm[i]);
    }
}
void attitude_height_control(spl06_data_struct *spl06_data)
{
    attitude_data.height_out -= pid_update(PID_HEIGHT, spl06_data->pressure, ATTITUDE_DT_50HZ);
    attitude_data.height_out = attitude_constrain(attitude_data.height_out, 0, 400); // constrain height control value
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