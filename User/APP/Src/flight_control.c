/**
 * @file     flight_control.c
 * @brief
 * @author   Mr.Five
 * @date     2025-08-07 11:48:51
 */

#include "flight_control.h"
static void enter_error_state(void);
static volatile quadcopter_state_enum quadcopter_state = STATE_INIT;
quadcopter_state_enum quadcopter_state_get(void)
{
    return quadcopter_state;
}
volatile quadcopter_state_enum *quadcopter_state_get_ptr(void){
    return &quadcopter_state;
}
void quadcopter_state_set(quadcopter_state_enum state)
{
    quadcopter_state = state;
}
void flight_control_run(bmi088_data_struct *bmi088_data, spl06_data_struct *spl06_data, pid_data_struct *pid_data, state_machine_event_enum queue_sm_evt)
{
    static uint32_t bmi088_sync_counter = 0;
    static uint32_t spl06_sync_counter = 0;
    static uint32_t current_tick = 0;
    static const uint32_t HEARTBEAT_TIMEOUT_TICKS = 5000;
    static volatile uint32_t g_last_heartbeat_tick = 0;
    if (queue_sm_evt == EVT_CRITICAL_ERROR)
    {
        enter_error_state(); // 进入错误状态
    }
    switch (quadcopter_state)
    {
    case STATE_INIT: // 1.init state
        break;

    case STATE_CALIBRATING: // 2.calibration state
        if (bmi088_data->bias_ok_flag)
        {
            quadcopter_state = STATE_IDLE;
        }
        break;

    case STATE_IDLE: // 3.idle state
        if (queue_sm_evt == EVT_CMD_TAKE_OFF)
        {
            g_last_heartbeat_tick = osKernelGetTickCount();
            attitude_enable_pwm(1);
            quadcopter_state = STATE_FLYING;
        }
        break;

    case STATE_FLYING: // 4.flying state
        // 在飞行状态，优先处理“降落”指令
        if (queue_sm_evt == EVT_CMD_LAND)
        {
            attitude_enable_pwm(0);
            quadcopter_state = STATE_CALIBRATING;
            return; // 直接返回，避免后续处理
        }
        else if (queue_sm_evt == EVT_KEEP_ALIVE)
        {
            g_last_heartbeat_tick = osKernelGetTickCount(); // 更新心跳时间戳
        }
        current_tick = osKernelGetTickCount();
        // 检查当前时间与上次心跳时间的差值
        // if ((current_tick - g_last_heartbeat_tick) > HEARTBEAT_TIMEOUT_TICKS)
        // {
        //     // 超时发生！发送一个严重错误事件
        //     enter_error_state(); // 进入错误状态
        //     return;
        // }
        if (++bmi088_sync_counter >= 4) // 每4次循环处理一次传感器数据，约400Hz  
        {
            bmi088_sync_counter = 0; // 重置计数器
#ifdef ATTITUDE_USING_QUATERNION
            attitude_mahony_angle_calculate(bmi088_data);
#else
            attitude_angle_calculate(&attitude_data, &bmi088_data);
#endif /* ATTITUDE_USING_QUATERNION */
            attitude_motor_control(bmi088_data);
            // 高度控制以较低频率运行 (~50Hz)
            // if (++spl06_sync_counter >= 8)
            // {
            //     spl06_sync_counter = 0;
            //     if (pid_data[PID_HEIGHT].target > 0.1f || pid_data[PID_HEIGHT].target < -0.1f) // 仅当设定点大于0.1时才进行高度控制
            //     {
            //         attitude_height_control(spl06_data);
            //     }
            // }
        }
        break;

    case STATE_ERROR: // 5.error state
        // 终结状态，只闪烁错误灯
        light_control(LIGHT_ERROR, LIGHT_BLINK);
        osDelay(200);
        break;
    // 意外状态，直接进入错误状态以策安全
    default:
        enter_error_state();
        break;
    }
}
// Auxiliary function to safely enter the error state
static void enter_error_state(void)
{
    // 1. 安全第一：立即关闭所有电机
    attitude_enable_pwm(0);

    // 2. 强制将系统状态设置为错误
    quadcopter_state = STATE_ERROR;

    // 3. （可选）可以在此处记录具体的错误信息

    // 状态机的 ERROR case 将会接管后续的错误指示（如LED闪烁）
}