#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/**
 * @file     state_machine.h
 * @brief
 * @author   Mr.Five
 * @date     2025-07-05 13:53:38
 */
/**
 * 无人机飞控状态机
 */
/**
 * @brief 定义无人机的主要运行状态
 */
typedef enum
{
    STATE_INIT,        // 系统初始化
    STATE_CALIBRATING, // 等待传感器校准
    STATE_IDLE,        // 已就绪，等待起飞指令
    STATE_FLYING,      // 飞行中，控制环路已激活
    STATE_ERROR        // 发生严重错误，安全停机
} quadcopter_state_enum;
/**
 * @brief 定义驱动状态机改变的事件
 */
typedef enum
{
    EVT_NONE,           // 无事件
    EVT_CMD_TAKE_OFF,   // 收到“起飞”指令事件
    EVT_CMD_LAND,       // 收到“降落”指令事件
    EVT_CRITICAL_ERROR, // 发生严重错误事件
    EVT_KEEP_ALIVE,     // 收到“心跳”事件
} state_machine_event_enum;

#endif /* STATE_MACHINE_H */