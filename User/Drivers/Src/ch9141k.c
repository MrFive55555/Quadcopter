/**
 * @file     ch9141k.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-16 19:49:39
 */

#include "ch9141k.h"
static void ch9141k_send_at_command(const char *cmd);
static void ch9141k_send_at_command(const char *cmd)
{
    char buffer[64];
    sprintf(buffer, "%s", cmd);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer),100);
}
void ch9141k_init(void)
{
    // 步骤 1: 硬件准备
    // 确保 SLEEP 引脚为高电平，使芯片处于工作状态
    CH9141K_WAKEUP(); // 假设这个函数将 SLEEP 引脚拉高

    // 将 AT 引脚拉低，进入 AT 命令模式
    CH9141K_AT_ENTRY();
    osDelay(100); // 等待芯片稳定

    // 步骤 2: 发送AT指令进行配置
    // (请确保您的单片机UART此时使用的是芯片默认或已知的波特率，出厂默认为115200)
    ch9141k_send_at_command("AT\r\n"); // 发送测试指令，检查是否返回 "OK"
    osDelay(100);

    ch9141k_send_at_command("AT+BLEMODE=2\r\n"); // 1. 明确设置为从机模式
    osDelay(100);

    ch9141k_send_at_command("AT+NAME=MyCH9141\r\n"); // 2. 设置一个蓝牙名称，方便手机识别
    osDelay(100);

    ch9141k_send_at_command("AT+UART=115200,8,1,0,50\r\n"); // 3. 设置正确的串口参数
    osDelay(100);

    ch9141k_send_at_command("AT+PASS=123456\r\n"); // 3. 设置正确的串口参数
    osDelay(100);

    // 步骤 3: 重启芯片使配置生效
    ch9141k_send_at_command("AT+RESET\r\n"); // 4. 发送复位指令
    osDelay(500);                  // 等待芯片重启完成

    // 步骤 4: 退出AT模式，进入透传模式
    // 此时芯片已经按照新的配置（从机模式，115200波特率）开始广播
    CH9141K_AT_EXIT(); // 将 AT 引脚拉高，进入数据透传模式

    // 初始化完成，现在可以等待手机连接并进行数据收发了
}