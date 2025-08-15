/**
 * @file     com.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-10 19:06:58
 */

#include "com.h"
static void com_parse_command(uint8_t *buf);
static void com_parse_take_off(uint8_t *buf);
static void com_parse_throttle(uint8_t *buf);
static void com_parse_height(uint8_t *buf);
static void com_parse_log(uint8_t *buf);
static void com_parse_keep_alive(uint8_t *buf);
static void com_parse_attitude(uint8_t *buf);
static com_data_struct com_data = {
    .head_frame_size = 3,
}; 
static const command_lookup_struct command_table[] = {
    {"TAK", com_parse_take_off},
    {"THR", com_parse_throttle},
    {"HET", com_parse_height},
    {"LOG", com_parse_log},
    {"KEP", com_parse_keep_alive},
    {"CTL", com_parse_attitude},
};
#define COMMAND_TABLE_SIZE (sizeof(command_table) / sizeof(command_lookup_struct))
#define COM_CALLBACK_SIZE 3
static com_callback_t com_callback[COM_CALLBACK_SIZE];
com_data_struct *com_get_data(void){
    return &com_data;
}
void com_register_callback(com_callback_t callback)
{
    static uint8_t register_count = 0;
    if (register_count < COM_CALLBACK_SIZE)
    {
        com_callback[register_count++] = callback;
    }
}
/**
 * parse buf to get command
 * command format: $CMxxxxxx\r\n
 * where $CM is the head of command, xxxxxx is the command content
 */
void com_parse_buf(uint8_t *buf, uint32_t length)
{
    uint16_t index = 0;
    uint16_t command_head_index;
    while (index != length)
    {
        if (com_data.is_head_frame) // end of data frame
        {
            if (buf[index - 1] == CHAR_CR && buf[index] == CHAR_ENTER)
            {
                com_parse_command(&buf[com_data.head_frame_size]);
                com_data.is_head_frame = 0;
            }
        }
        else
        {
            if (index < com_data.head_frame_size)
            {
                com_data.head_frame[index] = buf[index];
            }
            else
            {
                if (com_data.head_frame[index - 3] == '$' && com_data.head_frame[index - 2] == 'C' && com_data.head_frame[index - 1] == 'M')
                {
                    com_data.is_head_frame = 1; // head of data frame
                }
                else
                {
                    break;
                }
            }
        }
        index++;
    }
}
static void com_parse_command(uint8_t *buf)
{
    // 循环遍历查找表
    for (uint8_t i = 0; i < COMMAND_TABLE_SIZE; i++)
    {
        if (strncmp((const char *)buf, command_table[i].command_str, 3) == 0)
        {
            command_table[i].handler(buf + 3); // ignore command tag, only directly transfer data
            return;
        }
    }
}
/**
 * CTL command handler
 * 解析姿态控制命令
 * 格式: CTL<roll>,<pitch>,<yaw>,<throttle>
 * 例如: CTL100,200,300,1500
 */
static void com_parse_attitude(uint8_t *buf)
{
    uint8_t data_size = 0;
    int16_t target[4] = {0};
    while (*buf != '\r')
    {
        if (*buf >= '0' && *buf <= '9')
        {
            target[data_size] = target[data_size] * 10 + (*buf - '0'); // 将字符转换为数字
        }
        else if (*buf == ',')
        {
            data_size++;
        }
        buf++;
    }
    if (data_size == 3) // make sure having four datas
    {
        attitude_target_set(target);
    }
}
static void com_parse_take_off(uint8_t *buf)
{
    if (*buf != '1')
    {
        if (quadcopter_state_get() == STATE_FLYING)
        {
            // 发送降落命令
            com_callback[0]((void *)EVT_CMD_LAND);
        }
    }
    else
    {
        if (quadcopter_state_get() == STATE_IDLE)
        {
            // 发送起飞命令
            com_callback[0]((void *)EVT_CMD_TAKE_OFF);
        }
    }
}
static void com_parse_throttle(uint8_t *buf)
{
    if (quadcopter_state_get() == STATE_FLYING)
    {
        if (*buf != '1')
        {
            attitude_throttle_add(-100);
        }
        else
        {
           attitude_throttle_add(100);
        }
    }
}
static void com_parse_height(uint8_t *buf)
{
    if (quadcopter_state_get() == STATE_FLYING)
    {
        if (*buf != '1')
        {
            if (pid_get_data()[PID_HEIGHT].target < 0.1f && pid_get_data()[PID_HEIGHT].target > -0.1f)
            {
                pid_get_data()[PID_HEIGHT].target = spl06_get_data()->pressure;
            }
            (pid_get_data() + PID_HEIGHT)->target -= 5.0f;
        }
        else
        {
            if (pid_get_data()[PID_HEIGHT].target < 0.1f && pid_get_data()[PID_HEIGHT].target > -0.1f)
            {
                pid_get_data()[PID_HEIGHT].target = spl06_get_data()->pressure;
            }
            (*(pid_get_data() + PID_HEIGHT)).target -= 5.0f;
        }
    }
}
static void com_parse_log(uint8_t *buf)
{
    if (*buf != '1')
    {
        // 发送停止日志命令
        com_callback[1]((void *)0);
    }
    else
    {
        // 发送开始日志命令
        com_callback[1]((void *)1);
    }
}
static void com_parse_keep_alive(uint8_t *buf)
{
    com_callback[2]((void *)EVT_KEEP_ALIVE);
}