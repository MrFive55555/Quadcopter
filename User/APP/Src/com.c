/**
 * @file     com.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-10 19:06:58
 */

#include "com.h"
static void com_parse_command(com_data_struct *com_data, uint8_t *buf);
static void com_str_to_pid(com_data_struct *com_data, char *buf);
/**
 * parse buf to get command
 * command format: $CMxxxxxx\r\n
 * where $CM is the head of command, xxxxxx is the command content
 */
void com_parse_buf(com_data_struct *com_data, uint8_t *buf, uint32_t length)
{
    uint16_t index = 0;
    uint16_t command_head_index;
    while (index != length)
    {
        if (com_data->is_head_frame)
        {
            if (buf[index - 1] == CHAR_CR && buf[index] == CHAR_ENTER)
            {
                com_parse_command(com_data, &buf[com_data->head_frame_size]);
                com_data->is_head_frame = 0;
            }
        }
        else
        {
            if (index < com_data->head_frame_size)
            {
                com_data->head_frame[index] = buf[index];
            }
            else
            {
                if (com_data->head_frame[index - 3] == '$' && com_data->head_frame[index - 2] == 'C' && com_data->head_frame[index - 1] == 'M')
                {
                    com_data->is_head_frame = 1; // head of frame is found
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
static void com_parse_command(com_data_struct *com_data, uint8_t *buf)
{
    uint16_t index = 0;
    /**
     * index will be setted to 0 when judge head frame
     * so we need reset index to judge next head frame
     */
    if (buf[index++] == 'T' && buf[index++] == 'A' && buf[index++] == 'K')
    {
        com_data->command_type = COM_TAKE_OFF; // Set command type to take off
        if (buf[index] == '1')
        {
            com_data->take_off = 1;
        }
        else
        {
            com_data->take_off = 0;
        }
    }
    index = 0;
    if (buf[index++] == 'P' && buf[index++] == 'I' && buf[index++] == 'D') // PIDxx.xx,xx.xx,xx.xx
    {
        com_data->command_type = COM_PID_SET;
        com_str_to_pid(com_data, (char *)&buf[index]);
    }
    index = 0;
    if (buf[index++] == 'A' && buf[index++] == 'D' && buf[index++] == 'H')
    {
        com_data->command_type = COM_ADD_HEIGHT;
    }
    index = 0;
    if (buf[index++] == 'M' && buf[index++] == 'N' && buf[index++] == 'H')
    {
        com_data->command_type = COM_MINUS_HEIGHT;
    }
    index = 0;
    if (buf[index++] == 'P' && buf[index++] == 'U' && buf[index++] == 'L')
    {
        com_data->command_type = COM_ADD_THROTTLE;
    }
    index = 0;
    if (buf[index++] == 'D' && buf[index++] == 'O' && buf[index++] == 'W')
    {
        com_data->command_type = COM_MINUS_THROTTLE;
    }
    index = 0;
    if (buf[index++] == 'L' && buf[index++] == 'O' && buf[index++] == 'G')
    {
        com_data->command_type = COM_ENABLE_LOG;
    }
    index = 0;
    if (buf[index++] == 'D' && buf[index++] == 'O' && buf[index++] == 'G')
    {
        com_data->command_type = COM_DISABLE_LOG;
    }
    index = 0;
    if (buf[index++] == 'K' && buf[index++] == 'E' && buf[index++] == 'P')
    {
        com_data->command_type = COM_KEEP_ALIVE;
    }
}
void com_usb_parse_buf(uint8_t *buf, uint32_t length)
{
}

// char buffer[] = "PID52.75,8.32,0.51";

// 这个函数不会修改传入的 data
static void com_str_to_pid(com_data_struct *com_data, char *buf)
{
    char *endptr; // 用于接收 strtod 解析结束的位置

    com_data->pid[0] = strtof(buf, &endptr); // strtof 是 float 版本
    if (buf == endptr || *endptr != ',')
    {

        return;
    }

    const char *i_start = endptr + 1;
    com_data->pid[1] = strtof(i_start, &endptr);
    if (i_start == endptr || *endptr != ',')
    {

        return;
    }

    const char *d_start = endptr + 1;
    com_data->pid[2] = strtof(d_start, &endptr);
}