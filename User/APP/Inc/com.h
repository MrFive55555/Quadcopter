#ifndef COM_H
#define COM_H

/**
 * @file     com.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-10 19:02:19
 */
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "attitude.h"
#include "state_machine.h"
#include "flight_control.h"
#include "pid.h"

#define BUFFER_SIZE 256
#define CHAR_BLANK 32
#define CHAR_ENTER 10
#define CHAR_CR 13
typedef void (*command_handler_func)(uint8_t *buf);
typedef void (*com_callback_t)(void *);
typedef struct
{
  const char *command_str;    // 命令字符串
  const command_handler_func handler; // 命令处理函数
} command_lookup_struct;
typedef struct
{
  uint8_t head_frame_size;
  uint8_t head_frame[3];
  uint8_t is_head_frame; 
} com_data_struct;
typedef enum
{
  COM_UART_SEND = 0,
  COM_UART_READ,
  COM_USB_SEND,
  COM_USB_READ,
} com_source_enum;
typedef struct
{
  com_source_enum source;
  uint32_t start_pos;
  uint32_t length;
} com_queue_struct;
/**
 * function declaration
 */
com_data_struct *com_get_data(void);
void com_register_callback(com_callback_t callback);
void com_parse_buf(uint8_t *buf, uint32_t length);
#endif /* COM_H */