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
#define BUFFER_SIZE 256
#define CHAR_BLANK 32
#define CHAR_ENTER 10
#define CHAR_CR 13
typedef enum
{
  COM_NONE = 0,   // no command
  COM_TAKE_OFF,   // take off command
  COM_PID_SET,    // set pid
  COM_PID_GET,    // get pid
  COM_ADD_HEIGHT, // add height
  COM_MINUS_HEIGHT, // sub height
  COM_ADD_THROTTLE, // add throttle
  COM_MINUS_THROTTLE, // sub throttle
  COM_ENABLE_LOG, // enable log
  COM_DISABLE_LOG, // disable log
  COM_KEEP_ALIVE, // keep alive command
} com_type_enum;
typedef struct
{
  uint8_t head_frame_size;
  uint8_t head_frame[4]; // head frame is 3 bit,but use more bit to judge
  uint8_t is_head_frame; // command content
  uint8_t command_type;
  uint8_t take_off; // 0: not take off, 1: take off
  // uint8_t height;
  float pid[3]; // pid value
  uint32_t start_time;
  uint32_t end_time;
  uint32_t measure_time;
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
// void com_init(uint8_t *buf,uint16_t length);
void com_parse_buf(com_data_struct *com_data, uint8_t *buf, uint32_t length);
void com_usb_parse_buf(uint8_t *buf, uint32_t length);
#endif /* COM_H */