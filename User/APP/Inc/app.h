#ifndef APP_H
#define APP_H

/**
 * @file     App.h
 * @brief    
 * @author   Mr.Five
 * @date     2025-06-08 21:15:59
 */
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "tusb.h"

#include "state_machine.h"
#include "attitude.h"
#include "com.h"
#include "bmi088.h"
#include "spl06.h"
#include "light.h"
#include "error.h"
#include "dwt.h"
#include "ch9141k.h"
/**
 * global variables
 */
extern uint32_t executed;
/**
 * function declaration
 */
void app_start(void);
void COM_Uart_Idle_Callback(void); 
#endif /* APP_H */