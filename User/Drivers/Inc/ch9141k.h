#ifndef CH9141K_H
#define CH9141K_H

/**
 * @file     ch9141k.h
 * @brief    
 * @author   Mr.Five
 * @date     2025-06-16 19:49:24
 */
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "cmsis_os2.h"
#define CH9141K_AT_ENTRY() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET) 
#define CH9141K_AT_EXIT() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET) 
#define CH9141K_SLEEP() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)  
#define CH9141K_WAKEUP() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)   
void ch9141k_init(void);
#endif /* CH9141K_H */