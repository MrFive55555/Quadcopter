#ifndef LIGHT_H
#define LIGHT_H

/**
 * @file     light.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-13 10:14:23
 */
#include "main.h"
#define LIGHT_LED_ONE_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LIGHT_LED_ONE_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define LIGHT_LED_ONE_BLINK() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13)
#define LIGHT_LED_TWO_ON() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define LIGHT_LED_TWO_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define LIGHT_LED_TWO_BLINK() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12)
typedef enum
{
    LIGHT_OFF = 0,
    LIGHT_ON = 1,
    LIGHT_BLINK = 2,
} light_state_enum;
typedef enum
{
   LIGHT_TAKE_OFF = 0,
   LIGHT_ERROR = 1,
} light_type_enum;
void light_off_all(void);
void light_control(light_type_enum type, light_state_enum state);
#endif /* LIGHT_H */