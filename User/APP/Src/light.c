/**
 * @file     light.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-13 10:14:32
 */

#include "light.h"
void light_init(void)
{
    // Initialize the light control pins
    light_control(LIGHT_TAKE_OFF, LIGHT_OFF);
    light_control(LIGHT_ERROR, LIGHT_OFF);
}
void light_control(light_type_enum type, light_state_enum state)
{
    if (type == LIGHT_TAKE_OFF)
    {
        if (state == LIGHT_ON)
        {
            LIGHT_LED_ONE_ON();
        }
        else if (state == LIGHT_OFF)
        {
            LIGHT_LED_ONE_OFF();
        }
        else if (state == LIGHT_BLINK)
        {
            LIGHT_LED_ONE_BLINK();
        }
    }
    else if (type == LIGHT_ERROR)
    {
        if (state == LIGHT_ON)
        {
            LIGHT_LED_TWO_ON();
        }
        else if (state == LIGHT_OFF)
        {
            LIGHT_LED_TWO_OFF();
        }
        else if (state == LIGHT_BLINK)
        {
            LIGHT_LED_TWO_BLINK();
        }
    }
}