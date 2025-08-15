/**
 * @file     error.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-17 11:07:16
 */

#include "error.h"
static error_callback_t error_callback;
void error_register_callback(error_callback_t callback)
{
    error_callback = callback;
}
void error_check(error_device_enum device, error_type_enum error)
{
    //if(error==ERROR_BMI088_GYRO_ID) executed = error;
    if (error != ERROR_NONE)
    {
        error_queue_struct error_queue = {0};
        error_queue.device = device;
        error_queue.type = error;
        error_callback(error_queue);
    }
}