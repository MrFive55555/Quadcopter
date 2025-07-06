/**
 * @file     error.c
 * @brief
 * @author   Mr.Five
 * @date     2025-06-17 11:07:16
 */

#include "error.h"
static osMessageQueueId_t queue_error_id = NULL;
void error_init(osMessageQueueId_t queue)
{
    queue_error_id = queue;
}
void error_check(error_device_enum device, error_type_enum error)
{
    //if(error==ERROR_BMI088_GYRO_ID) executed = error;
    if (error != ERROR_NONE)
    {
        error_queue_struct error_queue = {0};
        error_queue.device = device;
        error_queue.type = error;
        if (queue_error_id != NULL)
        {
            osMessageQueuePut(queue_error_id, &error_queue, 0, 0);
        }
    }
}