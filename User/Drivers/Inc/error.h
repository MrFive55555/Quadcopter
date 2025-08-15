#ifndef ERROR_H
#define ERROR_H

/**
 * @file     error.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-17 11:07:02
 */
typedef enum
{
    ERROR_BMI088 = 0,
    ERROR_SPL06 = 1,
} error_device_enum;
typedef enum
{
    ERROR_NONE = 0,  // 无错误
    ERROR_SPL06_ID,  // ID错误
    ERROR_SPL06_I2C, // I2C通信错误
    ERROR_BMI088_ACC_ID,
    ERROR_BMI088_GYRO_ID,   
    ERROR_BMI088_SPI,
} error_type_enum;
typedef struct
{
    error_device_enum device; // 设备类型
    error_type_enum type;     // 错误类型
} error_queue_struct;
typedef void (*error_callback_t)(error_queue_struct);
void error_register_callback(error_callback_t callback);
void error_check(error_device_enum device, error_type_enum error);
#endif /* ERROR_H */