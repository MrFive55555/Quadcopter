#ifndef BMI088_H
#define BMI088_H

/**
 * @file     bmi088.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-08 21:55:04
 */
#include <string.h>
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "error.h"
/**
 * BMI088 register addresses
 */
#define BMI088_CHIP_ID 0x00
// ----------------- 加速度计寄存器 -----------------
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_X_LSB 0x12
#define BMI088_ACC_X_MSB 0x13
#define BMI088_ACC_Y_LSB 0x14
#define BMI088_ACC_Y_MSB 0x15
#define BMI088_ACC_Z_LSB 0x16
#define BMI088_ACC_Z_MSB 0x17
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_RANGE 0x41
#define BMI088_INT1_IO_CONF 0x53
#define BMI088_INT2_IO_CONF 0x54
#define BMI088_INT1_INT2_MAP_DATA 0x58
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_SOFTRESET 0x7E

// ACC_RANGE（量程配置值）
#define BMI088_ACC_RANGE_3G 0x00
#define BMI088_ACC_RANGE_6G 0x01
#define BMI088_ACC_RANGE_12G 0x02
#define BMI088_ACC_RANGE_24G 0x03

// ACC_PWR_CTRL 控制
#define BMI088_ACC_PWR_CTRL_OFF 0x00
#define BMI088_ACC_PWR_CTRL_ON 0x04

// ACC_SOFTRESET
#define BMI088_SOFTRESET_CMD 0xB6

// ----------------- 陀螺仪寄存器 -----------------
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_X_LSB 0x02
#define BMI088_GYRO_X_MSB 0x03
#define BMI088_GYRO_Y_LSB 0x04
#define BMI088_GYRO_Y_MSB 0x05
#define BMI088_GYRO_Z_LSB 0x06
#define BMI088_GYRO_Z_MSB 0x07
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_INT_CTRL 0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18

// GYRO_RANGE（量程配置值）
#define BMI088_GYRO_RANGE_2000DPS 0x00
#define BMI088_GYRO_RANGE_1000DPS 0x01
#define BMI088_GYRO_RANGE_500DPS 0x02
#define BMI088_GYRO_RANGE_250DPS 0x03
#define BMI088_GYRO_RANGE_125DPS 0x04
/**
 * BMI088 sensor configuration
 */
#define BMI088_CAL_COUNT_MAX 4000                                // 偏置校准最大采样次数
#define BMI088_ACC_NORM_1G (32768.0f / 3.0f)                    // ±3g 对应的加速度计数据 1g 的值
#define BMI088_STATIC_G_NORM_SQ_MIN (BMI088_ACC_NORM_1G * 0.9f) // 0.9g
#define BMI088_STATIC_G_NORM_SQ_MAX (BMI088_ACC_NORM_1G * 1.1f) // 1.1g
typedef enum
{
    BMI088_ACCEL = 0, // 加速度计
    BMI088_GYRO = 1   // 陀螺仪
} bmi088_sensor_enum;
typedef enum
{
    X = 0,
    Y,
    Z,
} bmi088_axis_enum;
typedef struct
{
    // 4字节类型
    float accl[3]; // 加速度计数据，单位：g
    float gyro[3]; // 陀螺仪数据，单位：°/s
    int32_t accl_sample_sum[3];
    int32_t gyro_sample_sum[3];

    // 2字节类型
    int16_t accl_sample[3];
    int16_t gyro_sample[3];
    int16_t accl_sample_bias[3]; // zero bias of acceleration data
    int16_t gyro_sample_bias[3]; // zero bias of gyroscope data
    uint16_t cal_count;

    // 1字节类型
    error_type_enum error;
    uint8_t accl_id;
    uint8_t gyro_id;
    uint8_t bias_ok_flag; // 0: no bias, 1: bias ok
    
} bmi088_data_struct;
#define BMI088_ACCEL_INT_PIN GPIO_PIN_1                                               // 加速度计 INT1 引脚
#define BMI088_ACCEL_CS_ENABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET) // CS Enable
#define BMI088_ACCEL_CS_DISABLE() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)  // CS Disable
#define BMI088_GYRO_CS_ENABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)  // CS Enable
#define BMI088_GYRO_CS_DISABLE() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)   // CS Disable
/**
 * function declaration
 */
void bmi088_reset(bmi088_data_struct *bmi088_data);
void bmi088_init(SPI_HandleTypeDef *hspix, bmi088_data_struct *bmi088_data);
void bmi088_read_acc_gyro(SPI_HandleTypeDef *hspix, bmi088_data_struct *bmi088_data);
#endif /* BMI088_H */