#ifndef SPL06_H
#define SPL06_H

/**
 * @file     spl06.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-10 10:38:51
 */
#include "main.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>
#include "error.h"
/**
 * SPL06 sensor configuration
 */
#define SPL06_REG_ID 0x0D        // Device ID register
#define SPL06_PRS_CFG 0x06       // Pressure configuration register
#define SPL06_TMP_CFG 0x07       // Temperature configuration register
#define SPL06_MEAS_CFG 0x08      // Measurement configuration register
#define SPL06_CFG_REG 0x09       // Interrupt and FIFO configuration register
#define SPL06_ADDR (0x77 << 1)   // I2C address of SPL06 sensor
#define SPL06_INT_PIN GPIO_PIN_5 // Interrupt pin for SPL06 sensor
#define SPL06_FILTER_WINDOW 16   // Filter window size for SPL06 sensor
#define SPL06_SEA_LEVEL_PRESSURE_PA 101325.0f
typedef struct
{
    int32_t c00;
    int32_t c10;
    int16_t c0;
    int16_t c1;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} spl06_calib_struct;
typedef struct
{
    int32_t press_raw;
    int32_t temp_raw;
    int32_t press_raw_sum;
    int32_t temp_raw_sum;
    int32_t press_raw_filter[SPL06_FILTER_WINDOW];
    int32_t temp_raw_filter[SPL06_FILTER_WINDOW];
    uint32_t kP; // 压力校准系数
    uint32_t kT; // 温度校准系数
    error_type_enum error;
    float pressure;
    float temperature;
    float altitude; // 海拔高度
    uint8_t id;
} spl06_data_struct;

/**
 * function declaration
 */
void spl06_init(I2C_HandleTypeDef *hi2cx);
spl06_data_struct *spl06_get_data(void);
void spl06_read_temp_press(I2C_HandleTypeDef *hi2cx);
#endif /* SPL06_H */