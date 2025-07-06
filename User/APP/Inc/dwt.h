#ifndef DWT_H
#define DWT_H

/**
 * @file     dwt.h
 * @brief
 * @author   Mr.Five
 * @date     2025-06-20 10:14:25
 */
#include "stm32h503xx.h"
#include "core_cm33.h"
/**
 * @brief  获取当前周期计数值
 * @retval 当前的 CPU 周期数
 */
#define DWT_GET_CYCLE_COUNT() (DWT->CYCCNT)
#define DWT_START(start_time) start_time = DWT_GET_CYCLE_COUNT()
#define DWT_END(end_time) end_time = DWT_GET_CYCLE_COUNT()
#define DWT_ELAPSED_CYCLES(measure_time,start_time, end_time) measure_time = ((end_time) > (start_time) ? ((end_time) - (start_time)) : (0xFFFFFFFF - (start_time) + (end_time)))
static inline void dwt_init(void)
{
    // 使能 DWT 和 ITM/ETM 跟踪功能
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 如果之前没有使能过，则先解锁 DWT
    // 对于 H5 系列，通常不需要解锁，但加上也无妨
    // DWT->LAR = 0xC5ACCE55;

    // 清零周期计数器
    DWT->CYCCNT = 0;

    // 使能周期计数器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
/**
 * @brief  将 CPU 周期数转换为微秒 (us)
 * @param  cycles CPU 周期数
 * @retval 对应的微秒数 (浮点型)
 */
static inline float dwt_cycles_to_us(uint32_t cycles)
{
    // SystemCoreClock 是 HAL 库提供的全局变量，表示核心时钟频率 (Hz)
    return (float)cycles / (SystemCoreClock / 1000000.0f);
}

/**
 * @brief  将 CPU 周期数转换为毫秒 (ms)
 * @param  cycles CPU 周期数
 * @retval 对应的毫秒数 (浮点型)
 */
static inline float dwt_cycles_to_ms(uint32_t cycles)
{
    return (float)cycles / (SystemCoreClock / 1000.0f);
}
#endif /* DWT_H */