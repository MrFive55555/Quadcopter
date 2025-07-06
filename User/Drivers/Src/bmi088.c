#include "bmi088.h"

/* 信号量句柄 */
// static osSemaphoreId_t sem_spi_dma;
static void bmi088_read_id(SPI_HandleTypeDef *hspix, bmi088_sensor_enum sensor, bmi088_data_struct *bmi088_data);
static void bmi088_write_data(SPI_HandleTypeDef *hspix, bmi088_sensor_enum sensor, uint8_t reg, uint8_t data, bmi088_data_struct *bmi088_data);
static void bmi088_read_data(SPI_HandleTypeDef *hspix, bmi088_sensor_enum sensor, uint8_t reg, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t size, bmi088_data_struct *bmi088_data);
static void bmi088_zero_bias(bmi088_data_struct *bmi088_data);
void bmi088_init(SPI_HandleTypeDef *hspix, bmi088_data_struct *bmi088_data)
{
    // sem_spi_dma = osSemaphoreNew(1, 0, NULL);
    /**
     * config accel
     */
    // switch to SPI mode
    uint8_t dummy;
    osDelay(100);
    BMI088_ACCEL_CS_ENABLE();
    HAL_SPI_Receive(hspix, &dummy, 1, 100);
    osDelay(1);
    BMI088_ACCEL_CS_DISABLE();
    osDelay(1);
    // start SPI communication formally
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_ACC_SOFTRESET, 0xB6, bmi088_data); // rest
    osDelay(1);
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_ACC_PWR_CONF, 0x00, bmi088_data); // activate mode
    osDelay(1);
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_ACC_PWR_CTRL, BMI088_ACC_PWR_CTRL_ON, bmi088_data); // enable accelerometer
    osDelay(50);
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, bmi088_data);
    osDelay(1);
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_ACC_CONF, 0x8C, bmi088_data); // 1600Hz ODR,OSR4
    osDelay(1);
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_INT1_IO_CONF, 0x0A, bmi088_data); // enable INT1, active high, push-pull
    osDelay(1);
    bmi088_write_data(hspix, BMI088_ACCEL, BMI088_INT1_INT2_MAP_DATA, 0x04, bmi088_data); // map INT1 to data ready
    osDelay(1);
    bmi088_read_id(hspix, BMI088_ACCEL, bmi088_data);
    osDelay(1); // 等待传感器稳定
    error_check(ERROR_BMI088, bmi088_data->error);
    /**
     * config gyro
     */
    osDelay(30);
    bmi088_write_data(hspix, BMI088_GYRO, BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_2000DPS, bmi088_data);
    osDelay(1);
    bmi088_write_data(hspix, BMI088_GYRO, BMI088_GYRO_BANDWIDTH, 0x01, bmi088_data); // ODR 2000hz,OSR 230hz
    osDelay(1);
    // bmi088_write_data(hspix, BMI088_GYRO, BMI088_GYRO_INT_CTRL, 0x08, bmi088_data); //  enable INT3
    // osDelay(1);
    // bmi088_write_data(hspix, BMI088_GYRO, BMI088_GYRO_INT3_INT4_IO_CONF, 0x01, bmi088_data); //enable INT3, active high, push-pull
    // osDelay(1);
    // bmi088_write_data(hspix, BMI088_GYRO, BMI088_GYRO_INT3_INT4_IO_MAP, 0x01, bmi088_data); // map INT3 to data ready
    bmi088_read_id(hspix, BMI088_GYRO, bmi088_data);
    osDelay(1);
    error_check(ERROR_BMI088, bmi088_data->error);
    /**
     * read id can't exectue first , otherwise the id will be wrong
     * The reason I don't know.
     */
}
static void bmi088_read_id(SPI_HandleTypeDef *hspix, bmi088_sensor_enum sensor, bmi088_data_struct *bmi088_data)
{
    uint8_t tx_data[3] = {BMI088_CHIP_ID | 0x80, 0x00, 0x00}; // 命令: 读0x00寄存器
    uint8_t rx_data[3] = {0};

    if (sensor == BMI088_ACCEL)
    {
        BMI088_ACCEL_CS_ENABLE();
        if (HAL_SPI_TransmitReceive(hspix, tx_data, rx_data, 3, 100) != HAL_OK)
        {
            bmi088_data->error = ERROR_BMI088_SPI;
            BMI088_ACCEL_CS_DISABLE();
            return;
        }
        BMI088_ACCEL_CS_DISABLE();

        // 第二次读取的才是有效ID
        bmi088_data->accl_id = rx_data[2];
        if (bmi088_data->accl_id != 0x1E) // 0x1E是加速度计的正确ID
            bmi088_data->error = ERROR_BMI088_ACC_ID;
    }
    else // BMI088_GYRO
    {
        // 陀螺仪不需要虚拟读取，直接读即可
        BMI088_GYRO_CS_ENABLE();
        if (HAL_SPI_TransmitReceive(hspix, tx_data, rx_data, 2, 100) != HAL_OK)
        {
            bmi088_data->error = ERROR_BMI088_SPI;
            BMI088_GYRO_CS_DISABLE();
            return;
        }
        BMI088_GYRO_CS_DISABLE();

        bmi088_data->gyro_id = rx_data[1];
        if (bmi088_data->gyro_id != 0x0F) // 0x0F是陀螺仪的正确ID
            bmi088_data->error = ERROR_BMI088_GYRO_ID;
    }
}
static void bmi088_write_data(SPI_HandleTypeDef *hspix, bmi088_sensor_enum sensor, uint8_t reg, uint8_t data, bmi088_data_struct *bmi088_data)
{
    uint8_t tx_buf[2] = {reg & 0x7F, data};
    if (sensor == BMI088_ACCEL)
        BMI088_ACCEL_CS_ENABLE();
    else if (sensor == BMI088_GYRO)
        BMI088_GYRO_CS_ENABLE();

    if (HAL_SPI_Transmit(hspix, tx_buf, 2, 100) != HAL_OK)
    {
        bmi088_data->error = ERROR_BMI088_SPI;
    }
    // osSemaphoreAcquire(sem_spi_dma, osWaitForever);

    if (sensor == BMI088_ACCEL)
        BMI088_ACCEL_CS_DISABLE();
    else if (sensor == BMI088_GYRO)
        BMI088_GYRO_CS_DISABLE();
}
// 这是一个更标准的突发读取实现
static void bmi088_read_data(SPI_HandleTypeDef *hspix, bmi088_sensor_enum sensor, uint8_t reg, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t size, bmi088_data_struct *bmi088_data)
{

    tx_buf[0] = reg | 0x80; // 设置最高位为1表示读取
    // 对于加速度计，burst read需要处理一个额外的dummy字节
    if (sensor == BMI088_ACCEL)
    {
        BMI088_ACCEL_CS_ENABLE();
        if (HAL_SPI_TransmitReceive(hspix, tx_buf, rx_buf, size, 100) != HAL_OK)
        {
            bmi088_data->error = ERROR_BMI088_SPI;
        }
        BMI088_ACCEL_CS_DISABLE();
    }
    else // 陀螺仪没有dummy字节
    {
        BMI088_GYRO_CS_ENABLE();
        if (HAL_SPI_TransmitReceive(hspix, tx_buf, rx_buf, size, 100) != HAL_OK)
        {
            bmi088_data->error = ERROR_BMI088_SPI;
        }
        BMI088_GYRO_CS_DISABLE();
    }
}
void bmi088_read_acc_gyro(SPI_HandleTypeDef *hspix, bmi088_data_struct *bmi088_data)
{
    /**
     * [0] is used to send register address,
     * [1] is used to dummy data for Accelerater,because accelerater needs to receive a dummy byte
     */
    uint8_t tx_buf[8] = {0};
    uint8_t rx_buf[8] = {0};
    /**
     * read accelerater data
     * 0x80 is read command, 0x7F is write command
     */
    bmi088_read_data(hspix, BMI088_ACCEL, BMI088_ACC_X_LSB, tx_buf, rx_buf, 8, bmi088_data);
    // osSemaphoreAcquire(sem_spi_dma, osWaitForever);
    bmi088_data->accl_sample[0] = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    bmi088_data->accl_sample[1] = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    bmi088_data->accl_sample[2] = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
    /**
     *  read gyroscope data
     *  0x80 is read command, 0x7F is write command
     */
    bmi088_read_data(hspix, BMI088_GYRO, BMI088_GYRO_X_LSB, tx_buf, rx_buf, 7, bmi088_data);
    // osSemaphoreAcquire(sem_spi_dma, osWaitForever);
    bmi088_data->gyro_sample[0] = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    bmi088_data->gyro_sample[1] = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    bmi088_data->gyro_sample[2] = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);
    if (!bmi088_data->bias_ok_flag)
        bmi088_zero_bias(bmi088_data);
    else
        for (uint8_t i = 0; i < 3; i++)
        {
            bmi088_data->accl_sample[i] -= bmi088_data->accl_sample_bias[i];
            bmi088_data->gyro_sample[i] -= bmi088_data->gyro_sample_bias[i];
        }
}
static void bmi088_zero_bias(bmi088_data_struct *bmi088_data)
{
    float ax = bmi088_data->accl_sample[X];
    float ay = bmi088_data->accl_sample[Y];
    float az = bmi088_data->accl_sample[Z];
    float g_norm = sqrt(ax * ax + ay * ay + az * az);

    if (g_norm >= BMI088_STATIC_G_NORM_SQ_MIN && g_norm <= BMI088_STATIC_G_NORM_SQ_MAX)
    {
        if (bmi088_data->cal_count++ < BMI088_CAL_COUNT_MAX)
        {
            for (int i = 0; i < 3; i++)
            {
                bmi088_data->accl_sample_sum[i] += bmi088_data->accl_sample[i];
                bmi088_data->gyro_sample_sum[i] += bmi088_data->gyro_sample[i];
            }
        }
        else
        {
            // X和Y轴的偏置计算是正确的
            bmi088_data->accl_sample_bias[X] = bmi088_data->accl_sample_sum[X] / BMI088_CAL_COUNT_MAX;
            bmi088_data->accl_sample_bias[Y] = bmi088_data->accl_sample_sum[Y] / BMI088_CAL_COUNT_MAX;

            // 【关键修改】修正Z轴的偏置计算
            // 1. 先计算出1g在LSB单位下对应的值
            float one_g_in_lsb = 10920.0f; // ATTITUDE_ACC_SENS 是 g/LSB，所以倒数是 LSB/g

            // 2. Z轴的偏置 = Z轴的平均读数 - 1g对应的读数
            bmi088_data->accl_sample_bias[Z] = (bmi088_data->accl_sample_sum[Z] / BMI088_CAL_COUNT_MAX) - one_g_in_lsb;
            for (int i = 0; i < 3; i++)
            {
                //bmi088_data->accl_sample_bias[i] = bmi088_data->accl_sample_sum[i] / BMI088_CAL_COUNT_MAX;
                bmi088_data->gyro_sample_bias[i] = bmi088_data->gyro_sample_sum[i] / BMI088_CAL_COUNT_MAX;
            }
            bmi088_data->bias_ok_flag = 1;
        }
    }
    else
    {
        bmi088_data->cal_count = 0;
        for (uint8_t i = 0; i < 3; i++)
        {
            bmi088_data->accl_sample_sum[i] = 0;
            bmi088_data->gyro_sample_sum[i] = 0;
        }
    }
}
void bmi088_reset(bmi088_data_struct *bmi088_data)
{
    memset(bmi088_data, 0, sizeof(bmi088_data_struct));
}