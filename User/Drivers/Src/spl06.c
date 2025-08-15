#include "spl06.h"
static void spl06_read_id(I2C_HandleTypeDef *hi2cx);
static void spl06_write_data(I2C_HandleTypeDef *hi2cx, uint8_t reg, uint8_t data);
static void spl06_read_data(I2C_HandleTypeDef *hi2cx, uint8_t reg, uint8_t *buf, uint16_t size);
static void spl06_read_pressure_raw(I2C_HandleTypeDef *hi2cx);
static void spl06_read_temperature_raw(I2C_HandleTypeDef *hi2cx);
static void spl06_read_calibbration(I2C_HandleTypeDef *hi2cx);
static void spl06_calculate_pressure();
static void spl06_calculate_temperature();
static void spl06_moving_average_filter();
static void spl06_pressure_to_altitude();
static spl06_data_struct spl06_data;
static spl06_calib_struct spl06_calib;
spl06_data_struct *spl06_get_data(void){
   return &spl06_data;
}
void spl06_init(I2C_HandleTypeDef *hi2cx)
{
   // sem_i2c_dma = osSemaphoreNew(1, 0, NULL);
   /**
    * 10m==120pa,1m==12pa,10cm==1.2pa
    * for oversampling x64, precision = +- 0.5Pa = +- 10/1.2 * 0.5 = +- 4.17cm
    */
   osDelay(100);
   spl06_data.kP = 7864320; // x8
   spl06_data.kT = 524288;  // x1

   spl06_write_data(hi2cx, SPL06_PRS_CFG, 0x63);  // ODR 64 OSR 8
   spl06_write_data(hi2cx, SPL06_TMP_CFG, 0x80);  // ODR 1 OSR 0
   spl06_write_data(hi2cx, SPL06_MEAS_CFG, 0x07); // contrinuous mode, pressure and temperature
   spl06_write_data(hi2cx, SPL06_CFG_REG, 0x20);  // active low interrupt, FIFO disable, complete pressure to interrupt
   osDelay(50);
   spl06_read_id(hi2cx);
   spl06_read_calibbration(hi2cx);
   error_check(ERROR_SPL06, spl06_data.error);
}
static void spl06_read_id(I2C_HandleTypeDef *hi2cx)
{
   if (HAL_I2C_Mem_Read(hi2cx, SPL06_ADDR, SPL06_REG_ID, I2C_MEMADD_SIZE_8BIT, &spl06_data.id, 1, 100) != HAL_OK)
   {
      spl06_data.error = ERROR_SPL06_I2C; // I2C communication error
   }
   if (spl06_data.id < 0x10) // 0x10 is the correct ID for SPL06
   {
      spl06_data.error = ERROR_SPL06_ID; // ID error
   }
}
static void spl06_write_data(I2C_HandleTypeDef *hi2cx, uint8_t reg, uint8_t data)
{
   if (HAL_I2C_Mem_Write(hi2cx, SPL06_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
   {
      spl06_data.error = ERROR_SPL06_I2C; // I2C communication error
   }

  
}
static void spl06_read_data(I2C_HandleTypeDef *hi2cx, uint8_t reg, uint8_t *buf, uint16_t size)
{
   if (HAL_I2C_Mem_Read(hi2cx, SPL06_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, size, 100) != HAL_OK)
   {
      spl06_data.error = ERROR_SPL06_I2C; // I2C communication error
   }
}

static void spl06_read_pressure_raw(I2C_HandleTypeDef *hi2cx)
{
   uint8_t buf[3];
   spl06_read_data(hi2cx, 0x00, buf, 3);
   int32_t raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
   if (raw & 0x800000)
      raw |= 0xFF000000;
   spl06_data.press_raw = raw;
}

static void spl06_read_temperature_raw(I2C_HandleTypeDef *hi2cx)
{
   uint8_t buf[3];
   spl06_read_data(hi2cx, 0x03, buf, 3);
   int32_t raw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
   if (raw & 0x800000)
      raw |= 0xFF000000;
   spl06_data.temp_raw = raw;
}

static void spl06_read_calibbration(I2C_HandleTypeDef *hi2cx)
{
   uint8_t buf[18] = {0};
   spl06_read_data(hi2cx, 0x10, buf, 18);

   spl06_calib.c0 = (int16_t)(((buf[0] << 4) | (buf[1] >> 4)) & 0x0FFF);
   if (spl06_calib.c0 & 0x0800)
      spl06_calib.c0 |= 0xF000;
   spl06_calib.c1 = (int16_t)(((buf[1] & 0x0F) << 8) | buf[2]);
   if (spl06_calib.c1 & 0x0800)
      spl06_calib.c1 |= 0xF000;

   spl06_calib.c00 = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
   if (spl06_calib.c00 & 0x80000)
      spl06_calib.c00 |= 0xFFF00000;

   spl06_calib.c10 = ((int32_t)(buf[5] & 0x0F) << 16) | ((int32_t)buf[6] << 8) | buf[7];
   if (spl06_calib.c10 & 0x80000)
      spl06_calib.c10 |= 0xFFF00000;

   spl06_calib.c01 = (int16_t)((buf[8] << 8) | buf[9]);
   spl06_calib.c11 = (int16_t)((buf[10] << 8) | buf[11]);
   spl06_calib.c20 = (int16_t)((buf[12] << 8) | buf[13]);
   spl06_calib.c21 = (int16_t)((buf[14] << 8) | buf[15]);
   spl06_calib.c30 = (int16_t)((buf[16] << 8) | buf[17]);
}

void spl06_read_temp_press(I2C_HandleTypeDef *hi2cx)
{
   spl06_read_pressure_raw(hi2cx);
   spl06_read_temperature_raw(hi2cx);
   spl06_moving_average_filter();
   spl06_calculate_temperature();
   spl06_calculate_pressure();
   spl06_pressure_to_altitude();
}

static void spl06_calculate_temperature()
{
   float Traw_sc = (float)spl06_data.temp_raw / (float)spl06_data.kT;
   spl06_data.temperature = 0.5f * spl06_calib.c0 + spl06_calib.c1 * Traw_sc;
}

static void spl06_calculate_pressure()
{
   float Traw_sc = (float)spl06_data.temp_raw / (float)spl06_data.kT;
   float Praw_sc = (float)spl06_data.press_raw / (float)spl06_data.kP;
   float Pcomp = spl06_calib.c00 + Praw_sc * (spl06_calib.c10 + Praw_sc * (spl06_calib.c20 + Praw_sc * spl06_calib.c30)) + Traw_sc * spl06_calib.c01 + Traw_sc * Praw_sc * (spl06_calib.c11 + Praw_sc * spl06_calib.c21);
   spl06_data.pressure = Pcomp;
}
static void spl06_moving_average_filter()
{
   static uint8_t filter_count = 0;
   static uint8_t index = 0;
   if (filter_count < SPL06_FILTER_WINDOW)
   {
      spl06_data.press_raw_sum += spl06_data.press_raw;
      spl06_data.temp_raw_sum += spl06_data.temp_raw;
      spl06_data.press_raw_filter[filter_count] = spl06_data.press_raw;
      spl06_data.temp_raw_filter[filter_count] = spl06_data.temp_raw;
      filter_count++;
   }
   else
   {
      // minus oldest value
      spl06_data.press_raw_sum -= spl06_data.press_raw_filter[index];
      spl06_data.temp_raw_sum -= spl06_data.temp_raw_filter[index];
      // add newest value
      spl06_data.press_raw_sum += spl06_data.press_raw;
      spl06_data.temp_raw_sum += spl06_data.temp_raw;
      // save newest value
      spl06_data.press_raw_filter[index] = spl06_data.press_raw;
      spl06_data.temp_raw_filter[index] = spl06_data.temp_raw;
      index = (index + 1) % SPL06_FILTER_WINDOW;
   }
   spl06_data.press_raw = spl06_data.press_raw_sum / filter_count;
   spl06_data.temp_raw = spl06_data.temp_raw_sum / filter_count;
}
static void spl06_pressure_to_altitude()
{
   // 计算 (P / P0)
   float pressure_ratio = spl06_data.pressure / SPL06_SEA_LEVEL_PRESSURE_PA;

   // 计算 (P / P0)^(1 / 5.255)
   // 1 / 5.255 ≈ 0.190294957
   float power_of_ratio = pow(pressure_ratio, 0.190294957f);

   // 计算最终高度 unit:m
   spl06_data.altitude = 44330.0f * (1.0f - power_of_ratio);
}