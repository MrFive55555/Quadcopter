/**
 * @file     app.c
 * @brief    CMSIS-RTOS2 version of quadcopter app
 */

#include "app.h"
/* Global variables */
pid_data_struct pid_pitch = {0};
pid_data_struct pid_roll = {0};
pid_data_struct pid_pitch_rate = {0};
pid_data_struct pid_roll_rate = {0};
pid_data_struct pid_yaw = {0};
pid_data_struct pid_height = {0};
attitude_data_struct attitude_data = {0};
bmi088_data_struct bmi088_data = {0};
spl06_data_struct spl06_data = {0};
uint8_t dma_receive_buffer[BUFFER_SIZE];
com_data_struct com_data = {
    .head_frame_size = 3,
};
error_queue_struct error_queue = {0};
/* Semaphore */

/* Event flags */
#define EVENT_MEASURE_BMI088_READ (1 << 0)
#define EVENT_MEASURE_SPL06_READ (1 << 1)
osEventFlagsId_t event_measure_id;

/* Message queue */
#define QUEUE_COM_MSG_NUM 4
osMessageQueueId_t queue_com_id = NULL;
#define QUEUE_ERROR_MSG_NUM 1
osMessageQueueId_t queue_error_id = NULL;
/* timer */
osTimerId_t timer_send_id = NULL;
/* Thread attributes */
const osThreadAttr_t task_sm_attributes = {
    .name = "stateMachineTask",
    .priority = osPriorityRealtime5,
    .stack_size = 2048};
const osThreadAttr_t task_com_attributes = {
    .name = "commTask",
    .priority = osPriorityLow2,
    .stack_size = 2048};
const osThreadAttr_t task_usb_attributes = {
    .name = "usbTask",
    .priority = osPriorityLow3,
    .stack_size = 2048};
const osThreadAttr_t task_error_attributes = {
    .name = "errorTask",
    .priority = osPriorityRealtime6,
    .stack_size = 512};

/* Function declarations */
static void timer_send_callback(void *argument);
static void task_state_machine(void *argument);
static void task_com(void *argument);
static void task_usb(void *argument);
static void task_error(void *argument);
static void enter_error_state(void); // 进入错误状态的辅助函数
static void com_execute_command(char *tx_buf);
/* --- state machine parameters --- */
volatile quadcopter_state_enum quadcopter_state = STATE_INIT;
osMessageQueueId_t queue_sm_evt_id = NULL;
#define QUEUE_SM_EVENT_MSG_NUM 8
volatile uint32_t g_last_heartbeat_tick = 0;
/* Application start */
void app_start(void)
{
  quadcopter_state = STATE_INIT; // 设置初始状态
  /**
   *event create
   */
  event_measure_id = osEventFlagsNew(NULL);
  /**
   * queue create
   */
  queue_error_id = osMessageQueueNew(QUEUE_ERROR_MSG_NUM, sizeof(error_queue_struct), NULL);
  queue_com_id = osMessageQueueNew(QUEUE_COM_MSG_NUM, sizeof(com_queue_struct), NULL);
  queue_sm_evt_id = osMessageQueueNew(QUEUE_SM_EVENT_MSG_NUM, sizeof(state_machine_event_enum), NULL); // 队列深度为8
  error_init(queue_error_id);
  /**
   * timer create
   */
  timer_send_id = osTimerNew(timer_send_callback, osTimerPeriodic, NULL, NULL);
  /**
   * peripheral initialization
   */
  light_init();
  /**
   * occur error
   * if error occurs, the system will stop here.
   * now I don't know how to handle error in RTOS2
   * so I just don't use it for now.
   * resolution: use timer to judge idle.
   */
  HAL_UART_Receive_DMA(&huart1, dma_receive_buffer, sizeof(dma_receive_buffer));
  HAL_TIM_Base_Start_IT(&htim6);
  tusb_init();
  attitude_init(&attitude_data, 0);
  dwt_init(); // Initialize DWT for cycle counting
  pid_init(&pid_pitch, 8.0f, 0.2f, 0.0f, 200, 400);
  pid_init(&pid_roll, 8.0f, 0.2f, 0.0f, 200, 400);
  pid_init(&pid_pitch_rate, 2.0f, 0.01f, 0.002f, 200, 400);
  pid_init(&pid_roll_rate, 2.0f, 0.01f, 0.002f, 200, 400);
  pid_init(&pid_height, 2.0f, 0.01f, 0.01f, 200, 400);
  bmi088_init(&hspi1, &bmi088_data);
  spl06_init(&hi2c1, &spl06_data);
  ch9141k_init();
  /**
   *  thread create
   */
  osThreadNew(task_state_machine, NULL, &task_sm_attributes);
  osThreadNew(task_com, NULL, &task_com_attributes);
  // osThreadNew(task_usb, NULL, &task_usb_attributes);
  osThreadNew(task_error, NULL, &task_error_attributes);
  // 初始化完成，转换到下一个状态
  quadcopter_state = STATE_CALIBRATING;
  osThreadTerminate(osThreadGetId()); // Terminate the start task immediately
}
/**
 * @brief 核心的状态机任务
 * @details
 * - 主循环由高频的传感器事件标志驱动，保证飞行控制的实时性。
 * - 在每次循环中，非阻塞地检查事件队列，处理低频的命令事件（如起飞、降落）。
 * - 根据当前的系统状态，执行不同的逻辑。
 */
static void task_state_machine(void *argument)
{
  static uint32_t flags;
  static state_machine_event_enum queue_sm_evt;
  static osStatus_t event_status;
  static uint32_t bmi088_sync_counter = 0;
  static uint32_t spl06_sync_counter = 0;
  static uint32_t current_tick = 0;
  static const uint32_t HEARTBEAT_TIMEOUT_TICKS = 5000;
  while (1)
  {
    // 1. 等待高频传感器数据就绪 (主驱动)
    flags = osEventFlagsWait(event_measure_id, EVENT_MEASURE_BMI088_READ, osFlagsWaitAny, osWaitForever);
    // 2. 读取传感器数据
    if (flags & EVENT_MEASURE_BMI088_READ)
    {
      bmi088_read_acc_gyro(&hspi1, &bmi088_data);
    }
    // 3. 非阻塞地检查命令事件队列
    event_status = osMessageQueueGet(queue_sm_evt_id, &queue_sm_evt, NULL, 0U); // 0U超时=立即返回
    if (queue_sm_evt == EVT_CRITICAL_ERROR)
    {
      enter_error_state(); // 进入错误状态
    }
    if (event_status != osOK)
    {
      queue_sm_evt = EVT_NONE; // 如果没有收到事件，则设为无事件
    }
    // --- 4. 根据当前状态执行逻辑 ---
    switch (quadcopter_state)
    {
    case STATE_INIT: // 1.init state
      break;

    case STATE_CALIBRATING: // 2.calibration state
      if (bmi088_data.bias_ok_flag)
      {
        quadcopter_state = STATE_IDLE; // 校准完成，进入待机
      }
      break;

    case STATE_IDLE: // 3.idle state
      // 在待机状态，只响应“起飞”指令
      if (queue_sm_evt == EVT_CMD_TAKE_OFF)
      {
        g_last_heartbeat_tick = osKernelGetTickCount(); // 更新心跳时间戳
        attitude_enable_pwm(1);
        quadcopter_state = STATE_FLYING;
      }
      break;

    case STATE_FLYING: // 4.flying state
      // 在飞行状态，优先处理“降落”指令
      if (queue_sm_evt == EVT_CMD_LAND)
      {
        attitude_enable_pwm(0);
        pid_reset(&pid_pitch);
        pid_reset(&pid_roll);
        pid_reset(&pid_pitch_rate);
        pid_reset(&pid_roll_rate);
        pid_reset(&pid_height);
        attitude_reset(&attitude_data);
        bmi088_reset(&bmi088_data);
        spl06_reset(&spl06_data);
        quadcopter_state = STATE_CALIBRATING;
        continue; // 立即开始下一次循环，不再执行飞行计算
      }
      else if (queue_sm_evt == EVT_KEEP_ALIVE)
      {
        g_last_heartbeat_tick = osKernelGetTickCount(); // 更新心跳时间戳
      }
      current_tick = osKernelGetTickCount();
      // 检查当前时间与上次心跳时间的差值
      if ((current_tick - g_last_heartbeat_tick) > HEARTBEAT_TIMEOUT_TICKS)
      {
        // 超时发生！发送一个严重错误事件
        enter_error_state(); // 进入错误状态
      }
      if (++bmi088_sync_counter >= 4) // 每4次循环处理一次传感器数据，约400Hz
      {
        bmi088_sync_counter = 0; // 重置计数器
        attitude_angle_calculate(&attitude_data, &bmi088_data);
        attitude_motor_control(&attitude_data, &bmi088_data, &pid_pitch, &pid_roll, &pid_pitch_rate, &pid_roll_rate);
        // 高度控制以较低频率运行 (~50Hz)
        if (++spl06_sync_counter >= 8)
        {
          spl06_sync_counter = 0;
          spl06_read_temp_press(&hi2c1, &spl06_data);
          if (pid_height.setpoint > 0.1f || pid_height.setpoint < -0.1f) // 仅当设定点大于0.1时才进行高度控制
          {
            attitude_height_control(&attitude_data, &spl06_data, &pid_height);
          }
        }
      }
      break;

    case STATE_ERROR: // 5.error state
      // 终结状态，只闪烁错误灯
      light_control(LIGHT_ERROR, LIGHT_BLINK);
      osDelay(200);
      break;
    // 意外状态，直接进入错误状态以策安全
    default:
      enter_error_state();
      break;
    }
  }
}

/* Communication task */
static void task_com(void *argument)
{
  com_queue_struct recv_msg;
  static char tx_buf[BUFFER_SIZE]; // 用于发送数据的缓冲区
  static uint8_t linear_buffer[BUFFER_SIZE];
  osStatus_t status;
  while (1)
  {
    status = osMessageQueueGet(queue_com_id, &recv_msg, NULL, osWaitForever);
    if (status == osOK && recv_msg.source == COM_UART_READ)
    {
      // --- 开始处理环形缓冲区 ---
      // 检查数据是否跨越了缓冲区边界（回卷）
      if (recv_msg.start_pos + recv_msg.length > BUFFER_SIZE)
      {
        // 情况一：数据已回卷，需要分两次复制
        // 1. 复制从 start_pos 到缓冲区末尾的部分
        uint32_t first_chunk_len = BUFFER_SIZE - recv_msg.start_pos;
        memcpy(linear_buffer, &dma_receive_buffer[recv_msg.start_pos], first_chunk_len);
        // 2. 复制从缓冲区开头到剩余结尾的部分
        uint32_t second_chunk_len = recv_msg.length - first_chunk_len;
        memcpy(&linear_buffer[first_chunk_len], dma_receive_buffer, second_chunk_len);
      }
      else
      {
        // 情况二：数据未回卷，是连续的，只需复制一次
        memcpy(linear_buffer, &dma_receive_buffer[recv_msg.start_pos], recv_msg.length);
      }
      // --- 环形缓冲区处理完毕 ---
      // 现在，linear_buffer 中包含了完整、连续的一帧数据
      // 将这个干净的线性缓冲区传递给解析函数
      com_parse_buf(&com_data, linear_buffer, recv_msg.length);
      com_execute_command(tx_buf); // 执行命令
    }
    else if (recv_msg.source == COM_UART_SEND)
    {
      // light_control(LIGHT_ERROR, LIGHT_BLINK);
      int len = sprintf(tx_buf, "angle:%f,%f,pwm:%d,%d,%d,%d\r\n",
                        attitude_data.angle[PITCH], attitude_data.angle[ROLL],
                        attitude_data.pwm[0], attitude_data.pwm[1], attitude_data.pwm[2], attitude_data.pwm[3]);
      if (len > 0)
      {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, len);
      }
    }
  }
}

/* USB task */
static void task_usb(void *argument)
{
  light_control(LIGHT_TAKE_OFF, LIGHT_BLINK);
  while (1)
  {
    tud_task();
    light_control(LIGHT_TAKE_OFF, LIGHT_BLINK);
    // Future USB communication logic
    // if (tud_cdc_connected())
    // {
    //   tud_cdc_write_str("Hello from STM32H5 + TinyUSB!\r\n");
    //   tud_cdc_write_flush(); // 确保数据发出
    // }
    osDelay(1000);
  }
}
/**
 * @brief 错误处理任务 (已修改)
 * @details 当收到内部错误时，向状态机发送一个严重错误事件。
 */
static void task_error(void *argument)
{
  osStatus_t status;
  while (1)
  {
    status = osMessageQueueGet(queue_error_id, &error_queue, NULL, osWaitForever);
    if (status == osOK)
    {
      // 发生内部错误，通知状态机
      state_machine_event_enum queue_sm_evt = EVT_CRITICAL_ERROR;
      osMessageQueuePut(queue_sm_evt_id, &queue_sm_evt, 0, 0);
      // 状态机将接管后续所有安全操作
    }
  }
}
/* EXTI interrupt callback */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{

  if (GPIO_Pin == BMI088_ACCEL_INT_PIN)
  {
    osEventFlagsSet(event_measure_id, EVENT_MEASURE_BMI088_READ);
  }
  if (GPIO_Pin == SPL06_INT_PIN)
  {
    osEventFlagsSet(event_measure_id, EVENT_MEASURE_SPL06_READ);
  }
}
/* UART idle interrupt callback */
void COM_Uart_Idle_Callback()
{
  static uint32_t old_pos = 0;
  static uint32_t idle_counter = 0;
  const uint32_t IDLE_TIMEOUT_MS = 10; // 定义10ms为超时（即连续10次轮询无数据）

  // 获取DMA当前写入位置
  uint32_t current_pos = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

  // 1. 检查DMA位置是否变化
  if (current_pos != old_pos)
  {
    // 位置变化了，说明在过去的1ms内有新数据到达
    idle_counter = 0;      // 重置空闲计数器
    old_pos = current_pos; // 更新位置“书签”
  }
  else
  {
    // 位置没变，说明过去的1ms内没有新数据
    idle_counter++; // 增加空闲计数
  }

  // 2. 检查是否达到超时条件
  if (idle_counter >= IDLE_TIMEOUT_MS)
  {
    // 空闲计数达到阈值，我们判定一帧数据已结束
    idle_counter = 0; // 清零，为下一帧做准备

    // 计算最终的长度。此时old_pos是上一次有数据活动时的位置
    // 我们需要一个变量记录一帧的真实起始点
    static uint32_t frame_start_pos = 0;
    uint32_t length = (old_pos - frame_start_pos + BUFFER_SIZE) % BUFFER_SIZE;

    if (length > 0)
    {
      // 找到了完整的一帧，发送消息给任务
      com_queue_struct msg = {.source = COM_UART_READ, .start_pos = frame_start_pos, .length = length};
      osMessageQueuePut(queue_com_id, &msg, 0, 0);

      // 更新下一帧的起始点
      frame_start_pos = old_pos;
    }
  }
}
static void timer_send_callback(void *argument)
{
  com_queue_struct msg = {.source = COM_UART_SEND, .start_pos = 0, .length = 0};
  osMessageQueuePut(queue_com_id, &msg, 0, 0);
}
/**
 * @brief 辅助函数，用于执行进入错误状态时的所有关键操作
 */
static void enter_error_state(void)
{
  // 1. 安全第一：立即关闭所有电机
  attitude_enable_pwm(0);

  // 2. 强制将系统状态设置为错误
  quadcopter_state = STATE_ERROR;

  // 3. （可选）可以在此处记录具体的错误信息

  // 状态机的 ERROR case 将会接管后续的错误指示（如LED闪烁）
}
/**
 * @brief 执行上位机发送过来的指令
 */
static void com_execute_command(char *tx_buf)
{
  // *** 关键改动：将命令转换为事件 ***
  state_machine_event_enum queue_sm_evt = EVT_NONE;
  switch (com_data.command_type)
  {
  case COM_NONE:
    return;
    break;
  case COM_TAKE_OFF:
    if (com_data.take_off)
    {
      if (quadcopter_state == STATE_IDLE)
      {
        queue_sm_evt = EVT_CMD_TAKE_OFF;
      }
    }
    else
    {
      if (quadcopter_state == STATE_FLYING)
      {
        queue_sm_evt = EVT_CMD_LAND;
      }
    }
    break;
  case COM_PID_SET:
    break;
  case COM_PID_GET:
    break;
  case COM_ADD_HEIGHT:
    if (quadcopter_state == STATE_FLYING)
    {
      if (pid_height.setpoint < 0.1f && pid_height.setpoint > -0.1f)
      {
        pid_height.setpoint = spl06_data.pressure;
      }
      pid_height.setpoint -= 5.0f; // 增加高度设定点
    }
    break;
  case COM_MINUS_HEIGHT:
    if (quadcopter_state == STATE_FLYING)
    {
      if (pid_height.setpoint < 0.1f && pid_height.setpoint > -0.1f)
      {
        pid_height.setpoint = spl06_data.pressure;
      }
      pid_height.setpoint += 5.0f; // 减低高度设定点
    }
    break;
  case COM_ADD_THROTTLE:
    if (quadcopter_state == STATE_FLYING)
    {
      attitude_data.throttle += 100; // 增加油门
    }
    break;
  case COM_MINUS_THROTTLE:
    if (quadcopter_state == STATE_FLYING)
    {
      if (attitude_data.throttle > 0)
        attitude_data.throttle -= 100;
    }
    break;
  case COM_ENABLE_LOG:
    osTimerStart(timer_send_id, 100);
    break;
  case COM_DISABLE_LOG:
    osTimerStop(timer_send_id);
    break;
  case COM_KEEP_ALIVE:
    queue_sm_evt = EVT_KEEP_ALIVE; // 发送心跳事件
    int len = sprintf(tx_buf, "$ACK_KEP\r\n");
    if (len > 0)
    {
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, len);
    }
    break;
  default:
    break;
  }
  if (queue_sm_evt != EVT_NONE)
  {
    // 发送事件到状态机队列
    osMessageQueuePut(queue_sm_evt_id, &queue_sm_evt, 0, 0);
  }
  com_data.command_type = COM_NONE; // reset command type after processing
}