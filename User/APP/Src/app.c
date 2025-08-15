/**
 * @file     app.c
 * @brief    CMSIS-RTOS2 version of quadcopter app
 */

#include "app.h"
/**
 * global data to debug 
 */
bmi088_data_struct *bmi_debug = NULL;
spl06_data_struct *spl_debug = NULL;
attitude_data_struct *attitude_debug = NULL;
pid_data_struct *pid_debug = NULL;
com_data_struct *com_data = NULL;
volatile quadcopter_state_enum *quadcopter_state_debug = 0;
static void app_get_debug_data(void);
static void app_get_debug_data(void){
    bmi_debug = bmi088_get_data();
    spl_debug = spl06_get_data();
    attitude_debug = attitude_get_data();
    pid_debug = pid_get_data();
    quadcopter_state_debug = quadcopter_state_get_ptr();
    com_data = com_get_data();
}
/* Function declarations */
static void timer_send_callback(void *argument);
static void task_state_machine(void *argument);
static void task_com(void *argument);
static void task_usb(void *argument);
static void task_error(void *argument);
static void enter_error_state(void); // 进入错误状态的辅助函数
static void app_error_handler(error_queue_struct error_queue);
static void app_command_event_handler(void *argument);
static void app_command_timer_handler(void *argument);
static void app_command_keep_alive_handler(void *argument);
uint8_t dma_receive_buffer[BUFFER_SIZE];
static error_queue_struct error_queue = {0};
/* Semaphore */
/* Event flags */
#define EVENT_MEASURE_BMI088_READ (1 << 0)
#define EVENT_MEASURE_SPL06_READ (1 << 1)
static osEventFlagsId_t event_measure_id;
/* Message queue */
#define QUEUE_COM_MSG_NUM 4
static osMessageQueueId_t queue_com_id = NULL;
#define QUEUE_ERROR_MSG_NUM 1
static osMessageQueueId_t queue_error_id = NULL;
#define QUEUE_SM_EVENT_MSG_NUM 8
static osMessageQueueId_t queue_sm_evt_id = NULL;
/* timer */
static osTimerId_t timer_send_id = NULL;
/* Thread attributes */
static const osThreadAttr_t task_sm_attributes = {
    .name = "stateMachineTask",
    .priority = osPriorityRealtime5,
    .stack_size = 2048};
static const osThreadAttr_t task_com_attributes = {
    .name = "commTask",
    .priority = osPriorityLow2,
    .stack_size = 2048};
static const osThreadAttr_t task_usb_attributes = {
    .name = "usbTask",
    .priority = osPriorityLow3,
    .stack_size = 2048};
static const osThreadAttr_t task_error_attributes = {
    .name = "errorTask",
    .priority = osPriorityRealtime6,
    .stack_size = 512};
/* Application start */
void app_start(void)
{
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
  /**
   * timer create
   */
  timer_send_id = osTimerNew(timer_send_callback, osTimerPeriodic, NULL, NULL);
  osTimerStop(timer_send_id);
  /**
   * peripheral initialization
   */
  error_register_callback(app_error_handler);
  com_register_callback(app_command_event_handler);
  com_register_callback(app_command_timer_handler); 
  com_register_callback(app_command_keep_alive_handler);
  dwt_init(); // Initialize DWT for cycle counting
  HAL_UART_Receive_DMA(&huart1, dma_receive_buffer, sizeof(dma_receive_buffer));
  HAL_TIM_Base_Start_IT(&htim6);
  ch9141k_init();
  light_off_all();
  // tusb_init();
  bmi088_init(&hspi1);
  spl06_init(&hi2c1);
  attitude_init(2.0f, 0.005f);
#ifdef ATTITUDE_USING_QUATERNION
  pid_init(PID_PITCH, 3.5f, 0.0f, 0.0f, 200, 400);
  pid_init(PID_ROLL, 3.5f, 0.0f, 0.0f, 200, 400);
  pid_init(PID_YAW, 2.5f, 0.0f, 0.0f, 200, 400);
  pid_init(PID_PITCH_RATE, 0.8f, 0.0f, 0.0005f, 200, 400);
  pid_init(PID_ROLL_RATE, 0.8f, 0.0f, 0.0005f, 200, 400);
  pid_init(PID_YAW_RATE, 1.0f, 0.0f, 0.0f, 200, 400);
  pid_init(PID_HEIGHT, 2.0f, 0.01f, 0.01f, 200, 400);
#else
  pid_init(PID_PITCH, 8.0f, 0.2f, 0.0f, 200, 400);
  pid_init(PID_ROLL, 8.0f, 0.2f, 0.0f, 200, 400);
  pid_init(PID_YAW, 2.0f, 0.2f, 0.0f, 200, 400);
  pid_init(PID_PITCH_RATE, 2.0f, 0.01f, 0.002f, 200, 400);
  pid_init(PID_ROLL_RATE, 2.0f, 0.01f, 0.002f, 200, 400);
  pid_init(PID_YAW_RATE, 2.0f, 0.01f, 0.002f, 200, 400);
  pid_init(PID_HEIGHT, 2.0f, 0.01f, 0.01f, 200, 400);
#endif /* ATTITUDE_USING_QUATERNION */
  /**
   *  thread create
   */
  osThreadNew(task_state_machine, NULL, &task_sm_attributes);
  osThreadNew(task_com, NULL, &task_com_attributes);
  // osThreadNew(task_usb, NULL, &task_usb_attributes);
  osThreadNew(task_error, NULL, &task_error_attributes);
  // 初始化完成，转换到下一个状态
  quadcopter_state_set(STATE_CALIBRATING);
  app_get_debug_data();
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
  static uint32_t spl06_sampling_count = 0;
  bmi088_data_struct *bmi088_data = bmi088_get_data();
  spl06_data_struct *spl06_data = spl06_get_data();
  pid_data_struct *pid_data = pid_get_data();
  while (1)
  {
    // 1. 等待高频传感器数据就绪 (主驱动)
    flags = osEventFlagsWait(event_measure_id, EVENT_MEASURE_BMI088_READ, osFlagsWaitAny, osWaitForever);
    // 2. 读取传感器数据
    if (flags & EVENT_MEASURE_BMI088_READ)
    {
      spl06_sampling_count++;
      bmi088_read_acc_gyro(&hspi1);
    }
    // if (spl06_sampling_count >= 32) // 每32次读取一次气压计数据，约50Hz
    // {
    //   spl06_sampling_count = 0;
    //   spl06_read_temp_press(&hi2c1);
    // }
    // 3. 非阻塞地检查命令事件队列
    event_status = osMessageQueueGet(queue_sm_evt_id, &queue_sm_evt, NULL, 0U); // 0U超时=立即返回
    if (event_status != osOK)
    {
      queue_sm_evt = EVT_NONE; // 如果没有收到事件，则设为无事件
    }
    // --- 4. 根据当前状态执行逻辑 ---
    flight_control_run(bmi088_data, spl06_data, pid_data, queue_sm_evt);
  }
}
/* Communication task */
static void task_com(void *argument)
{
  com_queue_struct recv_msg;
  static char tx_buf[BUFFER_SIZE]; // 用于发送数据的缓冲区
  static uint8_t linear_buffer[BUFFER_SIZE];
  osStatus_t status;
  attitude_data_struct *attitude_data = attitude_get_data();
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
      com_parse_buf(linear_buffer, recv_msg.length);
    }
    else if (recv_msg.source == COM_UART_SEND)
    {
      // light_control(LIGHT_ERROR, LIGHT_BLINK);
      int len = sprintf(tx_buf, "angle:%f,%f,pwm:%d,%d,%d,%d\r\n",
                        attitude_data->angle[PITCH], attitude_data->angle[ROLL],
                        attitude_data->pwm[0], attitude_data->pwm[1], attitude_data->pwm[2], attitude_data->pwm[3]);
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
  else if (GPIO_Pin == SPL06_INT_PIN)
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
// Timer callback to send data periodically
static void timer_send_callback(void *argument)
{
  com_queue_struct msg = {.source = COM_UART_SEND, .start_pos = 0, .length = 0};
  osMessageQueuePut(queue_com_id, &msg, 0, 0);
}
// call back to handler errors
static void app_error_handler(error_queue_struct error_queue)
{
  osMessageQueuePut(queue_error_id, &error_queue, 0, 0);
}
// callback to handle events
static void app_command_event_handler(void *argument)
{
  // 这个函数在 app.c 的上下文中执行，所以它可以安全地访问 app.c 的变量
  state_machine_event_enum event = (state_machine_event_enum)argument;
  if (event != EVT_NONE)
  {
    // 将接收到的事件放入状态机消息队列
    osMessageQueuePut(queue_sm_evt_id, &event, 0, 0);
  }
}
// call back to handle timer
static void app_command_timer_handler(void *argument)
{
  uint8_t timer_enable = (uint8_t)argument;
  if (timer_enable)
  {
    // 启动定时器发送数据
    osTimerStart(timer_send_id, 100);
  }
  else
  {
    // 停止定时器发送数据
    osTimerStop(timer_send_id);
  }
}
// call back to trigger heart
static void app_command_keep_alive_handler(void *argument)
{
  char tx_buf[11];
  state_machine_event_enum event = (state_machine_event_enum)argument; // 发送心跳事件
  osMessageQueuePut(queue_sm_evt_id, &event, 0, 0);
  // feedback to client
  int len = sprintf(tx_buf, "$ACK_KEP\r\n");
  if (len > 0)
  {
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, len);
  }
}