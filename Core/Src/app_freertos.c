/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : FreeRTOS applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for startTask */
osThreadId_t startTaskHandle;
const osThreadAttr_t startTask_attributes = {
  .name = "startTask",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 128 * 4
};
/* Definitions for idleTask */
osThreadId_t idleTaskHandle;
const osThreadAttr_t idleTask_attributes = {
  .name = "idleTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* USER CODE BEGIN 4 */
#include "light.h"
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   (void)xTask;
   (void)pcTaskName;

   // 在这里设置一个断点！
   __asm("BKPT #0"); // 这是一条汇编指令，会使程序在此处暂停，进入调试状态

   while(1)
   {
      // 可以在这里翻转一个错误指示灯，表示发生了严重错误
      light_control(LIGHT_ERROR, LIGHT_BLINK); // 例如，闪烁错误灯
      osDelay(1000); // 延时1秒，避免过快循环
   }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of startTask */
  startTaskHandle = osThreadNew(task_start, NULL, &startTask_attributes);

  /* creation of idleTask */
  idleTaskHandle = osThreadNew(idle_task, NULL, &idleTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_task_start */
/**
 * @brief Function implementing the startTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_start */
void task_start(void *argument)
{
  /* USER CODE BEGIN startTask */
  /* Infinite loop */
  app_start(); // Start the application
  for (;;)
  {
    light_control(LIGHT_ERROR, LIGHT_BLINK); // Turn on the take-off light
    osDelay(200);
  }
  /* USER CODE END startTask */
}

/* USER CODE BEGIN Header_idle_task */
/**
 * @brief Function implementing the idleTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_idle_task */
void idle_task(void *argument)
{
  /* USER CODE BEGIN idleTask */
  /* Infinite loop */
  for (;;)
  {
    light_control(LIGHT_TAKE_OFF, LIGHT_BLINK); // Turn on the take-off light
    osDelay(200);
  }
  /* USER CODE END idleTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

