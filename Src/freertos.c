/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "robot.h"
#include "board_pins.h"
#include "arm.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MissionTask */
osThreadId_t MissionTaskHandle;
const osThreadAttr_t MissionTask_attributes = {
  .name = "MissionTask",
  /* Coupled XY alignment plus float diagnostics exceeds the former 1 KB
   * stack (Keil's printf indirect paths are not included in Max Depth). */
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LogTask */
osThreadId_t LogTaskHandle;
const osThreadAttr_t LogTask_attributes = {
  .name = "LogTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartTask02, NULL, &ControlTask_attributes);

  /* creation of MissionTask */
  MissionTaskHandle = osThreadNew(StartTask03, NULL, &MissionTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartTask04, NULL, &ImuTask_attributes);

  /* creation of LogTask */
  LogTaskHandle = osThreadNew(StartTask05, NULL, &LogTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
    robot_bt_service();           /* 蓝牙遥控/BT 命令解析(test.c):20ms 轮询够跟手 */
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* ControlTask(High)：1ms 精确速度环，用 osDelayUntil 防漂 */
  uint32_t wake = osKernelGetTickCount();
  for(;;)
  {
    osDelayUntil(wake + 1u);
    wake += 1u;
    robot_control_tick_1ms();
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the MissionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* MissionTask(AboveNormal)：阻塞式整场脚本。先等 BT 'g' 启动,再顺序跑
     读QR/越障/EOD/ANTI/RESCUE,到 DONE/ABORT 后驻留。脚本内 osDelay 让出
     CPU,ControlTask(High) 照常 1ms 抢占。 */
  for(;;)
  {
    robot_mission_main();
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* ImuTask(Normal)：周期做健康检查/协议解析(当前协议未定，几乎空转) */
  for(;;)
  {
    osDelay(5);
    robot_imu_tick();
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the LogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* LogTask(Low)：100ms 往调试口(蓝牙)打一帧状态摘要 */
  for(;;)
  {
    osDelay(100);
    robot_log_tick();
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static void rtos_emergency_stop(void)
{
  taskDISABLE_INTERRUPTS();
  arm_stepper_clock_stop();
  ARM_AXIS0_STEP_GPIO_Port->BSRR = ARM_AXIS0_STEP_Pin;
  ARM_AXIS1_STEP_GPIO_Port->BSRR = ARM_AXIS1_STEP_Pin;
  GPIOC->BSRR = (uint32_t)GPIO_PIN_8 << 16u;
  bp_laser_emergency_off();
  for (;;) { }
}

/* 栈溢出/动态内存申请失败都属于不可继续的控制故障。 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  rtos_emergency_stop();
}

void vApplicationMallocFailedHook(void)
{
  rtos_emergency_stop();
}

/* 蓝牙 diag 使用。HighWaterMark 单位是 StackType_t word，不是字节。 */
void robot_get_stack_watermarks(uint32_t out[5])
{
  if (!out) return;
  out[0] = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)defaultTaskHandle);
  out[1] = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)ControlTaskHandle);
  out[2] = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)MissionTaskHandle);
  out[3] = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)ImuTaskHandle);
  out[4] = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)LogTaskHandle);
}

/* USER CODE END Application */

