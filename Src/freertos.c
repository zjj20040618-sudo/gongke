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

/*
 * 初学者导读：FreeRTOS 让单个CPU轮流执行多个任务，不是五个CPU真正同时跑。
 * 任务是一段长期运行的函数，常写 for(;;) 无限循环；osDelay 会暂时让出CPU。
 * 优先级更高的就绪任务先运行，所以1ms轮速任务优先于蓝牙、任务脚本和日志。
 * osThreadNew(函数名, 参数指针, 属性地址) 创建任务，随后由调度器调用该函数。
 * .name/.stack_size 等写法是结构体指定成员初始化；这里栈大小单位是字节。
 * 任务栈保存局部变量和函数调用信息，不足时会进故障钩子并关闭电机使能/激光。
 */
#include "robot.h"
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
  .stack_size = 256 * 4,
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
  /* osThreadNew只创建任务；调用osKernelStart后，调度器才按优先级执行这些函数。 */

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
  /* 蓝牙服务每约20ms一次；osDelay按内核tick计数，本工程tick频率为1000Hz，所以20tick约20ms。 */
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
  /* wake存计划唤醒时刻；osDelayUntil等绝对tick，比“执行完再延1ms”更能减少周期漂移。 */
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
  /* 脚本阻塞的是本任务；内部osDelay让出CPU，轮速和蓝牙任务仍可运行。 */
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
  /* imu_feed已在UART4回调逐字节解析；这里每5tick检查链路超时，清失效标志。 */
  /* ImuTask(Normal)：当前协议已实现；本任务检查静默超时，字节解析在UART4接收回调。 */
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
  /* 日志低优先级，每100tick检查一次；robot_log_tick在BOOT静默，模式32另限到约1秒一帧。 */
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
  GPIOC->BSRR = (uint32_t)GPIO_PIN_8 << 16u;
  GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16u;
  taskDISABLE_INTERRUPTS();
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

