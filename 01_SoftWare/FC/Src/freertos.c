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
#include "Motor_Control.h"
#include "../Peripherals/ICM42688P.h"
#include "../Peripherals/IST8310.h"
#include "Kalman.h"
#include "PID_LOOP.h"
#include "crsf.h"
#include "../Peripherals/uart.h"
#include <stdio.h>
#include <limits.h>
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
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_Control */
osThreadId_t LED_ControlHandle;
const osThreadAttr_t LED_Control_attributes = {
  .name = "LED_Control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Attitude_Algori */
osThreadId_t Attitude_AlgoriHandle;
const osThreadAttr_t Attitude_Algori_attributes = {
  .name = "Attitude_Algori",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PID_LOOP */
osThreadId_t PID_LOOPHandle;
const osThreadAttr_t PID_LOOP_attributes = {
  .name = "PID_LOOP",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void __attribute__((unused)) Mag_Calibration_Run(void);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* creation of LED_Control */
  LED_ControlHandle = osThreadNew(StartTask02, NULL, &LED_Control_attributes);

  /* creation of Attitude_Algori */
  Attitude_AlgoriHandle = osThreadNew(StartTask03, NULL, &Attitude_Algori_attributes);

  /* creation of PID_LOOP */
  PID_LOOPHandle = osThreadNew(StartTask04, NULL, &PID_LOOP_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  (void)CRSF_Init();

  /* Infinite loop */
  for(;;)
  {
    CRSF_Update();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the LED_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		
		HAL_GPIO_TogglePin(GPIOE,LED0_Pin);
		
    osDelay(500);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Attitude_Algori thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  int16_t pitch_cd = 0;
  int16_t roll_cd = 0;
  int16_t yaw_cd = 0;
  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;
  char tx_buf[128];
  int tx_len = 0;

  (void)ICM42688P_Init();
  (void)IST8310_Init();
  UART_Init();
//  Mag_Calibration_Run();
  Kalman_SetMagCalibration(156.0f, -18.0f, -2.0f, 0.9662f, 1.0601f, 0.9786f);
  Kalman_Init();
  Kalman_CalibrateGyroBias(150U, 10U);

  for (uint32_t i = 0; i < 75U; i++)
  {
    (void)Kalman_GetAttitude(&pitch_cd, &roll_cd, &yaw_cd);
    osDelay(20);
  }
  Kalman_ResetReference();
  pitch_cd = 0;
  roll_cd = 0;
  yaw_cd = 0;

  /* Infinite loop */
  for(;;)
  {
    (void)Kalman_GetAttitude(&pitch_cd, &roll_cd, &yaw_cd);
    Kalman_GetGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);
    PID_LOOP_UpdateAttitude(pitch_cd, roll_cd, yaw_cd, gyro_x_dps, gyro_y_dps, gyro_z_dps);
    tx_len = snprintf(tx_buf, sizeof(tx_buf),
                      "crsf=%u,init=%u,st=%u,rxs=%u,ue=%lu,ds=%u,de=%lu,byte=%lu,pos=%u,last=%02X,frame=%u,err=%u,ch1=%u,ch2=%u,ch3=%u,ch4=%u\r\n",
                      CRSF_IsConnected(),
                      CRSF_GetInitOk(),
                      CRSF_GetInitStatus(),
                      CRSF_GetUartRxState(),
                      (unsigned long)CRSF_GetUartError(),
                      CRSF_GetDmaState(),
                      (unsigned long)CRSF_GetDmaError(),
                      (unsigned long)CRSF_GetByteCount(),
                      CRSF_GetDmaPos(),
                      CRSF_GetLastByte(),
                      CRSF_GetFrameCount(),
                      CRSF_GetErrorCount(),
                      CRSF_GetChannelUs(0),
                      CRSF_GetChannelUs(1),
                      CRSF_GetChannelUs(2),
                      CRSF_GetChannelUs(3));
    if (tx_len > 0)
    {
      (void)UART_SendBuffer((const uint8_t *)tx_buf, (uint16_t)tx_len);
    }
		
    osDelay(20);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the PID_LOOP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  PID_LOOP_Run();
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void __attribute__((unused)) Mag_Calibration_Run(void)
{
  int16_t mx = 0;
  int16_t my = 0;
  int16_t mz = 0;
  int16_t min_x = INT16_MAX;
  int16_t min_y = INT16_MAX;
  int16_t min_z = INT16_MAX;
  int16_t max_x = INT16_MIN;
  int16_t max_y = INT16_MIN;
  int16_t max_z = INT16_MIN;
  float offset_x = 0.0f;
  float offset_y = 0.0f;
  float offset_z = 0.0f;
  float range_x = 1.0f;
  float range_y = 1.0f;
  float range_z = 1.0f;
  float avg_range = 1.0f;
  float scale_x = 1.0f;
  float scale_y = 1.0f;
  float scale_z = 1.0f;
  uint32_t start_tick = HAL_GetTick();
  char tx[160];
  int len = 0;
  int32_t offset_x100 = 0;
  int32_t offset_y100 = 0;
  int32_t offset_z100 = 0;
  int32_t scale_x10000 = 10000;
  int32_t scale_y10000 = 10000;
  int32_t scale_z10000 = 10000;

  while ((UART_IsConnected() == 0U) && ((HAL_GetTick() - start_tick) < 5000U))
  {
    osDelay(20);
  }

  (void)UART_SendText("MAG CAL START: rotate all axes for 30s\r\n");
  start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < 30000U)
  {
    if (IST8310_ReadMagRaw(&mx, &my, &mz) != 0U)
    {
      if (mx < min_x) min_x = mx;
      if (my < min_y) min_y = my;
      if (mz < min_z) min_z = mz;
      if (mx > max_x) max_x = mx;
      if (my > max_y) max_y = my;
      if (mz > max_z) max_z = mz;
    }

    osDelay(20);
  }

  offset_x = ((float)max_x + (float)min_x) * 0.5f;
  offset_y = ((float)max_y + (float)min_y) * 0.5f;
  offset_z = ((float)max_z + (float)min_z) * 0.5f;
  range_x = ((float)max_x - (float)min_x) * 0.5f;
  range_y = ((float)max_y - (float)min_y) * 0.5f;
  range_z = ((float)max_z - (float)min_z) * 0.5f;

  if ((range_x > 1.0f) && (range_y > 1.0f) && (range_z > 1.0f))
  {
    avg_range = (range_x + range_y + range_z) / 3.0f;
    scale_x = avg_range / range_x;
    scale_y = avg_range / range_y;
    scale_z = avg_range / range_z;
  }

  Kalman_SetMagCalibration(offset_x, offset_y, offset_z, scale_x, scale_y, scale_z);
  offset_x100 = (int32_t)(offset_x * 100.0f);
  offset_y100 = (int32_t)(offset_y * 100.0f);
  offset_z100 = (int32_t)(offset_z * 100.0f);
  scale_x10000 = (int32_t)(scale_x * 10000.0f);
  scale_y10000 = (int32_t)(scale_y * 10000.0f);
  scale_z10000 = (int32_t)(scale_z * 10000.0f);

  len = snprintf(tx, sizeof(tx),
                 "MAG CAL DONE\r\n"
                 "min=%d,%d,%d max=%d,%d,%d\r\n"
                 "offset_x100=%ld,%ld,%ld scale_x10000=%ld,%ld,%ld\r\n",
                 min_x, min_y, min_z, max_x, max_y, max_z,
                 (long)offset_x100, (long)offset_y100, (long)offset_z100,
                 (long)scale_x10000, (long)scale_y10000, (long)scale_z10000);
  if (len > 0)
  {
    (void)UART_SendBuffer((const uint8_t *)tx, (uint16_t)len);
  }
}

/* USER CODE END Application */

