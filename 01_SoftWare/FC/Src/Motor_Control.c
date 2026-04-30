#include "Motor_Control.h"

#include "cmsis_os.h"
#include "tim.h"

#define DSHOT_BITS                 16U
#define DSHOT_RESET_BITS           24U
#define DSHOT_BUFFER_LEN           (DSHOT_BITS + DSHOT_RESET_BITS)

#define DSHOT_TIMER_ARR            212U
#define DSHOT_0_DUTY               80U
#define DSHOT_1_DUTY               160U

#define MOTOR_TEST_THROTTLE        600U
#define MOTOR_ARM_TIME_MS          3000U
#define MOTOR_FRAME_PERIOD_MS      1U

extern DMA_HandleTypeDef hdma_tim5_ch1;
extern DMA_HandleTypeDef hdma_tim5_ch2;

static uint8_t s_motor_inited = 0U;

__attribute__((section(".ram_d1"), aligned(32)))
static uint16_t s_motor_tim3_buf[2][DSHOT_BUFFER_LEN];

__attribute__((section(".ram_d1"), aligned(32)))
static uint32_t s_motor_tim5_buf[2][DSHOT_BUFFER_LEN];

static void Motor_DelayMs(uint32_t delay_ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static void Motor_WaitNextFrame(uint32_t *next_tick)
{
  if (osKernelGetState() == osKernelRunning)
  {
    *next_tick += MOTOR_FRAME_PERIOD_MS;
    if (osDelayUntil(*next_tick) != osOK)
    {
      *next_tick = osKernelGetTickCount();
    }
  }
  else
  {
    HAL_Delay(MOTOR_FRAME_PERIOD_MS);
  }
}

static uint16_t DShot_MakePacket(uint16_t throttle)
{
  uint16_t value;
  uint16_t crc;

  if (throttle > 2047U)
  {
    throttle = 2047U;
  }

  value = (uint16_t)(throttle << 1); /* telemetry = 0 */
  crc = (uint16_t)((value ^ (value >> 4) ^ (value >> 8)) & 0x0FU);

  return (uint16_t)((value << 4) | crc);
}

static void DShot_FillTim3Buffer(uint8_t index, uint16_t throttle)
{
  uint16_t packet = DShot_MakePacket(throttle);
  uint8_t i;

  for (i = 0U; i < DSHOT_BITS; i++)
  {
    s_motor_tim3_buf[index][i] = ((packet & 0x8000U) != 0U) ? DSHOT_1_DUTY : DSHOT_0_DUTY;
    packet <<= 1;
  }

  for (i = DSHOT_BITS; i < DSHOT_BUFFER_LEN; i++)
  {
    s_motor_tim3_buf[index][i] = 0U;
  }
}

static void DShot_FillTim5Buffer(uint8_t index, uint16_t throttle)
{
  uint16_t packet = DShot_MakePacket(throttle);
  uint8_t i;

  for (i = 0U; i < DSHOT_BITS; i++)
  {
    s_motor_tim5_buf[index][i] = ((packet & 0x8000U) != 0U) ? DSHOT_1_DUTY : DSHOT_0_DUTY;
    packet <<= 1;
  }

  for (i = DSHOT_BITS; i < DSHOT_BUFFER_LEN; i++)
  {
    s_motor_tim5_buf[index][i] = 0U;
  }
}

static void DShot_ReinitTim5DmaWord(void)
{
  HAL_DMA_DeInit(&hdma_tim5_ch1);
  HAL_DMA_DeInit(&hdma_tim5_ch2);

  hdma_tim5_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim5_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  if (HAL_DMA_Init(&hdma_tim5_ch1) != HAL_OK)
  {
    Error_Handler();
  }

  hdma_tim5_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_tim5_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  if (HAL_DMA_Init(&hdma_tim5_ch2) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&htim5, hdma[TIM_DMA_ID_CC1], hdma_tim5_ch1);
  __HAL_LINKDMA(&htim5, hdma[TIM_DMA_ID_CC2], hdma_tim5_ch2);
}

static void DShot_ConfigTimers(void)
{
  htim3.Instance->CR1 &= ~TIM_CR1_CEN;
  htim5.Instance->CR1 &= ~TIM_CR1_CEN;

  __HAL_TIM_SET_PRESCALER(&htim3, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim3, DSHOT_TIMER_ARR);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  htim3.Instance->EGR = TIM_EGR_UG;

  __HAL_TIM_SET_PRESCALER(&htim5, 0U);
  __HAL_TIM_SET_AUTORELOAD(&htim5, DSHOT_TIMER_ARR);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COUNTER(&htim5, 0U);
  htim5.Instance->EGR = TIM_EGR_UG;
}

static void DShot_StartPwmOutputs(void)
{
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void DShot_SetAllLow(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0U);
}

static void DShot_DisableDmaRequests(void)
{
  __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC3 | TIM_DMA_CC4);
  __HAL_TIM_DISABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2);
}

static void DShot_WaitResetTail(void)
{
  uint32_t i;

  for (i = 0U; i < DSHOT_RESET_BITS; i++)
  {
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    while (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) == RESET)
    {
    }
  }
}

static void DShot_WaitDmaDone(void)
{
  uint32_t tick_start = HAL_GetTick();

  while ((HAL_DMA_GetState(htim3.hdma[TIM_DMA_ID_CC3]) != HAL_DMA_STATE_READY) ||
         (HAL_DMA_GetState(htim3.hdma[TIM_DMA_ID_CC4]) != HAL_DMA_STATE_READY) ||
         (HAL_DMA_GetState(htim5.hdma[TIM_DMA_ID_CC1]) != HAL_DMA_STATE_READY) ||
         (HAL_DMA_GetState(htim5.hdma[TIM_DMA_ID_CC2]) != HAL_DMA_STATE_READY))
  {
    if ((HAL_GetTick() - tick_start) > 10U)
    {
      Error_Handler();
    }
  }
}

static void DShot_StartDmaAll(void)
{
  if (HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_CC3],
                       (uint32_t)s_motor_tim3_buf[0],
                       (uint32_t)&htim3.Instance->CCR3,
                       DSHOT_BUFFER_LEN) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_CC4],
                       (uint32_t)s_motor_tim3_buf[1],
                       (uint32_t)&htim3.Instance->CCR4,
                       DSHOT_BUFFER_LEN) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_Start_IT(htim5.hdma[TIM_DMA_ID_CC1],
                       (uint32_t)s_motor_tim5_buf[0],
                       (uint32_t)&htim5.Instance->CCR1,
                       DSHOT_BUFFER_LEN) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_Start_IT(htim5.hdma[TIM_DMA_ID_CC2],
                       (uint32_t)s_motor_tim5_buf[1],
                       (uint32_t)&htim5.Instance->CCR2,
                       DSHOT_BUFFER_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC3 | TIM_DMA_CC4);
  __HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2);
}

static void DShot_Send4(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
  DShot_FillTim3Buffer(0U, motor1);
  DShot_FillTim3Buffer(1U, motor2);
  DShot_FillTim5Buffer(0U, motor3);
  DShot_FillTim5Buffer(1U, motor4);

  DShot_DisableDmaRequests();
  htim3.Instance->CR1 &= ~TIM_CR1_CEN;
  htim5.Instance->CR1 &= ~TIM_CR1_CEN;
  DShot_SetAllLow();
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim5, 0U);

  DShot_StartDmaAll();
  __HAL_TIM_ENABLE(&htim3);
  __HAL_TIM_ENABLE(&htim5);
  DShot_WaitDmaDone();
  DShot_WaitResetTail();
  DShot_DisableDmaRequests();
  DShot_SetAllLow();
}

static void DShot_SendAll(uint16_t throttle)
{
  DShot_Send4(throttle, throttle, throttle, throttle);
}

void Motor_Init(void)
{
  DShot_ReinitTim5DmaWord();
  DShot_ConfigTimers();
  DShot_SetAllLow();
  DShot_StartPwmOutputs();
  s_motor_inited = 1U;
}

uint8_t Motor_Write(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
  if (s_motor_inited == 0U)
  {
    Motor_Init();
  }

  DShot_Send4(motor1, motor2, motor3, motor4);
  return 1U;
}

void Motor_Stop(void)
{
  if (s_motor_inited == 0U)
  {
    Motor_Init();
  }

  DShot_SendAll(0U);
}

void Motor_Test_Run(void)
{
  uint32_t i;
  uint32_t next_tick = 0U;
  uint32_t stop_frames = MOTOR_ARM_TIME_MS / MOTOR_FRAME_PERIOD_MS;

  Motor_Init();
  Motor_DelayMs(1000U);

  if (osKernelGetState() == osKernelRunning)
  {
    next_tick = osKernelGetTickCount();
  }

  for (i = 0U; i < stop_frames; i++)
  {
    DShot_SendAll(0U);
    Motor_WaitNextFrame(&next_tick);
  }

  while (1)
  {
    DShot_SendAll(MOTOR_TEST_THROTTLE);
    Motor_WaitNextFrame(&next_tick);
  }
}
