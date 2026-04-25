#include "Motor_Control.h"
#include "tim.h"
#include "cmsis_os.h"


/* ------------------------------------------------------------------ */
/* DShot600 时序参数
 *
 * 定时器时钟 128 MHz，PSC=0，ARR=212 → 周期 ~1.66 μs
 * T0H = 80  (~0.625 μs, 37.5%)
 * T1H = 160 (~1.25 μs,  75%)
 * 帧尾追加 24 个空周期拉低总线，让电调识别帧边界
 */
#define MOTOR_THROTTLE_MAX  2047U

#define DS_BITS        16U
#define DS_RESET       24U
#define DS_BUFLEN      (DS_BITS + DS_RESET)
#define DS_ARR         212U
#define DS_T0          80U
#define DS_T1          160U

/* 解锁前持续发 0 油门的时间 */
#define ARM_MS         3000U
#define FRAME_MS       1U				//帧发送周期（ms）

/* 测试油门 */
#define TEST_THR       60U

/* ------------------------------------------------------------------ */
/* DMA 缓冲区
 * TIM3 CCR 是 16 位；TIM5 CCR 是 32 位，类型必须匹配否则 DMA 写错位置
 * 放 DTCM 段绕开 DCache，省去每帧手动 flush                          */
__attribute__((section(".dtcmram")))
static uint16_t tim3_buf[2][DS_BUFLEN];

__attribute__((section(".dtcmram")))
static uint32_t tim5_buf[2][DS_BUFLEN];

/* ------------------------------------------------------------------ */
static uint16_t make_packet(uint16_t thr)
{
    if (thr > 2047U) thr = 2047U;

    uint16_t v   = (uint16_t)(thr << 1);          /* telemetry bit = 0 */
    uint16_t crc = (v ^ (v >> 4) ^ (v >> 8)) & 0x0F;
    return (uint16_t)((v << 4) | crc);
}

static void fill_tim3(uint8_t ch, uint16_t thr)
{
    uint16_t pkt = make_packet(thr);
    for (uint8_t i = 0; i < DS_BITS; i++) {
        tim3_buf[ch][i] = (pkt & 0x8000) ? DS_T1 : DS_T0;
        pkt <<= 1;
    }
    for (uint8_t i = DS_BITS; i < DS_BUFLEN; i++) tim3_buf[ch][i] = 0;
}
//fill_tim5 和 fill_tim3 逻辑完全一样，只是写入的目标 buffer 不同（tim5_buf 是 uint32_t，因为 TIM5 的 CCR 是 32 位寄存器）
static void fill_tim5(uint8_t ch, uint16_t thr)
{
    uint16_t pkt = make_packet(thr);
    for (uint8_t i = 0; i < DS_BITS; i++) {
        tim5_buf[ch][i] = (pkt & 0x8000) ? DS_T1 : DS_T0;
        pkt <<= 1;
    }
    for (uint8_t i = DS_BITS; i < DS_BUFLEN; i++) tim5_buf[ch][i] = 0;
}

/* ------------------------------------------------------------------ */
static void timers_init(void)
{
    /* 停表，重设参数，产生一次 UG 让影子寄存器生效 */
    htim3.Instance->CR1  &= ~TIM_CR1_CEN;
    htim5.Instance->CR1  &= ~TIM_CR1_CEN;

    __HAL_TIM_SET_PRESCALER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, DS_ARR);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    htim3.Instance->EGR  = TIM_EGR_UG;

    __HAL_TIM_SET_PRESCALER(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, DS_ARR);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    htim5.Instance->EGR  = TIM_EGR_UG;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
}

static void outputs_low(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
}

static void dma_disable(void)
{
    __HAL_TIM_DISABLE_DMA(&htim3, TIM_DMA_CC3 | TIM_DMA_CC4);
    __HAL_TIM_DISABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2);
}

/* 等四路 DMA 回到 READY；buffer 放 DTCM 不需要 cache flush */
static HAL_StatusTypeDef dma_wait(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_DMA_GetState(htim3.hdma[TIM_DMA_ID_CC3]) != HAL_DMA_STATE_READY ||
           HAL_DMA_GetState(htim3.hdma[TIM_DMA_ID_CC4]) != HAL_DMA_STATE_READY ||
           HAL_DMA_GetState(htim5.hdma[TIM_DMA_ID_CC1]) != HAL_DMA_STATE_READY ||
           HAL_DMA_GetState(htim5.hdma[TIM_DMA_ID_CC2]) != HAL_DMA_STATE_READY)
    {
        if ((HAL_GetTick() - t0) > timeout_ms) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

/* 等帧尾复位低电平输出完（DS_RESET 个周期），用定时器 Update Flag 计数
 * 注意：这里是裸轮询，适合裸机；若在 RTOS 任务里调用，整个 send_frame
 * 耗时约 (16+24)*1.66μs ≈ 66μs，对 1ms 周期任务影响可接受              */
static void wait_reset_tail(void)
{
    for (uint32_t i = 0; i < DS_RESET; i++) {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        while (!__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE));
    }
}

/* ------------------------------------------------------------------ */
static void send_frame(uint16_t thr)
{
    fill_tim3(0, thr);
    fill_tim3(1, thr);
    fill_tim5(0, thr);
    fill_tim5(1, thr);

    /* 停表 → 清计数 → 挂 DMA → 启表；顺序不能乱 */
    dma_disable();
    htim3.Instance->CR1 &= ~TIM_CR1_CEN;
    htim5.Instance->CR1 &= ~TIM_CR1_CEN;
    outputs_low();
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);

    HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_CC3],
                     (uint32_t)tim3_buf[0], (uint32_t)&htim3.Instance->CCR3, DS_BUFLEN);
    HAL_DMA_Start_IT(htim3.hdma[TIM_DMA_ID_CC4],
                     (uint32_t)tim3_buf[1], (uint32_t)&htim3.Instance->CCR4, DS_BUFLEN);
    HAL_DMA_Start_IT(htim5.hdma[TIM_DMA_ID_CC1],
                     (uint32_t)tim5_buf[0], (uint32_t)&htim5.Instance->CCR1, DS_BUFLEN);
    HAL_DMA_Start_IT(htim5.hdma[TIM_DMA_ID_CC2],
                     (uint32_t)tim5_buf[1], (uint32_t)&htim5.Instance->CCR2, DS_BUFLEN);

    __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_CC3 | TIM_DMA_CC4);
    __HAL_TIM_ENABLE_DMA(&htim5, TIM_DMA_CC1 | TIM_DMA_CC2);
    __HAL_TIM_ENABLE(&htim3);
    __HAL_TIM_ENABLE(&htim5);

    if (dma_wait(10) != HAL_OK) Error_Handler();
    wait_reset_tail();

    dma_disable();
    outputs_low();
}

/* ------------------------------------------------------------------ */
void Motor_Test_Run(void)
{
    timers_init();
    outputs_low();

    /* 上电后立刻发停止帧，不能加延迟
     * 电调上电约 500ms 内没收到有效 DShot 会跳过协议检测，
     * 之后发什么都没用，没有自检音基本就是这个原因 */
    uint32_t arm_frames = ARM_MS / FRAME_MS;
    uint32_t next_tick  = (osKernelGetState() == osKernelRunning)
                          ? osKernelGetTickCount() : 0;

    for (uint32_t i = 0; i < arm_frames; i++) {
        send_frame(0);
        if (osKernelGetState() == osKernelRunning) {
            next_tick += FRAME_MS;
            osDelayUntil(next_tick);
        } else {
            HAL_Delay(FRAME_MS);
        }
    }

    /* 持续发小油门，观察电机是否正常旋转 */
    while (1) {
			
        send_frame(TEST_THR);
        if (osKernelGetState() == osKernelRunning)
					{
            next_tick += FRAME_MS;
            osDelayUntil(next_tick);
					} else {
            HAL_Delay(FRAME_MS);
								 }
				
							}
}