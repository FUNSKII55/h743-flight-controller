#include "crsf.h"

#include <stddef.h>
#include "usart.h"

#define CRSF_DMA_BUF_LEN              256U
#define CRSF_MAX_FRAME_LEN            64U
#define CRSF_TIMEOUT_MS               500U

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U
#define CRSF_ADDRESS_CRSF_TRANSMITTER  0xEEU
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEAU
#define CRSF_ADDRESS_CRSF_RECEIVER     0xECU

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16U
#define CRSF_RC_PAYLOAD_LEN               22U

#define CRSF_CHANNEL_MIN_RAW          172U
#define CRSF_CHANNEL_MID_RAW          992U
#define CRSF_CHANNEL_MAX_RAW          1811U

typedef enum
{
  CRSF_PARSE_ADDR = 0,
  CRSF_PARSE_LEN,
  CRSF_PARSE_DATA
} CRSF_ParseState_t;

__attribute__((section(".ram_d1"), aligned(32)))
static uint8_t s_crsf_dma_buf[CRSF_DMA_BUF_LEN];

static uint8_t s_frame_buf[CRSF_MAX_FRAME_LEN];
static uint16_t s_dma_old_pos = 0U;
static uint8_t s_parse_state = CRSF_PARSE_ADDR;
static uint8_t s_frame_index = 0U;
static uint8_t s_frame_remaining = 0U;
static uint16_t s_channels_raw[CRSF_CHANNEL_COUNT] = {
  CRSF_CHANNEL_MID_RAW, CRSF_CHANNEL_MID_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MID_RAW,
  CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW,
  CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW,
  CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW, CRSF_CHANNEL_MIN_RAW
};
static volatile uint32_t s_last_frame_tick = 0U;
static volatile uint32_t s_byte_count = 0U;
static volatile uint16_t s_frame_count = 0U;
static volatile uint16_t s_error_count = 0U;
static volatile uint16_t s_dma_pos_debug = 0U;
static volatile uint8_t s_init_ok = 0U;
static volatile uint8_t s_init_status = 0xFFU;
static volatile uint8_t s_uart_rx_state = 0U;
static volatile uint32_t s_uart_error = 0U;
static volatile uint8_t s_dma_state = 0xFFU;
static volatile uint32_t s_dma_error = 0U;
static volatile uint8_t s_last_byte = 0U;

static uint8_t CRSF_IsAddress(uint8_t byte);
static uint8_t CRSF_CalcCrc(const uint8_t *data, uint8_t len);
static void CRSF_ParseByte(uint8_t byte);
static void CRSF_ProcessFrame(const uint8_t *frame, uint8_t total_len);
static void CRSF_UnpackChannels(const uint8_t *payload);

uint8_t CRSF_Init(void)
{
  s_dma_old_pos = 0U;
  s_parse_state = CRSF_PARSE_ADDR;
  s_frame_index = 0U;
  s_frame_remaining = 0U;
  s_frame_count = 0U;
  s_error_count = 0U;
  s_byte_count = 0U;
  s_dma_pos_debug = 0U;
  s_init_ok = 0U;
  s_init_status = 0xFFU;
  s_last_byte = 0U;
  s_last_frame_tick = 0U;

  (void)HAL_UART_AbortReceive(&huart2);
  if (huart2.hdmarx != NULL)
  {
    (void)HAL_DMA_Abort(huart2.hdmarx);
  }

  s_init_status = (uint8_t)HAL_UART_Receive_DMA(&huart2, s_crsf_dma_buf, CRSF_DMA_BUF_LEN);
  s_uart_rx_state = (uint8_t)huart2.RxState;
  s_uart_error = huart2.ErrorCode;
  if (huart2.hdmarx != NULL)
  {
    s_dma_state = (uint8_t)HAL_DMA_GetState(huart2.hdmarx);
    s_dma_error = huart2.hdmarx->ErrorCode;
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
  else
  {
    s_dma_state = 0xFEU;
    s_dma_error = 0xFFFFFFFFU;
  }

  if (s_init_status != (uint8_t)HAL_OK)
  {
    return 0U;
  }

  s_init_ok = 1U;
  return 1U;
}

void CRSF_Update(void)
{
  uint16_t dma_pos = 0U;

  if (huart2.hdmarx == NULL)
  {
    return;
  }

  dma_pos = (uint16_t)(CRSF_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
  if (dma_pos >= CRSF_DMA_BUF_LEN)
  {
    dma_pos = 0U;
  }
  s_dma_pos_debug = dma_pos;

  while (s_dma_old_pos != dma_pos)
  {
    s_last_byte = s_crsf_dma_buf[s_dma_old_pos];
    s_byte_count++;
    CRSF_ParseByte(s_last_byte);
    s_dma_old_pos++;
    if (s_dma_old_pos >= CRSF_DMA_BUF_LEN)
    {
      s_dma_old_pos = 0U;
    }
  }
}

uint8_t CRSF_IsConnected(void)
{
  return ((HAL_GetTick() - s_last_frame_tick) < CRSF_TIMEOUT_MS) ? 1U : 0U;
}

uint16_t CRSF_GetChannelRaw(uint8_t channel)
{
  if (channel >= CRSF_CHANNEL_COUNT)
  {
    return CRSF_CHANNEL_MID_RAW;
  }

  return s_channels_raw[channel];
}

uint16_t CRSF_GetChannelUs(uint8_t channel)
{
  uint32_t raw = CRSF_GetChannelRaw(channel);

  if (raw <= CRSF_CHANNEL_MIN_RAW)
  {
    return 1000U;
  }

  if (raw >= CRSF_CHANNEL_MAX_RAW)
  {
    return 2000U;
  }

  return (uint16_t)(1000U + (((raw - CRSF_CHANNEL_MIN_RAW) * 1000U) /
                             (CRSF_CHANNEL_MAX_RAW - CRSF_CHANNEL_MIN_RAW)));
}

uint16_t CRSF_GetFrameCount(void)
{
  return s_frame_count;
}

uint16_t CRSF_GetErrorCount(void)
{
  return s_error_count;
}

uint8_t CRSF_GetInitOk(void)
{
  return s_init_ok;
}

uint8_t CRSF_GetInitStatus(void)
{
  return s_init_status;
}

uint8_t CRSF_GetUartRxState(void)
{
  return s_uart_rx_state;
}

uint32_t CRSF_GetUartError(void)
{
  return s_uart_error;
}

uint8_t CRSF_GetDmaState(void)
{
  return s_dma_state;
}

uint32_t CRSF_GetDmaError(void)
{
  return s_dma_error;
}

uint32_t CRSF_GetByteCount(void)
{
  return s_byte_count;
}

uint16_t CRSF_GetDmaPos(void)
{
  return s_dma_pos_debug;
}

uint8_t CRSF_GetLastByte(void)
{
  return s_last_byte;
}

static uint8_t CRSF_IsAddress(uint8_t byte)
{
  return ((byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) ||
          (byte == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
          (byte == CRSF_ADDRESS_RADIO_TRANSMITTER) ||
          (byte == CRSF_ADDRESS_CRSF_RECEIVER)) ? 1U : 0U;
}

static uint8_t CRSF_CalcCrc(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;

  for (uint8_t i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1U) ^ 0xD5U);
      }
      else
      {
        crc = (uint8_t)(crc << 1U);
      }
    }
  }

  return crc;
}

static void CRSF_ParseByte(uint8_t byte)
{
  switch (s_parse_state)
  {
    case CRSF_PARSE_ADDR:
      if (CRSF_IsAddress(byte) != 0U)
      {
        s_frame_buf[0] = byte;
        s_frame_index = 1U;
        s_parse_state = CRSF_PARSE_LEN;
      }
      break;

    case CRSF_PARSE_LEN:
      if ((byte < 2U) || (byte > (CRSF_MAX_FRAME_LEN - 2U)))
      {
        s_error_count++;
        s_parse_state = CRSF_PARSE_ADDR;
        break;
      }

      s_frame_remaining = byte;
      s_frame_buf[s_frame_index++] = byte;
      s_parse_state = CRSF_PARSE_DATA;
      break;

    case CRSF_PARSE_DATA:
      s_frame_buf[s_frame_index++] = byte;
      s_frame_remaining--;
      if (s_frame_remaining == 0U)
      {
        CRSF_ProcessFrame(s_frame_buf, s_frame_index);
        s_parse_state = CRSF_PARSE_ADDR;
        s_frame_index = 0U;
      }
      break;

    default:
      s_parse_state = CRSF_PARSE_ADDR;
      s_frame_index = 0U;
      break;
  }
}

static void CRSF_ProcessFrame(const uint8_t *frame, uint8_t total_len)
{
  uint8_t len = frame[1];
  uint8_t type = frame[2];
  uint8_t rx_crc = frame[total_len - 1U];
  uint8_t calc_crc = CRSF_CalcCrc(&frame[2], (uint8_t)(len - 1U));

  if (calc_crc != rx_crc)
  {
    s_error_count++;
    return;
  }

  if ((type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) && ((len - 2U) == CRSF_RC_PAYLOAD_LEN))
  {
    CRSF_UnpackChannels(&frame[3]);
    s_last_frame_tick = HAL_GetTick();
    s_frame_count++;
  }
}

static void CRSF_UnpackChannels(const uint8_t *payload)
{
  for (uint8_t ch = 0U; ch < CRSF_CHANNEL_COUNT; ch++)
  {
    uint16_t bit_offset = (uint16_t)ch * 11U;
    uint8_t byte_index = (uint8_t)(bit_offset >> 3U);
    uint8_t bit_shift = (uint8_t)(bit_offset & 0x07U);
    uint32_t value = payload[byte_index];

    if ((byte_index + 1U) < CRSF_RC_PAYLOAD_LEN)
    {
      value |= ((uint32_t)payload[byte_index + 1U]) << 8U;
    }

    if ((byte_index + 2U) < CRSF_RC_PAYLOAD_LEN)
    {
      value |= ((uint32_t)payload[byte_index + 2U]) << 16U;
    }

    s_channels_raw[ch] = (uint16_t)((value >> bit_shift) & 0x07FFU);
  }
}
