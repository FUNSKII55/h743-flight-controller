#ifndef __CRSF_H__
#define __CRSF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define CRSF_CHANNEL_COUNT 16U

uint8_t CRSF_Init(void);
void CRSF_Update(void);
uint8_t CRSF_IsConnected(void);
uint16_t CRSF_GetChannelRaw(uint8_t channel);
uint16_t CRSF_GetChannelUs(uint8_t channel);
uint16_t CRSF_GetFrameCount(void);
uint16_t CRSF_GetErrorCount(void);
uint8_t CRSF_GetInitOk(void);
uint8_t CRSF_GetInitStatus(void);
uint8_t CRSF_GetUartRxState(void);
uint32_t CRSF_GetUartError(void);
uint8_t CRSF_GetDmaState(void);
uint32_t CRSF_GetDmaError(void);
uint32_t CRSF_GetByteCount(void);
uint16_t CRSF_GetDmaPos(void);
uint8_t CRSF_GetLastByte(void);

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H__ */
