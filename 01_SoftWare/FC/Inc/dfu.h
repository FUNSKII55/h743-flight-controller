#ifndef __DFU_H__
#define __DFU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

void DFU_CheckCommand(void);
void DFU_CheckBootRequest(void);
void DFU_EnterBootloader(void);

#ifdef __cplusplus
}
#endif

#endif /* __DFU_H__ */
