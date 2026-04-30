#ifndef __IST8310_H__
#define __IST8310_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define IST8310_WHO_AM_I_VALUE  0x10U

/* 初始化 IST8310 磁力计，并返回 WHO_AM_I。 */
uint8_t IST8310_Init(void);

/* 读取 IST8310 WHO_AM_I，正常值一般为 0x10。 */
uint8_t IST8310_ReadWhoAmI(void);

/* 单次测量并读取三轴磁力计原始值。 */
uint8_t IST8310_ReadMagRaw(int16_t *mx, int16_t *my, int16_t *mz);

#ifdef __cplusplus
}
#endif

#endif /* __IST8310_H__ */
