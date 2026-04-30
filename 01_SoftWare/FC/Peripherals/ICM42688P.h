/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ICM42688P.h
  * @brief   Minimal ICM42688P register driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ICM42688P_H__
#define __ICM42688P_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define ICM42688P_WHO_AM_I_VALUE  0x47U

/* 初始化 ICM42688P，并返回 WHO_AM_I 读取值。 */
uint8_t ICM42688P_Init(void);

/* 读取 WHO_AM_I，正常值应为 0x47。 */
uint8_t ICM42688P_ReadWhoAmI(void);

/* 读取三轴加速度原始寄存器值。 */
void ICM42688P_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az);

/* 读取三轴陀螺仪原始寄存器值。 */
void ICM42688P_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688P_H__ */
