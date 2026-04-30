#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* 初始化姿态卡尔曼滤波状态。 */
void Kalman_Init(void);

/* 上电静止采样陀螺仪零偏，sample_count 是采样次数，sample_delay_ms 是每次采样间隔。 */
void Kalman_CalibrateGyroBias(uint16_t sample_count, uint32_t sample_delay_ms);

/* 把当前姿态作为零点，后续输出相对角度。 */
void Kalman_ResetReference(void);

/* 设置磁力计校准参数。 */
void Kalman_SetMagCalibration(float offset_x, float offset_y, float offset_z,
                              float scale_x, float scale_y, float scale_z);

/* 读取 IMU/磁力计数据，输出 pitch/roll/yaw，单位为角度 x100。 */
uint8_t Kalman_GetAttitude(int16_t *pitch_cdeg, int16_t *roll_cdeg, int16_t *yaw_cdeg);

/* 读取最近一次姿态解算使用的陀螺仪角速度，单位为 deg/s。 */
void Kalman_GetGyroDps(float *gx_dps, float *gy_dps, float *gz_dps);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_H__ */
