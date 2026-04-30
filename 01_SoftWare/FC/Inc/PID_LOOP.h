#ifndef __PID_LOOP_H__
#define __PID_LOOP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

void PID_LOOP_UpdateAttitude(int16_t pitch_cd, int16_t roll_cd, int16_t yaw_cd,
                             float gyro_x_dps, float gyro_y_dps, float gyro_z_dps);
void PID_LOOP_Run(void);
void PID_LOOP_GetMotorOutput(uint16_t *motor1, uint16_t *motor2, uint16_t *motor3, uint16_t *motor4);
uint8_t PID_LOOP_IsArmed(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_LOOP_H__ */
