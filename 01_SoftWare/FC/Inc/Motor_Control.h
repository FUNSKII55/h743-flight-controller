#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Motor_Test_Run(void);
void Motor_Init(void);
uint8_t Motor_Write(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
