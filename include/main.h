#ifndef TEST1_MAIN_H
#define TEST1_MAIN_H

#include "stm32h7xx_hal.h"

#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define PWM_5V_EN_Pin GPIO_PIN_15
#define PWM_5V_EN_GPIO_Port GPIOC
void Error_Handler(void);

#endif
