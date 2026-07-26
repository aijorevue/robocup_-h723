#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

bool motor_send_wheel_speeds(const float wheel_rad_s[4]);
bool motor_send_zero_all(void);
bool motor_clear_errors_all(void);
bool motor_enable_all(void);
bool motor_disable_all(void);
bool motor_feedback_update(uint32_t now_ms, float wheel_rad_s[4]);

#endif
