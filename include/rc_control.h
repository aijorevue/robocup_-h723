#ifndef RC_CONTROL_H
#define RC_CONTROL_H

#include "rc_protocol.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t vx_channel;
    uint8_t vy_channel;
    uint8_t wz_channel;
    uint8_t unlock_channel;
    uint8_t speed_channel;
    float vx_direction;
    float vy_direction;
    float wz_direction;
    float max_linear_m_s;
    float max_angular_rad_s;
    float expo;
    float low_speed_scale;
    float mid_speed_scale;
    float high_speed_scale;
    uint16_t channel_min;
    uint16_t channel_center;
    uint16_t channel_max;
    uint16_t deadzone;
    uint16_t arm_center_window;
    uint16_t unlock_low_max;
    uint16_t unlock_high_min;
    uint16_t speed_low_max;
    uint16_t speed_high_min;
    uint32_t timeout_ms;
    bool allow_high_on_boot;
} rc_control_config_t;

typedef struct {
    bool saw_unlock_low;
    bool armed;
} rc_control_state_t;

typedef struct {
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
    bool armed;
} chassis_command_t;

void rc_control_init(rc_control_state_t *state);
void rc_control_force_safe(rc_control_state_t *state);
chassis_command_t rc_control_update(rc_control_state_t *state,
                                    const rc_control_config_t *config,
                                    const rc_frame_t *latest_frame,
                                    uint32_t valid_frame_count,
                                    uint32_t last_valid_ms,
                                    uint32_t now_ms);

#endif
