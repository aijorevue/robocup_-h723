#include "rc_control.h"

static float absf(float value)
{
    return value < 0.0f ? -value : value;
}

static float axis_value(uint16_t channel, const rc_control_config_t *config)
{
    float value;
    float magnitude;
    const float delta = (float)((int32_t)channel - (int32_t)config->channel_center);
    const float half_span = (float)(config->channel_max - config->channel_center);

    if (absf(delta) <= (float)config->deadzone || half_span <= (float)config->deadzone) {
        return 0.0f;
    }
    magnitude = (absf(delta) - (float)config->deadzone) /
                (half_span - (float)config->deadzone);
    if (magnitude > 1.0f) {
        magnitude = 1.0f;
    }
    value = delta < 0.0f ? -magnitude : magnitude;
    return (1.0f - config->expo) * value + config->expo * value * value * value;
}

static float speed_scale(uint16_t channel, const rc_control_config_t *config)
{
    if (channel <= config->speed_low_max) {
        return config->low_speed_scale;
    }
    if (channel >= config->speed_high_min) {
        return config->high_speed_scale;
    }
    return config->mid_speed_scale;
}

static bool configured_channels_valid(const rc_frame_t *frame,
                                      const rc_control_config_t *config)
{
    const uint8_t channels[5] = {
        config->vx_channel, config->vy_channel, config->wz_channel,
        config->unlock_channel, config->speed_channel
    };
    uint8_t i;

    for (i = 0U; i < 5U; ++i) {
        const uint8_t index = channels[i];
        if (index >= frame->channel_count || frame->channels[index] < config->channel_min ||
            frame->channels[index] > config->channel_max) {
            return false;
        }
    }
    return true;
}

void rc_control_init(rc_control_state_t *state)
{
    if (state != 0) {
        state->saw_unlock_low = false;
        state->armed = false;
    }
}

void rc_control_force_safe(rc_control_state_t *state)
{
    if (state != 0) {
        state->saw_unlock_low = false;
        state->armed = false;
    }
}

chassis_command_t rc_control_update(rc_control_state_t *state,
                                    const rc_control_config_t *config,
                                    const rc_frame_t *latest_frame,
                                    uint32_t valid_frame_count,
                                    uint32_t last_valid_ms,
                                    uint32_t now_ms)
{
    chassis_command_t command = {0};
    uint16_t unlock;
    float gear;

    if (state == 0 || config == 0 || latest_frame == 0 || valid_frame_count == 0U ||
        (uint32_t)(now_ms - last_valid_ms) >= config->timeout_ms) {
        rc_control_force_safe(state);
        return command;
    }
    if (!configured_channels_valid(latest_frame, config)) {
        rc_control_force_safe(state);
        return command;
    }

    unlock = latest_frame->channels[config->unlock_channel];
    if (unlock <= config->unlock_low_max) {
        state->saw_unlock_low = true;
        state->armed = false;
        return command;
    }
    if (unlock < config->unlock_high_min) {
        state->armed = false;
        return command;
    }
    if (!state->saw_unlock_low && !config->allow_high_on_boot) {
        state->armed = false;
        return command;
    }
    if (!state->armed) {
        state->armed = true;
    }

    gear = speed_scale(latest_frame->channels[config->speed_channel], config);
    command.vx_m_s = axis_value(latest_frame->channels[config->vx_channel], config) *
                     config->vx_direction * config->max_linear_m_s * gear;
    command.vy_m_s = axis_value(latest_frame->channels[config->vy_channel], config) *
                     config->vy_direction * config->max_linear_m_s * gear;
    command.wz_rad_s = axis_value(latest_frame->channels[config->wz_channel], config) *
                       config->wz_direction * config->max_angular_rad_s * gear;
    command.armed = true;
    return command;
}
