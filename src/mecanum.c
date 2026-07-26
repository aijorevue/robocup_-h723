#include "mecanum.h"

static float absf(float value)
{
    return value < 0.0f ? -value : value;
}

bool mecanum_inverse(const mecanum_config_t *config, float vx_m_s,
                     float vy_m_s, float wz_rad_s,
                     float wheel_rad_s[MECANUM_WHEEL_COUNT])
{
    float max_abs = 0.0f;
    float scale = 1.0f;
    float lever;
    int i;

    if (config == 0 || wheel_rad_s == 0 || config->wheel_radius_m <= 0.0f ||
        config->max_wheel_rad_s <= 0.0f) {
        return false;
    }

    lever = config->half_length_m + config->half_width_m;
    wheel_rad_s[MECANUM_FRONT_LEFT] =
        (vx_m_s - vy_m_s - lever * wz_rad_s) / config->wheel_radius_m;
    wheel_rad_s[MECANUM_FRONT_RIGHT] =
        (vx_m_s + vy_m_s + lever * wz_rad_s) / config->wheel_radius_m;
    wheel_rad_s[MECANUM_REAR_LEFT] =
        (vx_m_s + vy_m_s - lever * wz_rad_s) / config->wheel_radius_m;
    wheel_rad_s[MECANUM_REAR_RIGHT] =
        (vx_m_s - vy_m_s + lever * wz_rad_s) / config->wheel_radius_m;

    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        const float magnitude = absf(wheel_rad_s[i]);
        if (magnitude > max_abs) {
            max_abs = magnitude;
        }
    }
    if (max_abs > config->max_wheel_rad_s) {
        scale = config->max_wheel_rad_s / max_abs;
    }
    for (i = 0; i < MECANUM_WHEEL_COUNT; ++i) {
        wheel_rad_s[i] *= scale * config->direction[i];
    }
    return true;
}

bool mecanum_forward(const mecanum_config_t *config,
                     const float wheel_rad_s[MECANUM_WHEEL_COUNT],
                     float *vx_m_s, float *vy_m_s, float *wz_rad_s)
{
    float lever;

    if (config == 0 || wheel_rad_s == 0 || vx_m_s == 0 || vy_m_s == 0 ||
        wz_rad_s == 0 || config->wheel_radius_m <= 0.0f) {
        return false;
    }

    lever = config->half_length_m + config->half_width_m;
    *vx_m_s = config->wheel_radius_m *
              (wheel_rad_s[MECANUM_FRONT_LEFT] + wheel_rad_s[MECANUM_FRONT_RIGHT] +
               wheel_rad_s[MECANUM_REAR_LEFT] + wheel_rad_s[MECANUM_REAR_RIGHT]) * 0.25f;
    *vy_m_s = config->wheel_radius_m *
              (-wheel_rad_s[MECANUM_FRONT_LEFT] + wheel_rad_s[MECANUM_FRONT_RIGHT] +
               wheel_rad_s[MECANUM_REAR_LEFT] - wheel_rad_s[MECANUM_REAR_RIGHT]) * 0.25f;
    *wz_rad_s = config->wheel_radius_m *
                (-wheel_rad_s[MECANUM_FRONT_LEFT] + wheel_rad_s[MECANUM_FRONT_RIGHT] -
                 wheel_rad_s[MECANUM_REAR_LEFT] + wheel_rad_s[MECANUM_REAR_RIGHT]) /
                (4.0f * lever);
    return true;
}
