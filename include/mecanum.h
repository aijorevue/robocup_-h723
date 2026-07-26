#ifndef MECANUM_H
#define MECANUM_H

#include <stdbool.h>

enum {
    MECANUM_FRONT_LEFT = 0,
    MECANUM_FRONT_RIGHT = 1,
    MECANUM_REAR_LEFT = 2,
    MECANUM_REAR_RIGHT = 3,
    MECANUM_WHEEL_COUNT = 4
};

typedef struct {
    float wheel_radius_m;
    float half_length_m;
    float half_width_m;
    float max_wheel_rad_s;
    float direction[MECANUM_WHEEL_COUNT];
} mecanum_config_t;

/* vx forward, vy left/right per configured sign, wz positive yaw per configured sign. */
bool mecanum_inverse(const mecanum_config_t *config, float vx_m_s,
                     float vy_m_s, float wz_rad_s,
                     float wheel_rad_s[MECANUM_WHEEL_COUNT]);
bool mecanum_forward(const mecanum_config_t *config,
                     const float wheel_rad_s[MECANUM_WHEEL_COUNT],
                     float *vx_m_s, float *vy_m_s, float *wz_rad_s);

#endif
