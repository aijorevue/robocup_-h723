#ifndef TEST1_RUN_LOG_H
#define TEST1_RUN_LOG_H

#include <stdbool.h>
#include <stdint.h>

#define RUN_LOG_SAMPLE_PERIOD_MS 250U

enum {
    RUN_LOG_EVENT_SAMPLE = 0U,
    RUN_LOG_EVENT_CROSS_SETTLE_START = 1U,
    RUN_LOG_EVENT_CROSS_SETTLE_DONE = 2U,
    RUN_LOG_EVENT_CROSS_SETTLE_TIMEOUT = 3U,
    RUN_LOG_EVENT_ARM_START = 10U,
    RUN_LOG_EVENT_ARM_DONE = 11U,
    RUN_LOG_EVENT_ARM_BYPASS = 12U,
    RUN_LOG_EVENT_ARM_STOP = 13U,
    RUN_LOG_EVENT_ARM_STOP_DONE = 14U,
    RUN_LOG_EVENT_FAULT = 20U,
    RUN_LOG_EVENT_ROUTE_DONE = 21U
};

void run_log_reset(void);
void run_log_sample(uint32_t timestamp_ms, uint32_t state, uint32_t fault,
                    float gyro_z_rad_s, float yaw_rad, float command_speed_m_s,
                    float heading_correction_rad_s, float distance_m,
                    float temperature_c, const float wheel_rad_s[4],
                    float cross_track_m, float cross_track_command_m_s,
                    float actual_cross_speed_m_s, uint32_t event);
bool run_log_save(uint32_t final_state, uint32_t fault);
void run_log_dump_stored(void);

#endif
