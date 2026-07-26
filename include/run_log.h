#ifndef TEST1_RUN_LOG_H
#define TEST1_RUN_LOG_H

#include <stdbool.h>
#include <stdint.h>

#define RUN_LOG_SAMPLE_PERIOD_MS 250U

void run_log_reset(void);
void run_log_sample(uint32_t timestamp_ms, uint32_t state, uint32_t fault,
                    float gyro_z_rad_s, float yaw_rad, float command_speed_m_s,
                    float heading_correction_rad_s, float distance_m,
                    float temperature_c, const float wheel_rad_s[4]);
bool run_log_save(uint32_t final_state, uint32_t fault);
void run_log_dump_stored(void);

#endif
