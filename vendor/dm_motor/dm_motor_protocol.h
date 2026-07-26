#ifndef DM_MOTOR_PROTOCOL_H
#define DM_MOTOR_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DM_CAN_MODE_MIT_OFFSET      0x000u
#define DM_CAN_MODE_POS_VEL_OFFSET  0x100u
#define DM_CAN_MODE_VEL_OFFSET      0x200u
#define DM_CAN_MODE_EMIT_OFFSET     0x300u

typedef enum { DM_OK = 0, DM_ERR_NULL = -1, DM_ERR_RANGE = -2, DM_ERR_LEN = -3 } dm_status_t;

typedef struct {
    float p_min, p_max;    /* rad，电机输出端或驱动定义角度，需按手册确认 */
    float v_min, v_max;    /* rad/s */
    float t_min, t_max;    /* Nm 或驱动定义扭矩，需确认 */
    float kp_min, kp_max;  /* 常见 0..500，需确认 */
    float kd_min, kd_max;  /* 常见 0..5，需确认 */
} dm_motor_limits_t;

typedef struct {
    uint8_t can_id_low4;
    uint8_t state_high4;
    float position_rad;
    float velocity_rad_s;
    float torque;
    uint8_t mos_temp;
    uint8_t rotor_temp;
} dm_motor_feedback_t;

float dm_clampf(float x, float lo, float hi);
uint16_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float dm_uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits);

int dm_pack_special(uint8_t out[8], uint8_t tail);
int dm_pack_clear_error(uint8_t out[8]);
int dm_pack_enable(uint8_t out[8]);
int dm_pack_disable(uint8_t out[8]);
int dm_pack_save_zero(uint8_t out[8]);

int dm_pack_mit(const dm_motor_limits_t *lim, float p_rad, float v_rad_s,
                float kp, float kd, float torque, uint8_t out[8]);
int dm_unpack_feedback(const dm_motor_limits_t *lim, const uint8_t data[8], dm_motor_feedback_t *fb);
int dm_pack_pos_vel(float pos_rad, float vel_rad_s, uint8_t out[8]);
int dm_pack_velocity(float vel_rad_s, uint8_t out[8]);
int dm_pack_emit(float pos_rad, uint16_t limit_speed_rad_s_x100,
                 uint16_t limit_current_ratio_x10000, uint8_t out[8]);

#ifdef __cplusplus
}
#endif
#endif
