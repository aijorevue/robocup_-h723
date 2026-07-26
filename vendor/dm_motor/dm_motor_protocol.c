#include "dm_motor_protocol.h"
#include <string.h>

float dm_clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

uint16_t dm_float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    uint32_t max_int = (1u << bits) - 1u;
    if (span <= 0.0f || bits == 0 || bits > 16) return 0;
    x = dm_clampf(x, x_min, x_max);
    return (uint16_t)((x - x_min) * (float)max_int / span + 0.5f);
}

float dm_uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits) {
    uint32_t max_int = (1u << bits) - 1u;
    if (bits == 0 || bits > 16 || max_int == 0) return x_min;
    if (x > max_int) x = max_int;
    return ((float)x) * (x_max - x_min) / (float)max_int + x_min;
}

int dm_pack_special(uint8_t out[8], uint8_t tail) {
    if (!out) return DM_ERR_NULL;
    memset(out, 0xFF, 7); out[7] = tail; return 8;
}
int dm_pack_clear_error(uint8_t out[8]) { return dm_pack_special(out, 0xFB); }
int dm_pack_enable(uint8_t out[8]) { return dm_pack_special(out, 0xFC); }
int dm_pack_disable(uint8_t out[8]) { return dm_pack_special(out, 0xFD); }
int dm_pack_save_zero(uint8_t out[8]) { return dm_pack_special(out, 0xFE); }

int dm_pack_mit(const dm_motor_limits_t *lim, float p, float v, float kp, float kd, float t, uint8_t out[8]) {
    if (!lim || !out) return DM_ERR_NULL;
    uint16_t p_u  = dm_float_to_uint(p,  lim->p_min,  lim->p_max,  16);
    uint16_t v_u  = dm_float_to_uint(v,  lim->v_min,  lim->v_max,  12);
    uint16_t kp_u = dm_float_to_uint(kp, lim->kp_min, lim->kp_max, 12);
    uint16_t kd_u = dm_float_to_uint(kd, lim->kd_min, lim->kd_max, 12);
    uint16_t t_u  = dm_float_to_uint(t,  lim->t_min,  lim->t_max,  12);
    out[0] = (uint8_t)(p_u >> 8); out[1] = (uint8_t)p_u;
    out[2] = (uint8_t)(v_u >> 4);
    out[3] = (uint8_t)(((v_u & 0xF) << 4) | (kp_u >> 8));
    out[4] = (uint8_t)kp_u;
    out[5] = (uint8_t)(kd_u >> 4);
    out[6] = (uint8_t)(((kd_u & 0xF) << 4) | (t_u >> 8));
    out[7] = (uint8_t)t_u;
    return 8;
}

int dm_unpack_feedback(const dm_motor_limits_t *lim, const uint8_t data[8], dm_motor_feedback_t *fb) {
    if (!lim || !data || !fb) return DM_ERR_NULL;
    uint16_t p_u = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_u = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t t_u = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];
    fb->can_id_low4 = data[0] & 0x0F; fb->state_high4 = data[0] >> 4;
    fb->position_rad = dm_uint_to_float(p_u, lim->p_min, lim->p_max, 16);
    fb->velocity_rad_s = dm_uint_to_float(v_u, lim->v_min, lim->v_max, 12);
    fb->torque = dm_uint_to_float(t_u, lim->t_min, lim->t_max, 12);
    fb->mos_temp = data[6]; fb->rotor_temp = data[7]; return DM_OK;
}

static void put_f32_le(float x, uint8_t *p) { union { float f; uint8_t b[4]; } u; u.f = x; memcpy(p, u.b, 4); }
int dm_pack_pos_vel(float pos_rad, float vel_rad_s, uint8_t out[8]) { if (!out) return DM_ERR_NULL; put_f32_le(pos_rad, out); put_f32_le(vel_rad_s, out+4); return 8; }
int dm_pack_velocity(float vel_rad_s, uint8_t out[8]) { if (!out) return DM_ERR_NULL; memset(out, 0, 8); put_f32_le(vel_rad_s, out); return 8; }
int dm_pack_emit(float pos_rad, uint16_t spd100, uint16_t cur10000, uint8_t out[8]) { if (!out) return DM_ERR_NULL; put_f32_le(pos_rad, out); out[4]=spd100&0xFF; out[5]=spd100>>8; out[6]=cur10000&0xFF; out[7]=cur10000>>8; return 8; }
