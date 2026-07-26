#include "motor_output.h"

#include "app_config.h"
#include "board.h"
#include "dm_motor_protocol.h"
#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_SPECIAL_RETRY_COUNT 10U
#define MOTOR_SPECIAL_COMPAT_RETRY_COUNT 3U
#define MOTOR_SPECIAL_RETRY_GAP_MS 2U
#define MOTOR_FEEDBACK_STALE_TIMEOUT_MS 100U

static const dm_motor_limits_t feedback_limits = {
    .p_min = -12.5f,
    .p_max = 12.5f,
    .v_min = -MOTOR_MAX_WHEEL_RAD_S,
    .v_max = MOTOR_MAX_WHEEL_RAD_S,
    .t_min = -12.0f,
    .t_max = 12.0f,
    .kp_min = 0.0f,
    .kp_max = 500.0f,
    .kd_min = 0.0f,
    .kd_max = 5.0f
};

static const uint16_t motor_base_id[4] = {
    USER_VERIFY_MOTOR_FL_BASE_ID,
    USER_VERIFY_MOTOR_FR_BASE_ID,
    USER_VERIFY_MOTOR_RL_BASE_ID,
    USER_VERIFY_MOTOR_RR_BASE_ID
};

static const float motor_direction[4] = {
    WHEEL_FL_SIGN,
    WHEEL_FR_SIGN,
    WHEEL_RL_SIGN,
    WHEEL_RR_SIGN
};

static float latest_wheel_rad_s[4];
static uint32_t last_feedback_ms[4];
static bool feedback_seen[4];

_Static_assert(DM_CAN_MODE_VEL_OFFSET == DM_VELOCITY_COMMAND_ID_OFFSET,
               "DM velocity command ID offset mismatch");

static bool feedback_slot_valid(uint32_t now_ms, uint32_t slot)
{
    return feedback_seen[slot] &&
           (uint32_t)(now_ms - last_feedback_ms[slot]) <= MOTOR_FEEDBACK_STALE_TIMEOUT_MS;
}

static void feedback_from_frame(uint32_t now_ms, const board_can_rx_frame_t *frame)
{
    dm_motor_feedback_t feedback = {0};
    uint8_t motor_id;

    if (frame == NULL) {
        return;
    }
    if (dm_unpack_feedback(&feedback_limits, frame->data, &feedback) != DM_OK) {
        return;
    }
    motor_id = feedback.can_id_low4;
    if (motor_id < 1U || motor_id > 4U) {
        return;
    }
    latest_wheel_rad_s[motor_id - 1U] =
        feedback.velocity_rad_s * MOTOR_FEEDBACK_WHEEL_SPEED_SCALE *
        motor_direction[motor_id - 1U];
    last_feedback_ms[motor_id - 1U] = now_ms;
    feedback_seen[motor_id - 1U] = true;
}

static bool send_special_batch(int (*packer)(uint8_t out[8]), uint16_t id_offset)
{
    uint8_t payload[4][8];
    board_can_frame_t frames[4] = {0};
    int i;

    for (i = 0; i < 4; ++i) {
        frames[i].standard_id = (uint16_t)(motor_base_id[i] + id_offset);
        frames[i].data = payload[i];
        if (packer(payload[i]) != 8) {
            return false;
        }
    }
    return board_fdcan1_send_classic_std8_batch4(frames);
}

static bool send_special_to_all(int (*packer)(uint8_t out[8]))
{
    uint32_t round;

    /* Primary: base ID only (must succeed). Matches ROS2/new protocol. */
    for (round = 0U; round < MOTOR_SPECIAL_RETRY_COUNT; ++round) {
        if (!send_special_batch(packer, 0U)) {
            return false;
        }
        HAL_Delay(MOTOR_SPECIAL_RETRY_GAP_MS);
    }
    /* Secondary: base+0x200 best-effort for old SDK enable path. */
    for (round = 0U; round < MOTOR_SPECIAL_COMPAT_RETRY_COUNT; ++round) {
        (void)send_special_batch(packer, DM_CAN_MODE_VEL_OFFSET);
        HAL_Delay(MOTOR_SPECIAL_RETRY_GAP_MS);
    }
    return true;
}

bool motor_send_wheel_speeds(const float wheel_rad_s[4])
{
    uint8_t payload[4][8];
    board_can_frame_t frames[4] = {0};
    int i;

    if (wheel_rad_s == 0) {
        return false;
    }
    for (i = 0; i < 4; ++i) {
        frames[i].standard_id =
            (uint16_t)(motor_base_id[i] + DM_CAN_MODE_VEL_OFFSET);
        frames[i].data = payload[i];
        if (dm_pack_velocity(wheel_rad_s[i], payload[i]) != 8) {
            return false;
        }
    }
    return board_fdcan1_send_classic_std8_batch4(frames);
}

bool motor_send_zero_all(void)
{
    static const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (!board_fdcan1_abort_all_pending()) {
        return false;
    }
    return motor_send_wheel_speeds(zero);
}

bool motor_clear_errors_all(void)
{
    return send_special_to_all(dm_pack_clear_error);
}

bool motor_enable_all(void)
{
    return send_special_to_all(dm_pack_enable);
}

bool motor_disable_all(void)
{
    return send_special_to_all(dm_pack_disable);
}

bool motor_feedback_update(uint32_t now_ms, float wheel_rad_s[4])
{
    board_can_rx_frame_t frame = {0};
    uint32_t i;

    while (board_fdcan1_read_classic_std8(&frame)) {
        feedback_from_frame(now_ms, &frame);
    }
    if (wheel_rad_s != NULL) {
        for (i = 0U; i < 4U; ++i) {
            wheel_rad_s[i] = latest_wheel_rad_s[i];
        }
    }
    for (i = 0U; i < 4U; ++i) {
        if (!feedback_slot_valid(now_ms, i)) {
            return false;
        }
    }
    return true;
}
