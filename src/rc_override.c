#include "rc_override.h"

#include "app_config.h"
#include "board.h"
#include "lcd_display.h"
#include "mecanum.h"
#include "motor_output.h"
#include "rc_control.h"
#include "route_controller.h"

#include "stm32h7xx_hal.h"

#include <stdint.h>
#include <stdio.h>

#define RC_MOTOR_DISABLE_RETRY_MS 1000U
#define RC_MOTOR_ENABLE_RETRY_MS 250U

static rc_control_state_t control_state;
static rc_parser_stats_t stats_debug;
static chassis_command_t command_debug;
static rc_frame_t latest_frame_debug;
static bool have_frame_debug;
static bool override_running;
static bool owns_motors;
static bool motors_enabled;
static bool motor_enable_pending;
static bool motor_disable_pending;
static uint32_t last_disable_attempt_ms;
static uint32_t last_idle_status_ms;
static uint32_t last_enable_attempt_ms;

static const rc_control_config_t control_config = {
    .vx_channel = RC_CH_VX_INDEX,
    .vy_channel = RC_CH_VY_INDEX,
    .wz_channel = RC_CH_WZ_INDEX,
    .unlock_channel = RC_CH_UNLOCK_INDEX,
    .speed_channel = RC_CH_SPEED_INDEX,
    .vx_direction = RC_VX_DIRECTION,
    .vy_direction = RC_VY_DIRECTION,
    .wz_direction = RC_WZ_DIRECTION,
    .max_linear_m_s = RC_OVERRIDE_MAX_LINEAR_M_S,
    .max_angular_rad_s = RC_OVERRIDE_MAX_ANGULAR_RAD_S,
    .expo = RC_AXIS_EXPO,
    .low_speed_scale = RC_SPEED_LOW_SCALE,
    .mid_speed_scale = RC_SPEED_MID_SCALE,
    .high_speed_scale = RC_SPEED_HIGH_SCALE,
    .channel_min = RC_CHANNEL_MIN_US,
    .channel_center = RC_CHANNEL_CENTER_US,
    .channel_max = RC_CHANNEL_MAX_US,
    .deadzone = RC_STICK_DEADZONE_US,
    .arm_center_window = RC_ARM_CENTER_WINDOW_US,
    .unlock_low_max = RC_UNLOCK_LOW_MAX_US,
    .unlock_high_min = RC_UNLOCK_HIGH_MIN_US,
    .speed_low_max = RC_SPEED_LOW_MAX_US,
    .speed_high_min = RC_SPEED_HIGH_MIN_US,
    .timeout_ms = RC_TIMEOUT_MS,
    .allow_high_on_boot = RC_OVERRIDE_ALLOW_HIGH_ON_BOOT != 0U
};

static const mecanum_config_t rc_chassis = {
    .wheel_radius_m = RC_OVERRIDE_WHEEL_RADIUS_M,
    .half_length_m = RC_OVERRIDE_HALF_LENGTH_M,
    .half_width_m = RC_OVERRIDE_HALF_WIDTH_M,
    .max_wheel_rad_s = MOTOR_MAX_WHEEL_RAD_S,
    .direction = {
        RC_OVERRIDE_WHEEL_FL_SIGN,
        RC_OVERRIDE_WHEEL_FR_SIGN,
        RC_OVERRIDE_WHEEL_RL_SIGN,
        RC_OVERRIDE_WHEEL_RR_SIGN
    }
};

static bool wait_for_all_tx(void)
{
    return board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS);
}

static bool send_safety_zero_checked(void)
{
    if (!motor_send_zero_all() || !wait_for_all_tx()) {
        rc_control_force_safe(&control_state);
        command_debug = (chassis_command_t){0};
        return false;
    }
    return true;
}

static bool enable_motors_for_arm(void)
{
    motor_enable_pending = true;
    motor_disable_pending = false;

    if (!motor_clear_errors_all() || !wait_for_all_tx() ||
        !motor_enable_all() || !wait_for_all_tx() ||
        !motor_send_zero_all() || !wait_for_all_tx()) {
        return false;
    }

    motor_enable_pending = false;
    motors_enabled = true;
    return true;
}

static bool zero_then_disable_once(uint32_t now_ms)
{
    bool zero_ok;
    bool disable_ok;

    last_disable_attempt_ms = now_ms;
    command_debug = (chassis_command_t){0};

    zero_ok = send_safety_zero_checked();
    disable_ok = motor_disable_all();
    if (disable_ok) {
        disable_ok = wait_for_all_tx();
    }

    motor_enable_pending = false;
    if (zero_ok && disable_ok) {
        motors_enabled = false;
        motor_disable_pending = false;
        return true;
    }

    motor_disable_pending = true;
    rc_control_force_safe(&control_state);
    return false;
}

static void enter_fault_safe(uint32_t now_ms)
{
    const bool need_disable =
        owns_motors &&
        (motors_enabled || motor_enable_pending || motor_disable_pending);

    rc_control_force_safe(&control_state);
    command_debug = (chassis_command_t){0};
    if (need_disable) {
        (void)zero_then_disable_once(now_ms);
    } else if (owns_motors) {
        (void)send_safety_zero_checked();
    }
#if RC_OVERRIDE_ALLOW_HIGH_ON_BOOT
    /* A completed RC takeover is a new arm cycle; do not require a
     * low-to-high edge if the operator leaves CH5 enabled. */
    control_state.saw_unlock_low = true;
#endif
}

static void maintain_safe_state(uint32_t now_ms)
{
    if (!owns_motors) {
        return;
    }
    if (motors_enabled || motor_enable_pending || motor_disable_pending) {
        if (!motor_disable_pending ||
            (uint32_t)(now_ms - last_disable_attempt_ms) >=
                RC_MOTOR_DISABLE_RETRY_MS) {
            (void)zero_then_disable_once(now_ms);
        } else {
            (void)send_safety_zero_checked();
        }
    } else {
        (void)send_safety_zero_checked();
    }
}

static chassis_command_t update_command(uint32_t now_ms)
{
    rc_frame_t frame = {0};
    const bool have_frame = board_rc_snapshot(&frame, &stats_debug);

    latest_frame_debug = frame;
    have_frame_debug = have_frame;
    if (board_rc_take_unsafe_event()) {
        /* Receiver faults may stop motors only after RC has taken ownership.
         * Autonomous route code owns them at every other time. */
        if (!owns_motors) {
            rc_control_force_safe(&control_state);
            command_debug = (chassis_command_t){0};
            return command_debug;
        }
        enter_fault_safe(now_ms);
        command_debug = (chassis_command_t){0};
        return command_debug;
    }
    command_debug = rc_control_update(
        &control_state, &control_config, have_frame ? &frame : 0,
        stats_debug.valid_frame_count, stats_debug.last_valid_ms, now_ms);
    return command_debug;
}

static void log_active_status(void)
{
    char line[176];

    if (have_frame_debug && latest_frame_debug.channel_count >= 6U) {
        const int32_t vx_mm_s = (int32_t)(command_debug.vx_m_s * 1000.0f);
        const int32_t vy_mm_s = (int32_t)(command_debug.vy_m_s * 1000.0f);
        const int32_t wz_mrad_s = (int32_t)(command_debug.wz_rad_s * 1000.0f);
        (void)snprintf(line, sizeof(line),
                       "H7,RC,ACTIVE,ch1=%u,ch2=%u,ch3=%u,ch4=%u,ch5=%u,ch6=%u,vx_mm=%ld,vy_mm=%ld,wz_mrad=%ld,valid=%lu,uart_err=%lu\r\n",
                       latest_frame_debug.channels[0],
                       latest_frame_debug.channels[1],
                       latest_frame_debug.channels[2],
                       latest_frame_debug.channels[3],
                       latest_frame_debug.channels[4],
                       latest_frame_debug.channels[5],
                       (long)vx_mm_s,
                       (long)vy_mm_s,
                       (long)wz_mrad_s,
                       (unsigned long)stats_debug.valid_frame_count,
                       (unsigned long)g_uart5_error_count);
        board_uart1_write(line);
    } else {
        (void)snprintf(line, sizeof(line),
                       "H7,RC,ACTIVE,no_frame,valid=%lu,uart_err=%lu\r\n",
                       (unsigned long)stats_debug.valid_frame_count,
                       (unsigned long)g_uart5_error_count);
        board_uart1_write(line);
    }
}

void rc_override_init(void)
{
    rc_control_init(&control_state);
#if RC_OVERRIDE_ALLOW_HIGH_ON_BOOT
    control_state.saw_unlock_low = true;
#endif
    stats_debug = (rc_parser_stats_t){0};
    command_debug = (chassis_command_t){0};
    latest_frame_debug = (rc_frame_t){0};
    have_frame_debug = false;
    override_running = false;
    owns_motors = false;
    motors_enabled = false;
    motor_enable_pending = false;
    motor_disable_pending = false;
    last_disable_attempt_ms = HAL_GetTick();
    last_enable_attempt_ms = HAL_GetTick() - RC_MOTOR_ENABLE_RETRY_MS;
    last_idle_status_ms = HAL_GetTick() - 1000U;
}

bool rc_override_is_running(void)
{
    return override_running;
}

bool rc_override_service(void)
{
#if ROUTE_RC_OVERRIDE_ENABLED
    uint32_t last_control_ms;
    uint32_t released_since_ms = 0U;
    uint32_t last_status_ms;

    command_debug = update_command(HAL_GetTick());
    if (!command_debug.armed) {
        const uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - last_idle_status_ms) >= 1000U) {
            char line[144];
            last_idle_status_ms = now_ms;
            if (have_frame_debug && latest_frame_debug.channel_count >= 6U) {
                (void)snprintf(line, sizeof(line),
                               "H7,RC,IDLE,ch5=%u,ch1=%u,ch2=%u,ch4=%u,valid=%lu,uart_err=%lu\r\n",
                               latest_frame_debug.channels[4],
                               latest_frame_debug.channels[0],
                               latest_frame_debug.channels[1],
                               latest_frame_debug.channels[3],
                               (unsigned long)stats_debug.valid_frame_count,
                               (unsigned long)g_uart5_error_count);
            } else {
                (void)snprintf(line, sizeof(line),
                               "H7,RC,IDLE,no_frame,valid=%lu,uart_err=%lu\r\n",
                               (unsigned long)stats_debug.valid_frame_count,
                               (unsigned long)g_uart5_error_count);
            }
            board_uart1_write(line);
        }
        return false;
    }

    override_running = true;
    owns_motors = true;
    route_controller_request_rk_reset();
    lcd_display_set_start_status("RC");
    board_uart1_write("H7,RC,TAKEOVER\r\n");

    if (!motors_enabled) {
        last_enable_attempt_ms = HAL_GetTick();
        if (!enable_motors_for_arm()) {
            board_uart1_write("H7,RC,MOTOR_ENABLE_FAIL\r\n");
            enter_fault_safe(HAL_GetTick());
            motor_enable_pending = false;
        }
    }

    last_control_ms = HAL_GetTick();
    last_status_ms = last_control_ms - 1000U;
    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        route_controller_service_rk_link();
        lcd_display_update();
        if ((uint32_t)(now_ms - last_status_ms) >= 1000U) {
            last_status_ms = now_ms;
            log_active_status();
        }
        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms += CONTROL_PERIOD_MS;
        if ((uint32_t)(now_ms - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
        }

        command_debug = update_command(now_ms);
        if (!command_debug.armed) {
            if (released_since_ms == 0U) {
                released_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - released_since_ms) >=
                       RC_OVERRIDE_RELEASE_CONFIRM_MS) {
                (void)zero_then_disable_once(now_ms);
                rc_control_force_safe(&control_state);
#if RC_OVERRIDE_ALLOW_HIGH_ON_BOOT
                control_state.saw_unlock_low = true;
#endif
                command_debug = (chassis_command_t){0};
                override_running = false;
                owns_motors = false;
                motors_enabled = false;
                motor_enable_pending = false;
                motor_disable_pending = false;
                board_uart1_write("H7,RC,RELEASED,WAIT_USER_KEY\r\n");
                return true;
            }
            maintain_safe_state(now_ms);
            continue;
        }

        released_since_ms = 0U;
        if (!motors_enabled) {
            if ((uint32_t)(now_ms - last_enable_attempt_ms) >=
                RC_MOTOR_ENABLE_RETRY_MS) {
                last_enable_attempt_ms = now_ms;
                if (!enable_motors_for_arm()) {
                    motor_enable_pending = false;
                    board_uart1_write("H7,RC,MOTOR_ENABLE_RETRY\r\n");
                }
            }
            continue;
        }
        {
            float wheel_speed[MECANUM_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
            if (!mecanum_inverse(&rc_chassis, command_debug.vx_m_s,
                                 command_debug.vy_m_s,
                                 command_debug.wz_rad_s, wheel_speed) ||
                !motor_send_wheel_speeds(wheel_speed)) {
                board_uart1_write("H7,RC,MOTOR_COMMAND_FAIL\r\n");
                enter_fault_safe(now_ms);
                last_enable_attempt_ms = now_ms - RC_MOTOR_ENABLE_RETRY_MS;
                continue;
            }
        }
    }
#else
    return false;
#endif
}
