#include "rc_override.h"

#include "app_config.h"
#include "board.h"
#include "lcd_display.h"
#include "mecanum.h"
#include "motor_output.h"
#include "rc_control.h"
#include "route_controller.h"
#include "run_log.h"

#include "stm32h7xx_hal.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>

#define RC_MOTOR_DISABLE_RETRY_MS 1000U
#define RC_MOTOR_ENABLE_RETRY_MS 250U

static rc_control_state_t control_state;
static rc_parser_stats_t stats_debug;
static chassis_command_t command_debug;
static chassis_command_t last_drive_command;
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
static uint32_t last_input_event_log_ms;

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

_Static_assert(RC_OVERRIDE_SIGNAL_LOSS_RELEASE_MS >= RC_TIMEOUT_MS,
               "RC signal-loss release must not precede frame timeout");
_Static_assert(RC_OVERRIDE_RELEASE_CONFIRM_MS > 0U,
               "RC release confirmation must be non-zero");

static bool wait_for_all_tx(void)
{
    return board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS);
}

static void drain_motor_feedback(void)
{
    motor_feedback_drain(HAL_GetTick());
}

static bool send_safety_zero_checked(void)
{
    drain_motor_feedback();
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

    drain_motor_feedback();
    if (!motor_clear_errors_all() || !wait_for_all_tx()) {
        return false;
    }
    drain_motor_feedback();
    if (!motor_enable_all() || !wait_for_all_tx()) {
        return false;
    }
    drain_motor_feedback();
    if (!motor_send_zero_all() || !wait_for_all_tx()) {
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
    drain_motor_feedback();

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

static bool send_drive_command(const chassis_command_t *command)
{
    float wheel_speed[MECANUM_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

    return command != NULL &&
           mecanum_inverse(&rc_chassis, command->vx_m_s, command->vy_m_s,
                           command->wz_rad_s, wheel_speed) &&
           motor_send_wheel_speeds(wheel_speed);
}

static void log_rc_sample(uint32_t timestamp_ms,
                          const chassis_command_t *command,
                          const float measured_wheel_speed[4],
                          uint32_t event)
{
    float actual_vx = 0.0f;
    float actual_vy = 0.0f;
    float actual_wz = 0.0f;
    float command_speed;

    if (command == NULL || measured_wheel_speed == NULL) {
        return;
    }
    (void)mecanum_forward(&rc_chassis, measured_wheel_speed, &actual_vx,
                          &actual_vy, &actual_wz);
    (void)actual_vx;
    (void)actual_wz;
    command_speed = sqrtf(command->vx_m_s * command->vx_m_s +
                          command->vy_m_s * command->vy_m_s);
    run_log_sample(timestamp_ms, (uint32_t)RUN_RC_OVERRIDE, FAULT_NONE,
                   g_gyro_z_rad_s, g_yaw_rad, command_speed,
                   command->wz_rad_s, g_estimated_distance_m,
                   g_imu_temperature_c, measured_wheel_speed, g_cross_track_m,
                   command->vy_m_s, actual_vy, event);
}

static chassis_command_t update_command(uint32_t now_ms)
{
    rc_frame_t frame = {0};
    const bool have_frame = board_rc_snapshot(&frame, &stats_debug);

    latest_frame_debug = frame;
    have_frame_debug = have_frame;
    if (board_rc_take_unsafe_event()) {
        /* A parser/UART event invalidates the current sample, but it is not
         * itself a motor-stop request.  The valid-frame timeout below gives
         * the stream time to recover and the bounded loss policy handles a
         * genuinely disconnected receiver. */
        const uint32_t event_now_ms = HAL_GetTick();
        if ((uint32_t)(event_now_ms - last_input_event_log_ms) >= 1000U) {
            last_input_event_log_ms = event_now_ms;
            board_uart1_write_only("H7,RC,INPUT_EVENT,IGNORED_FOR_CONTINUITY\r\n");
        }
    }
    command_debug = rc_control_update(
        &control_state, &control_config, have_frame ? &frame : 0,
        stats_debug.valid_frame_count, stats_debug.last_valid_ms, now_ms);
    return command_debug;
}

static void log_active_status(void)
{
    const chassis_command_t *effective_command = &command_debug;
    const char *command_source = "LIVE";
    char line[384];

    if (!command_debug.armed && last_drive_command.armed) {
        effective_command = &last_drive_command;
        command_source = "HOLD";
    }

    if (have_frame_debug && latest_frame_debug.channel_count >= 6U) {
        const int32_t vx_mm_s = (int32_t)(effective_command->vx_m_s * 1000.0f);
        const int32_t vy_mm_s = (int32_t)(effective_command->vy_m_s * 1000.0f);
        const int32_t wz_mrad_s = (int32_t)(effective_command->wz_rad_s * 1000.0f);
        (void)snprintf(line, sizeof(line),
                       "H7,RC,ACTIVE,source=%s,ch1=%u,ch2=%u,ch3=%u,ch4=%u,ch5=%u,ch6=%u,vx_mm=%ld,vy_mm=%ld,wz_mrad=%ld,valid=%lu,lost=%lu,failsafe=%lu,range=%lu,uart_err=%lu,can_tx=%lu\r\n",
                       command_source,
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
                       (unsigned long)stats_debug.lost_frame_count,
                       (unsigned long)stats_debug.failsafe_frame_count,
                       (unsigned long)stats_debug.range_error_count,
                       (unsigned long)g_uart5_error_count,
                       (unsigned long)g_fdcan_tx_error_count);
        board_uart1_write(line);
    } else {
        (void)snprintf(line, sizeof(line),
                       "H7,RC,ACTIVE,source=%s,no_frame,valid=%lu,lost=%lu,failsafe=%lu,range=%lu,uart_err=%lu,can_tx=%lu\r\n",
                       command_source,
                       (unsigned long)stats_debug.valid_frame_count,
                       (unsigned long)stats_debug.lost_frame_count,
                       (unsigned long)stats_debug.failsafe_frame_count,
                       (unsigned long)stats_debug.range_error_count,
                       (unsigned long)g_uart5_error_count,
                       (unsigned long)g_fdcan_tx_error_count);
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
    last_drive_command = (chassis_command_t){0};
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
    last_input_event_log_ms = HAL_GetTick() - 1000U;
}

bool rc_override_is_running(void)
{
    return override_running;
}

bool rc_override_service(void)
{
#if ROUTE_RC_OVERRIDE_ENABLED
    float measured_wheel_speed[MECANUM_WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t last_control_ms;
    uint32_t released_since_ms = 0U;
    uint32_t signal_lost_since_ms = 0U;
    uint32_t last_status_ms;
    uint32_t last_log_ms;
    uint32_t last_command_error_log_ms;

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
    (void)run_log_save_event((uint32_t)RUN_RC_OVERRIDE, FAULT_NONE,
                             RUN_LOG_EVENT_RC_TAKEOVER);

    if (!motors_enabled) {
        last_enable_attempt_ms = HAL_GetTick();
        if (!enable_motors_for_arm()) {
            board_uart1_write("H7,RC,MOTOR_ENABLE_FAIL\r\n");
            (void)run_log_save_event((uint32_t)RUN_RC_OVERRIDE, FAULT_NONE,
                                     RUN_LOG_EVENT_RC_MOTOR_ENABLE_FAIL);
            enter_fault_safe(HAL_GetTick());
            motor_enable_pending = false;
        }
    }

    last_control_ms = HAL_GetTick();
    last_status_ms = last_control_ms - 1000U;
    last_log_ms = last_control_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    last_command_error_log_ms = last_control_ms - 1000U;
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

        /* RC control does not close a chassis feedback loop, but every motor
         * still replies on CAN.  Drain those frames so the 16-element RX FIFO
         * cannot overflow during a long manual takeover. */
        (void)motor_feedback_update(now_ms, measured_wheel_speed);

        command_debug = update_command(now_ms);
        if (!command_debug.armed) {
            const bool fresh_frame =
                have_frame_debug && stats_debug.valid_frame_count != 0U &&
                (uint32_t)(now_ms - stats_debug.last_valid_ms) < RC_TIMEOUT_MS;
            const bool release_requested =
                fresh_frame &&
                latest_frame_debug.channel_count > RC_CH_UNLOCK_INDEX &&
                latest_frame_debug.channels[RC_CH_UNLOCK_INDEX] <
                    RC_UNLOCK_HIGH_MIN_US;
            bool signal_loss_release = false;

            if (fresh_frame) {
                signal_lost_since_ms = 0U;
            } else if (signal_lost_since_ms == 0U) {
                signal_lost_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - signal_lost_since_ms) >=
                       RC_OVERRIDE_SIGNAL_LOSS_RELEASE_MS) {
                signal_loss_release = true;
            }

            if (release_requested || signal_loss_release) {
                if (released_since_ms == 0U) {
                    released_since_ms = now_ms;
                    if (motors_enabled && !send_safety_zero_checked()) {
                        board_uart1_write("H7,RC,RELEASE_ZERO_FAIL\r\n");
                    }
                } else if ((uint32_t)(now_ms - released_since_ms) >=
                           RC_OVERRIDE_RELEASE_CONFIRM_MS &&
                           (!motor_disable_pending ||
                            (uint32_t)(now_ms - last_disable_attempt_ms) >=
                                RC_MOTOR_DISABLE_RETRY_MS)) {
                    if (!zero_then_disable_once(now_ms)) {
                        board_uart1_write("H7,RC,RELEASE_RETRY\r\n");
                        continue;
                    }
                    rc_control_force_safe(&control_state);
#if RC_OVERRIDE_ALLOW_HIGH_ON_BOOT
                    control_state.saw_unlock_low = true;
#endif
                    command_debug = (chassis_command_t){0};
                    last_drive_command = (chassis_command_t){0};
                    override_running = false;
                    owns_motors = false;
                    motors_enabled = false;
                    motor_enable_pending = false;
                    motor_disable_pending = false;
                    (void)run_log_save_event(
                        (uint32_t)RUN_RC_OVERRIDE, FAULT_RC_OVERRIDE,
                        signal_loss_release ? RUN_LOG_EVENT_RC_SIGNAL_LOST
                                             : RUN_LOG_EVENT_RC_RELEASED);
                    (void)run_log_save_snapshot((uint32_t)RUN_RC_OVERRIDE,
                                                FAULT_RC_OVERRIDE);
                    board_uart1_write(signal_loss_release
                                          ? "H7,RC,RELEASED,SIGNAL_LOST,WAIT_USER_KEY\r\n"
                                          : "H7,RC,RELEASED,CH5_LOW,WAIT_USER_KEY\r\n");
                    return true;
                }
                continue;
            }

            if (motors_enabled && last_drive_command.armed &&
                !send_drive_command(&last_drive_command)) {
                board_uart1_write("H7,RC,MOTOR_COMMAND_FAIL\r\n");
                if ((uint32_t)(now_ms - last_command_error_log_ms) >=
                    1000U) {
                    last_command_error_log_ms = now_ms;
                    log_rc_sample(now_ms, &last_drive_command,
                                  measured_wheel_speed,
                                  RUN_LOG_EVENT_RC_MOTOR_COMMAND_FAIL);
                }
                enter_fault_safe(now_ms);
                last_enable_attempt_ms = now_ms - RC_MOTOR_ENABLE_RETRY_MS;
            }
            continue;
        }

        released_since_ms = 0U;
        signal_lost_since_ms = 0U;
        last_drive_command = command_debug;
        if (!motors_enabled) {
            if ((uint32_t)(now_ms - last_enable_attempt_ms) >=
                RC_MOTOR_ENABLE_RETRY_MS) {
                last_enable_attempt_ms = now_ms;
                if (!enable_motors_for_arm()) {
                    motor_enable_pending = false;
                    board_uart1_write("H7,RC,MOTOR_ENABLE_RETRY\r\n");
                    if ((uint32_t)(now_ms - last_command_error_log_ms) >=
                        1000U) {
                        last_command_error_log_ms = now_ms;
                        (void)run_log_save_event(
                            (uint32_t)RUN_RC_OVERRIDE, FAULT_NONE,
                            RUN_LOG_EVENT_RC_MOTOR_ENABLE_FAIL);
                    }
                }
            }
            continue;
        }
        if (!send_drive_command(&command_debug)) {
            board_uart1_write("H7,RC,MOTOR_COMMAND_FAIL\r\n");
            enter_fault_safe(now_ms);
            last_enable_attempt_ms = now_ms - RC_MOTOR_ENABLE_RETRY_MS;
            continue;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_rc_sample(now_ms, &command_debug, measured_wheel_speed,
                          RUN_LOG_EVENT_RC_SAMPLE);
        }
    }
#else
    return false;
#endif
}
