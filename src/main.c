#include "app_config.h"
#include "BMI088driver.h"
#include "board.h"
#include "lcd_display.h"
#include "motor_output.h"
#include "route_controller.h"
#include "run_log.h"

typedef struct {
    uint8_t is_red;
    float strafe_sign;
    float turn_sign;
} route_field_profile_t;

static route_field_profile_t route_field_profile(board_field_t field)
{
    route_field_profile_t profile;

    if(field==BOARD_FIELD_RED){
        profile.is_red=1U;
    }
    else{
        profile.is_red=0U;
    }


     if (profile.is_red != 0U)
    {
        profile.strafe_sign = -ROUTE_RIGHT_STRAFE_SIGN;
    }
    else
    {
        profile.strafe_sign = ROUTE_RIGHT_STRAFE_SIGN;
    }
    if (profile.is_red != 0U)
    {
        profile.turn_sign = ROUTE_LEFT_TURN_SIGN;
    }
    else
    {
        profile.turn_sign = ROUTE_RIGHT_TURN_SIGN;
    }
    return profile;
}

#define enter_fault(code)                                      \
    do {                                                       \
        if ((code) == FAULT_RC_OVERRIDE) {                    \
            board_uart1_write("H7,RC,ROUTE_ABORTED_BY_RC\r\n"); \
            goto route_start;                                 \
        }                                                      \
        route_controller_enter_fault_wait_restart(code);       \
        goto route_start;                                      \
    } while (0)

int main(void)
{
    board_field_t selected_field;
    route_field_profile_t field_profile;

    route_controller_init();

route_start:
    route_controller_wait_for_start();
    selected_field = board_selected_field();
    if (selected_field == BOARD_FIELD_UNKNOWN) {
        board_uart1_write("H7,FAULT,NO_FIELD_SELECTED\r\n");
        route_controller_enter_fault_wait_restart(FAULT_KINEMATICS);
        goto route_start;
    }
    field_profile = route_field_profile(selected_field);
    route_controller_set_field(field_profile.is_red);
    route_controller_reset_run_context();

    if (field_profile.is_red != 0U) {
        board_uart1_write("H7,ROUTE,FIELD=RED\r\n");
        board_usb_write("FIELD,RED\r\n");
    } else {
        board_uart1_write("H7,ROUTE,FIELD=BLUE\r\n");
        board_usb_write("FIELD,BLUE\r\n");
    }

#if ROUTE_AUTO_RUN_ON_BOOT == 0U
    route_controller_wait_for_usb_run_command();
#endif

#if ROUTE_WAIT_RK_READY_ON_BOOT
    while (route_controller_rk_link_ready() == 0U) {
        route_controller_wait_for_rk_ready_on_boot();
    }
#endif

    route_controller_begin_pretask_sync();
    if (!route_controller_wait_for_rk_reset_before_route()) {
        enter_fault(FAULT_ARM_TIMEOUT);
    }

    g_run_state = RUN_BOOT;
    if (!route_controller_wait_for_can_startup()) {
        if (g_fault_code == FAULT_RC_OVERRIDE) {
            enter_fault(FAULT_RC_OVERRIDE);
        }
#if ROUTE_REQUIRE_CAN_STARTUP
        enter_fault(FAULT_CAN_STARTUP);
#else
        board_uart1_write("H7,WARN,CAN_STARTUP_BYPASS\r\n");
        g_fault_code = FAULT_NONE;
#endif
    }

    g_run_state = RUN_IMU_INIT;
    g_bmi088_init_error = BMI088_init();
    if (g_bmi088_init_error != BMI088_NO_ERROR) {
        enter_fault(FAULT_IMU_INIT);
    }

    g_run_state = RUN_GYRO_CALIBRATION;
    if (!route_controller_calibrate_gyro()) {
        enter_fault(g_fault_code == FAULT_RC_OVERRIDE ? FAULT_RC_OVERRIDE :
                    FAULT_IMU_MOVING);
    }

    g_run_state = RUN_MOTOR_ENABLE;
    if (!route_controller_enable_motors()) {
#if ROUTE_REQUIRE_MOTOR_ENABLE
        enter_fault(FAULT_MOTOR_ENABLE);
#else
        board_uart1_write("H7,WARN,MOTOR_ENABLE_BYPASS\r\n");
        g_fault_code = FAULT_NONE;
#endif
    }
    route_controller_hold_zero(200U);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

#if ROUTE_REQUIRE_MOTOR_FEEDBACK
    {
        const uint32_t feedback_wait_started_ms = HAL_GetTick();
        float primed_wheels[4] = {0.0f};

        while (!motor_feedback_update(HAL_GetTick(), primed_wheels)) {
            if ((uint32_t)(HAL_GetTick() - feedback_wait_started_ms) > 1000U) {
                enter_fault(FAULT_MOTOR_COMMAND);
            }
            route_controller_hold_zero(CONTROL_PERIOD_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
        }
    }
#else
    {
        const uint32_t feedback_wait_started_ms = HAL_GetTick();
        float primed_wheels[4] = {0.0f};

        while (!motor_feedback_update(HAL_GetTick(), primed_wheels)) {
            if ((uint32_t)(HAL_GetTick() - feedback_wait_started_ms) > 200U) {
                board_uart1_write("H7,WARN,MOTOR_FEEDBACK_BYPASS\r\n");
                break;
            }
            route_controller_hold_zero(CONTROL_PERIOD_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
        }
    }
#endif

    route_controller_reset_pose();

#if ROUTE_TASK1_ONLY
    /* The arm must be physically at the high observation pose before the
     * chassis starts the entry arc. White-line alignment starts only after
     * the arc reaches the station. */
    if (!route_controller_wait_for_disc_prep_high()) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
    }
#endif

#if ROUTE_DISC_ARC_ENTRY_ENABLED
    /*
     * Replace the old strafe -> forward -> in-place turn sequence with one
     * continuous cubic entry.  The field profile mirrors both the side
     * of the obstacle and the final station heading.
     */
    g_run_state = RUN_DISC_ARC_ENTRY;
    if (!route_controller_run_disc_arc_entry(field_profile.strafe_sign,
                                             field_profile.turn_sign)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
#else
    g_run_state = field_profile.is_red != 0U ? RUN_STRAFE_LEFT :
                                               RUN_STRAFE_RIGHT;
    if (!route_controller_run_translation_profile(
            0.0f, field_profile.strafe_sign, ROUTE_STRAFE_DISTANCE_M,
            ROUTE_INITIAL_STRAFE_SPEED_M_S,
            ROUTE_INITIAL_STRAFE_ACCEL_M_S2)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    if (!route_controller_run_relative_turn(0.0f)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_FORWARD;
    if (!route_controller_run_translation_profile(
            ROUTE_FORWARD_SIGN, 0.0f, ROUTE_FORWARD_DISTANCE_M,
            ROUTE_LONG_FORWARD_SPEED_M_S, ROUTE_LONG_FORWARD_ACCEL_M_S2)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    if (!route_controller_run_relative_turn(0.0f)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = field_profile.is_red != 0U ? RUN_TURN_LEFT : RUN_TURN_RIGHT;
    if (!route_controller_run_relative_turn(
            field_profile.turn_sign * ROUTE_TURN_ANGLE_RAD *
            ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
#endif

    /* Stop background sync from consuming WHITE_LINE replies, then use the
     * field strip to remove the remaining arc endpoint and yaw error. */
    route_controller_mark_first_arm_station();
#if ROUTE_DISC_VISUAL_ALIGN_ENABLED
    g_run_state = RUN_DISC_VISUAL_ALIGN;
    if (!route_controller_run_disc_visual_alignment()) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
#endif

    /* Once the white line reaches the reference row, continue at 0.08 m/s for
     * one second while holding the calibrated heading and PREP_HIGH pose.
     * DISC_CATCH START is sent only after
     * this move stops, so RK lowers the arm and enables ball recognition in
     * the final station pose. */
    g_run_state = RUN_DISC_FINAL_APPROACH;
    if (!route_controller_run_timed_forward(
            ROUTE_DISC_FINAL_APPROACH_SPEED_M_S,
            ROUTE_DISC_FINAL_APPROACH_DURATION_MS)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

#if ROUTE_TASK1_DISC_CATCH_ENABLED
    g_run_state = RUN_ARM_DISC_CATCH;
    route_controller_log_event(RUN_LOG_EVENT_ARM_START);
    if (!route_controller_wait_for_rk_arm_task(ROUTE_TASK1_RK_ARM_TASK)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
    }
    route_controller_log_event(route_controller_last_arm_task_bypassed() != 0U
                                   ? RUN_LOG_EVENT_ARM_BYPASS
                                   : RUN_LOG_EVENT_ARM_DONE);
#endif

#if ROUTE_TASK1_ONLY
    /* DISC_CATCH owns retraction and sends DONE only after the arm is home. */
    board_uart1_write("H7,ROUTE,TASK1_ONLY_COMPLETE\r\n");
    goto route_task1_only_shutdown;
#endif

#if !ROUTE_TASK1_ONLY
    g_run_state = RUN_REVERSE_AFTER_DISC;
    if (!route_controller_run_translation(-ROUTE_FORWARD_SIGN, 0.0f,
                                          ROUTE_AFTER_DISC_REVERSE_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = field_profile.is_red != 0U ? RUN_SECOND_TURN_LEFT :
                                               RUN_SECOND_TURN_RIGHT;
    if (!route_controller_run_relative_turn(
            field_profile.turn_sign * ROUTE_TURN_ANGLE_RAD *
            ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_PLATFORM_APPROACH;
    if (!route_controller_run_translation(ROUTE_FORWARD_SIGN, 0.0f,
                                          ROUTE_PLATFORM_APPROACH_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = field_profile.is_red != 0U ? RUN_THIRD_TURN_LEFT :
                                               RUN_THIRD_TURN_RIGHT;
    if (!route_controller_run_relative_turn(
            field_profile.turn_sign * ROUTE_TURN_ANGLE_RAD *
            ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

#if ROUTE_TASK2_PLATFORM_PICK_ENABLED
    {
        uint32_t platform_index;

        for (platform_index = 0U;
             platform_index < ROUTE_TASK2_PLATFORM_PICK_COUNT;
             ++platform_index) {
            g_run_state = RUN_ARM_PLATFORM_PICK;
            route_controller_log_event(RUN_LOG_EVENT_ARM_START);
            if (!route_controller_wait_for_rk_arm_task(ROUTE_TASK2_RK_ARM_TASK)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
            }
            route_controller_log_event(route_controller_last_arm_task_bypassed() != 0U
                                           ? RUN_LOG_EVENT_ARM_BYPASS
                                           : RUN_LOG_EVENT_ARM_DONE);
            if (platform_index + 1U < ROUTE_TASK2_PLATFORM_PICK_COUNT) {
                const float shift_distance_m =
                    platform_index == 0U
                        ? ROUTE_TASK2_PLATFORM_FIRST_SHIFT_M
                        : ROUTE_TASK2_PLATFORM_SECOND_SHIFT_M;

                route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
                if (g_run_state == RUN_FAULT) {
                    enter_fault(g_fault_code);
                }
                g_run_state = field_profile.is_red != 0U
                                   ? RUN_PLATFORM_SHIFT_RIGHT
                                   : RUN_PLATFORM_SHIFT_LEFT;
                if (!route_controller_run_translation(
                        0.0f, -field_profile.strafe_sign, shift_distance_m)) {
                    enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
                }
                route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
                if (g_run_state == RUN_FAULT) {
                    enter_fault(g_fault_code);
                }
            }
        }
    }
#endif

    g_run_state = RUN_DIAGONAL_AFTER_PLATFORM;
    if (!route_controller_run_translation(
            -ROUTE_FORWARD_SIGN * ROUTE_AFTER_PLATFORM_REVERSE_COMPONENT_M,
            -field_profile.strafe_sign * ROUTE_AFTER_PLATFORM_LEFT_COMPONENT_M,
            ROUTE_AFTER_PLATFORM_DIAGONAL_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = field_profile.is_red != 0U ? RUN_LAST_TURN_LEFT :
                                               RUN_LAST_TURN_RIGHT;
    if (!route_controller_run_relative_turn(
            field_profile.turn_sign * ROUTE_TURN_ANGLE_RAD *
            ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    {
        bool orbit_arm_started = false;

#if ROUTE_TASK3_COLUMN_CATCH_ENABLED
        g_run_state = RUN_ARM_COLUMN_CATCH;
        route_controller_log_event(RUN_LOG_EVENT_ARM_START);
        orbit_arm_started = route_controller_start_rk_arm_task(
            ROUTE_TASK3_RK_ARM_TASK);
        if (!orbit_arm_started && g_fault_code != FAULT_NONE) {
            enter_fault(g_fault_code);
        }
        route_controller_log_event(orbit_arm_started
                                       ? RUN_LOG_EVENT_ARM_ACK
                                       : RUN_LOG_EVENT_ARM_BYPASS);
#endif

        g_run_state = RUN_FRONT_CENTER_ORBIT;
        if (!route_controller_run_front_center_orbit(
                field_profile.turn_sign * ROUTE_FRONT_CENTER_ORBIT_ANGLE_RAD *
                    ROUTE_GYRO_TURN_SCALE,
                ROUTE_FRONT_CENTER_ORBIT_RADIUS_M)) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
        }
        route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
        if (g_run_state == RUN_FAULT) {
            enter_fault(g_fault_code);
        }

        g_run_state = RUN_FINAL_REVERSE;
        if (!route_controller_run_translation(-ROUTE_FORWARD_SIGN, 0.0f,
                                              ROUTE_FINAL_REVERSE_DISTANCE_M)) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
        }
        route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
        if (g_run_state == RUN_FAULT) {
            enter_fault(g_fault_code);
        }

#if ROUTE_TASK3_COLUMN_CATCH_ENABLED
        if (orbit_arm_started) {
            g_run_state = RUN_ARM_COLUMN_CATCH;
            route_controller_log_event(RUN_LOG_EVENT_ARM_STOP);
            if (!route_controller_stop_rk_arm_task(ROUTE_TASK3_RK_ARM_TASK)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
            }
            route_controller_log_event(RUN_LOG_EVENT_ARM_STOP_DONE);
        }
#else
        (void)orbit_arm_started;
#endif
    }
#endif

route_task1_only_shutdown:
#if !ROUTE_TASK1_ONLY
    g_run_state = RUN_SERVO_90;
    board_servo_set_angle_deg_index(0U, SERVO_MG90S_ROUTE_ANGLE_DEG);
    board_servo_set_angle_deg_index(1U, SERVO_MG90S_ROUTE_ANGLE_DEG);
    route_controller_hold_zero(ROUTE_SERVO_OPEN_HOLD_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
    board_servo_set_angle_deg_index(0U, ROUTE_SERVO_INITIAL_ANGLE_DEG);
    board_servo_set_angle_deg_index(1U, ROUTE_SERVO_INITIAL_ANGLE_DEG);
    route_controller_hold_zero(ROUTE_SERVO_RETURN_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
    board_servo_disable_index(0U);
    board_servo_disable_index(1U);
#endif

    g_run_state = RUN_STOPPING;
    route_controller_hold_zero(1000U);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
    if (!motor_disable_all() ||
        !board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS)) {
        enter_fault(FAULT_MOTOR_COMMAND);
    }
    g_run_state = RUN_DONE;
    {
        static const float stopped[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        run_log_sample(HAL_GetTick(), (uint32_t)g_run_state, g_fault_code,
                       g_gyro_z_rad_s, g_yaw_rad, 0.0f, 0.0f,
                       g_estimated_distance_m, g_imu_temperature_c, stopped,
                       g_cross_track_m, g_cross_track_command_m_s,
                       g_actual_cross_speed_m_s, RUN_LOG_EVENT_ROUTE_DONE);
    }
    (void)run_log_save((uint32_t)g_run_state, g_fault_code);
    run_log_dump_stored();

    board_uart1_write("H7,ROUTE,DONE,WAIT_NEXT_START\r\n");
    goto route_start;
}
