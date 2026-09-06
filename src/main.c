#include "app_config.h"
#include "BMI088driver.h"
#include "board.h"
#include "lcd_display.h"
#include "motor_output.h"
#include "route_controller.h"
#include "run_log.h"

#include <stdio.h>

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

/* A route-stage fault is reported and the next stage is attempted.  The
 * exception is RC takeover: it is an explicit operator command, so autonomy
 * must yield immediately.  Unknown field selection still returns to the
 * start gate because there is no valid route mirror to execute. */
/* RC service has already captured takeover/release evidence.  The RC branch
 * below closes that snapshot before returning to the LCD start gate. */
#define enter_fault(code)                                      \
    do {                                                       \
        const uint32_t _fault_code = (code);                   \
        if (_fault_code == FAULT_RC_OVERRIDE) {                \
            (void)run_log_save((uint32_t)RUN_RC_OVERRIDE,      \
                               FAULT_RC_OVERRIDE);             \
            run_log_dump_stored();                             \
            board_uart1_write("H7,RC,ROUTE_ABORTED_BY_RC\r\n"); \
            goto route_start;                                 \
        }                                                      \
        g_fault_code = _fault_code;                            \
        route_controller_log_event(RUN_LOG_EVENT_FAULT);       \
        {                                                      \
            char _fault_log[96];                               \
            (void)snprintf(_fault_log, sizeof(_fault_log),     \
                           "H7,WARN,FAULT_CONTINUE,CODE=%lu," \
                           "NEXT_STAGE=1\r\n",                \
                           (unsigned long)_fault_code);        \
            board_uart1_write(_fault_log);                     \
        }                                                      \
        g_fault_code = FAULT_NONE;                             \
        if (g_run_state == RUN_FAULT) {                        \
            g_run_state = RUN_BOOT;                            \
        }                                                      \
    } while (0)

int main(void)
{
    board_field_t selected_field;
    route_field_profile_t field_profile;
    uint8_t task2_test_is_red = 0U;
    uint32_t task2_test_sequence = 0U;
    char task2_test_letter1[2] = {0};
    char task2_test_letter2[2] = {0};
    bool task2_test = false;
    uint8_t task3_test_is_red = 0U;
    uint32_t task3_test_sequence = 0U;
    bool task3_test = false;

    route_controller_init();

route_start:
    route_controller_wait_for_start();
    task2_test = route_controller_take_task2_test(
        &task2_test_is_red, &task2_test_sequence, task2_test_letter1,
        task2_test_letter2, sizeof(task2_test_letter1));
    task3_test = route_controller_take_task3_test(
        &task3_test_is_red, &task3_test_sequence);
    selected_field = (task2_test || task3_test)
                         ? (task2_test
                                ? (task2_test_is_red != 0U ? BOARD_FIELD_RED
                                                            : BOARD_FIELD_BLUE)
                                : (task3_test_is_red != 0U ? BOARD_FIELD_RED
                                                           : BOARD_FIELD_BLUE))
                         : board_selected_field();
    if (selected_field == BOARD_FIELD_UNKNOWN) {
        board_uart1_write("H7,FAULT,NO_FIELD_SELECTED\r\n");
        route_controller_enter_fault_wait_restart(FAULT_KINEMATICS);
        goto route_start;
    }
    field_profile = route_field_profile(selected_field);
    route_controller_set_field(field_profile.is_red);
    route_controller_reset_run_context();
    board_servo_apply_power_on_pose();

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
    if (!task2_test && !task3_test) {
        while (route_controller_rk_link_ready() == 0U) {
            route_controller_wait_for_rk_ready_on_boot();
        }
    }
#endif

    if (!task2_test && !task3_test) {
        route_controller_begin_pretask_sync();
    }
#if ROUTE_WAIT_RK_READY_ON_BOOT
    if (!task2_test && !task3_test &&
        !route_controller_wait_for_rk_reset_before_route()) {
        enter_fault(FAULT_ARM_TIMEOUT);
    }
#endif

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

    if (task2_test) {
        if (!route_controller_run_task2_test(task2_test_sequence,
                                             task2_test_letter1,
                                             task2_test_letter2)) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND
                                                   : g_fault_code);
        }
        board_uart1_write("H7,TEST,TASK2,WAIT_NEXT_STATION\r\n");
        for (;;) {
            uint32_t next_sequence;
            uint32_t step;

            if (!route_controller_wait_for_task2_test_next(
                    task2_test_sequence)) {
                goto route_test_shutdown;
            }
            next_sequence = task2_test_sequence;
            step = route_controller_task2_test_step();
            if (!route_controller_take_task2_test(
                    &task2_test_is_red, &next_sequence, task2_test_letter1,
                    task2_test_letter2, sizeof(task2_test_letter1))) {
                enter_fault(FAULT_ARM_REMOTE);
            }
            if (!route_controller_run_task2_test(
                    next_sequence, task2_test_letter1, task2_test_letter2)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND
                                                       : g_fault_code);
            }
            board_uart1_write(step >= 7U
                                  ? "H7,TEST,TASK2,ALL_STATIONS_DONE\r\n"
                                  : "H7,TEST,TASK2,WAIT_NEXT_STATION\r\n");
            if (step >= 7U) {
                goto route_test_shutdown;
            }
        }
    }

    if (task3_test) {
        g_run_state = RUN_FRONT_CENTER_ORBIT;
        if (!route_controller_run_task3_test(task3_test_sequence)) {
            if (g_fault_code == FAULT_NONE) {
                goto route_test_shutdown;
            }
            enter_fault(g_fault_code);
        }
        board_uart1_write("H7,TEST,TASK3,ORBIT_COMPLETE\r\n");
        goto route_test_shutdown;
    }

#if ROUTE_DISC_ARC_ENTRY_ENABLED
    /*
     * Replace the old strafe -> forward -> in-place turn sequence with one
     * continuous cubic entry.  The field profile mirrors both the side
     * of the obstacle and the final station heading.
     */
    /* Request the high pose before the first arc command.  The request and
     * ACK handling are non-blocking; the arc loop services the USB link while
     * the RK/C8T6 arm moves in parallel with chassis motion. */
    route_controller_start_disc_prep_high_async();
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
            field_profile.turn_sign * ROUTE_STANDARD_QUARTER_TURN_RAD *
            ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
#endif

    /* Move the arm to the high observation pose before querying the line.
     * Otherwise the white-line view can stay blocked and RK returns NOT_FOUND
     * before DISC_CATCH ever gets a chance to lift the arm. */
    if (!route_controller_wait_for_disc_prep_high()) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
    }

    /* Stop background sync at the first arm station.  When formal white-line
     * vision is enabled, this also makes H7 the sole reader of line replies. */
    route_controller_mark_first_arm_station();
#if ROUTE_DISC_VISUAL_ALIGN_ENABLED
    g_run_state = RUN_DISC_VISUAL_ALIGN;
    if (!route_controller_run_disc_visual_alignment()) {
        if (g_fault_code == FAULT_WHITE_LINE_NOT_FOUND) {
            char fallback_log[96];

            g_fault_code = FAULT_NONE;
            g_run_state = RUN_DISC_FINAL_APPROACH;
            (void)snprintf(
                fallback_log, sizeof(fallback_log),
                "H7,WARN,TASK1,WHITE_LINE_NOT_FOUND,CONTINUE,FALLBACK=%umm\r\n",
                (unsigned)(ROUTE_DISC_LINE_FALLBACK_FORWARD_M * 1000.0f +
                           0.5f));
            board_uart1_write(fallback_log);
            /* A zero fallback is an intentional no-motion policy. Calling
             * the distance controller with 0 m would create a false
             * kinematics fault and prevent DISC_CATCH from starting. */
            if (ROUTE_DISC_LINE_FALLBACK_FORWARD_M > 0.0f &&
                !route_controller_run_translation(
                    ROUTE_FORWARD_SIGN, 0.0f,
                    ROUTE_DISC_LINE_FALLBACK_FORWARD_M)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND
                                                        : g_fault_code);
            }
        } else {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
        }
    } else {
        g_run_state = RUN_DISC_FINAL_APPROACH;
            if (ROUTE_DISC_LINE_AFTER_CROSSED_FORWARD_M > 0.0f) {
            if (!route_controller_run_translation_profile(
                    ROUTE_FORWARD_SIGN, 0.0f,
                    ROUTE_DISC_LINE_AFTER_CROSSED_FORWARD_M,
                    ROUTE_DISC_LINE_FORWARD_SPEED_M_S,
                    ROUTE_DISC_LINE_ACCEL_M_S2)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
            }
        } else {
            board_uart1_write(
                "H7,ROUTE,WHITE_LINE,REFERENCE_REACHED,FORWARD=70mm\r\n");
        }
    }
#else
    /* Formal task-one line path is enabled above.  Keep this branch as a
     * commissioning fallback only when line alignment is deliberately
     * disabled; it must not be the normal route. */
    g_run_state = RUN_DISC_VISUAL_ALIGN;
    board_uart1_write(
        "H7,ROUTE,TASK1,WHITE_LINE,BYPASS,START,GYRO_ALIGN=ON,FORWARD=25.5mm\r\n");
    if (!route_controller_run_relative_turn(0.0f)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    g_run_state = RUN_DISC_FINAL_APPROACH;
    if (!route_controller_run_translation(
            ROUTE_FORWARD_SIGN, 0.0f, ROUTE_DISC_LINE_BYPASS_FORWARD_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    board_uart1_write(
        "H7,ROUTE,TASK1,WHITE_LINE,BYPASS,DONE,GYRO_ALIGN=ON,FORWARD=25.5mm\r\n");
#endif

#if ROUTE_STOP_AFTER_WHITE_LINE
    /* This commissioning route ends at the calibrated white-line pose. Keep
     * PREP_HIGH in place so the new arc-synchronous lift can be inspected. */
    board_uart1_write("H7,ROUTE,WHITE_LINE_TEST_COMPLETE\r\n");
    goto route_task1_only_shutdown;
#endif

#if ROUTE_TASK1_DISC_CATCH_ENABLED
    g_run_state = RUN_ARM_DISC_CATCH;
    route_controller_log_event(RUN_LOG_EVENT_ARM_START);
    if (!route_controller_wait_for_rk_arm_task(ROUTE_TASK1_RK_ARM_TASK)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
    }
    route_controller_log_event(
        route_controller_last_arm_task_soft_timed_out() != 0U
            ? RUN_LOG_EVENT_ARM_STOP
            : (route_controller_last_arm_task_bypassed() != 0U
                   ? RUN_LOG_EVENT_ARM_BYPASS
                   : RUN_LOG_EVENT_ARM_DONE));
#endif

#if ROUTE_TASK1_ONLY
    /* DISC_CATCH owns retraction and sends DONE only after the arm is home. */
    board_uart1_write("H7,ROUTE,TASK1_ONLY_COMPLETE\r\n");
    goto route_task1_only_shutdown;
#endif

#if !ROUTE_TASK1_ONLY
    /* DISC_CATCH sends DONE only after RK has closed ID7 and returned ID5,
     * ID1, ID2, and ID6 to the safe home pose. Keep that retracted state for
     * the entire task-one-to-task-two chassis transfer. */
    board_uart1_write(
        "H7,ARM,DISC_CATCH,HOME_CONFIRMED_BEFORE_TASK2_TRANSFER\r\n");

    /*
     * Enter task two as one continuous diagonal segment.  The route-frame
     * components reproduce the 1.35 m reverse and 2.12 m side approach.
     * The chassis rotates smoothly through 172 degrees during the segment,
     * mirrored by field, so there are no intermediate 90-degree stops.
     */
    g_run_state = RUN_TASK2_DIAGONAL_TURN;
    board_uart1_write(
        field_profile.is_red != 0U
            ? "H7,ROUTE,TASK2_DIAGONAL,FIELD=RED,BACKWARD=1350mm,"
              "LATERAL=2120mm,TURN=LEFT172\r\n"
            : "H7,ROUTE,TASK2_DIAGONAL,FIELD=BLUE,BACKWARD=1350mm,"
              "LATERAL=2120mm,TURN=RIGHT172\r\n");
    if (!route_controller_run_translation_with_turn(
            -ROUTE_FORWARD_SIGN * ROUTE_TASK2_ENTRY_BACKWARD_COMPONENT_M,
            field_profile.strafe_sign * ROUTE_TASK2_ENTRY_LATERAL_COMPONENT_M,
            ROUTE_TASK2_ENTRY_DIAGONAL_DISTANCE_M,
            ROUTE_TRANSLATION_SPEED_M_S,
            ROUTE_TRANSLATION_ACCEL_M_S2,
            field_profile.turn_sign * ROUTE_TASK2_ENTRY_TURN_ANGLE_RAD *
                ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    /* Close any residual yaw error left by the task-two diagonal before the
     * arm is raised and the camera-guided task-two entry begins. This reuses
     * the same BMI088 heading correction used after the task-one arc; it does
     * not re-zero the gyro bias while the route is running. */
    board_uart1_write(
        "H7,ROUTE,TASK2_ENTRY,GYRO_ALIGN,START,TARGET=DIAGONAL_FINAL\r\n");
    if (!route_controller_run_relative_turn(0.0f)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT
                                               : g_fault_code);
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }
    board_uart1_write(
        "H7,ROUTE,TASK2_ENTRY,GYRO_ALIGN,DONE,TARGET=DIAGONAL_FINAL\r\n");

#if ROUTE_TASK2_PLATFORM_PICK_ENABLED
    {
        uint32_t platform_index;

        /* PRESELECT_DONE means the secondary pair is locked AND RK has
         * waited for the task-two high pose. Keep the chassis stopped until
         * that barrier, then use the standalone white-line entry sequence. */
        g_run_state = RUN_ARM_PLATFORM_PICK;
        board_uart1_write(
            field_profile.is_red != 0U
                ? "H7,ROUTE,TASK2_PRESELECT,FIELD=RED,COUNT=2,BEFORE_WHITE_LINE\r\n"
                : "H7,ROUTE,TASK2_PRESELECT,FIELD=BLUE,COUNT=2,BEFORE_WHITE_LINE\r\n");
        if (!route_controller_wait_for_rk_platform_preselect()) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
        }
        board_uart1_write("H7,ROUTE,TASK2_PRESELECT_DONE,ARM_HIGH_READY\r\n");
        if (!route_controller_run_task2_platform_entry()) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
        }
        route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
        if (g_run_state == RUN_FAULT) {
            enter_fault(g_fault_code);
        }

        for (platform_index = 0U;
             platform_index < ROUTE_TASK2_PLATFORM_PICK_COUNT;
             ++platform_index) {
            char task2_log[96];

            g_run_state = RUN_ARM_PLATFORM_PICK;
            (void)snprintf(task2_log, sizeof(task2_log),
                           "H7,ROUTE,TASK2_SLOT_START,SLOT=%lu\r\n",
                           (unsigned long)(platform_index + 1U));
            board_uart1_write(task2_log);
            route_controller_log_event(RUN_LOG_EVENT_ARM_START);
            if (!route_controller_wait_for_rk_platform_slot(platform_index + 1U)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
            }
            route_controller_log_event(
                route_controller_last_arm_task_soft_timed_out() != 0U
                    ? RUN_LOG_EVENT_ARM_STOP
                    : (route_controller_last_arm_task_bypassed() != 0U
                           ? RUN_LOG_EVENT_ARM_BYPASS
                           : RUN_LOG_EVENT_ARM_DONE));
            if (platform_index + 1U < ROUTE_TASK2_PLATFORM_PICK_COUNT) {
                float shift_distance_m;

                switch (platform_index) {
                case 0U:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_1_M;
                    break;
                case 1U:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_2_M;
                    break;
                case 2U:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_3_M;
                    break;
                case 3U:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_4_M;
                    break;
                case 4U:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_5_M;
                    break;
                case 5U:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_6_M;
                    break;
                default:
                    shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_7_M;
                    break;
                }

                route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
                if (g_run_state == RUN_FAULT) {
                    enter_fault(g_fault_code);
                }
                g_run_state = field_profile.is_red != 0U
                                   ? RUN_PLATFORM_SHIFT_RIGHT
                                   : RUN_PLATFORM_SHIFT_LEFT;
                (void)snprintf(task2_log, sizeof(task2_log),
                               "H7,ROUTE,TASK2_SHIFT_MM,VALUE=%lu\r\n",
                               (unsigned long)(shift_distance_m * 1000.0f));
                board_uart1_write(task2_log);
                if (!route_controller_run_translation_profile(
                        0.0f, -field_profile.strafe_sign, shift_distance_m,
                        ROUTE_TASK2_TEST_TRANSLATION_SPEED_M_S,
                        ROUTE_TRANSLATION_ACCEL_M_S2)) {
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

        {
            g_run_state = field_profile.is_red != 0U
                               ? RUN_LAST_TURN_LEFT
                               : RUN_LAST_TURN_RIGHT;
            if (!route_controller_run_relative_turn(
                    field_profile.turn_sign * ROUTE_STANDARD_QUARTER_TURN_RAD *
                    ROUTE_GYRO_TURN_SCALE)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
            }
            route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }

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
            /* The formal route uses the slow, field-tested orbit controller,
             * but its route geometry is independent from the standalone test:
             * 360 degrees at the formal 450 mm front-center radius. */
            route_controller_set_task3_orbit_test_mode(1U);
            if (!route_controller_run_front_center_orbit(
                    field_profile.turn_sign * ROUTE_FORMAL_TASK3_ORBIT_ANGLE_RAD *
                        ROUTE_GYRO_TURN_SCALE,
                    ROUTE_FRONT_CENTER_ORBIT_RADIUS_M)) {
                route_controller_set_task3_orbit_test_mode(0U);
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
            }
            route_controller_set_task3_orbit_test_mode(0U);
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

            /* The red-field 360-degree orbit ends with the chassis facing the
             * reverse leg from the opposite side.  Add the required right
             * 88-degree transition only for RED; BLUE keeps the existing
             * post-orbit heading and reverse sequence. */
            if (field_profile.is_red != 0U) {
                g_run_state = RUN_TURN_RIGHT;
                board_uart1_write(
                    "H7,ROUTE,TASK3,POST_ORBIT_TURN,FIELD=RED,"
                    "DIR=RIGHT,ANGLE=88deg\r\n");
                if (!route_controller_run_relative_turn(
                        ROUTE_RIGHT_TURN_SIGN * ROUTE_STANDARD_QUARTER_TURN_RAD *
                        ROUTE_GYRO_TURN_SCALE)) {
                    enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT
                                                            : g_fault_code);
                }
                route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
                if (g_run_state == RUN_FAULT) {
                    enter_fault(g_fault_code);
                }
            }

            /* The RK STOP handler retracts the arm before this reverse. */
            g_run_state = RUN_FINAL_REVERSE;
            if (!route_controller_run_translation(
                    -ROUTE_FORWARD_SIGN, 0.0f,
                    ROUTE_TASK3_POST_REVERSE_DISTANCE_M)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND
                                                        : g_fault_code);
            }
            route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }

            /* Open PA0 first, hold it for the configured interval, then
             * restore its power-on angle before moving the chassis. */
            board_servo_set_angle_deg_index(
                SERVO_MG90S_PA0_INDEX, SERVO_MG90S_POST_ROUTE_PA0_ANGLE_DEG);
            board_uart1_write(
                "H7,LOCAL_SERVO,POST_ROUTE,INDEX,0,ANGLE_DEG,80.0,HOLD_MS,5000\r\n");
            route_controller_hold_zero(ROUTE_TASK3_POST_AUX_HOLD_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
            board_servo_set_angle_deg_index(
                SERVO_MG90S_PA0_INDEX, SERVO_MG90S_POWER_ON_PA0_ANGLE_DEG);
            board_uart1_write(
                "H7,LOCAL_SERVO,POST_ROUTE,INDEX,0,ANGLE_DEG,150.0,RESTORE\r\n");
            route_controller_hold_zero(ROUTE_SERVO_RETURN_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }

            /* Move left before operating PA2.  This is a chassis-left move
             * in the robot frame, independent of field color mirroring. */
            g_run_state = RUN_PLATFORM_SHIFT_LEFT;
            if (!route_controller_run_translation_profile(
                    0.0f, -ROUTE_RIGHT_STRAFE_SIGN,
                    ROUTE_POST_ROUTE_LEFT_SHIFT_DISTANCE_M,
                    ROUTE_TRANSLATION_SPEED_M_S,
                    ROUTE_TRANSLATION_ACCEL_M_S2)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND
                                                        : g_fault_code);
            }
            route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
            board_uart1_write(
                "H7,ROUTE,TASK3,POST_REVERSE_LEFT_SHIFT,DISTANCE=300mm\r\n");

            /* Operate PA2 after the left shift, then return it to its
             * power-on angle before the route is marked complete. */
            board_servo_set_angle_deg_index(
                SERVO_MG90S_PA2_INDEX, SERVO_MG90S_POST_ROUTE_PA2_ANGLE_DEG);
            board_uart1_write(
                "H7,LOCAL_SERVO,POST_ROUTE,INDEX,1,ANGLE_DEG,90.0,HOLD_MS,5000\r\n");
            route_controller_hold_zero(ROUTE_TASK3_POST_AUX_HOLD_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
            board_servo_set_angle_deg_index(
                SERVO_MG90S_PA2_INDEX, SERVO_MG90S_POWER_ON_PA2_ANGLE_DEG);
            board_uart1_write(
                "H7,LOCAL_SERVO,POST_ROUTE,INDEX,1,ANGLE_DEG,30.0,RESTORE\r\n");
            route_controller_hold_zero(ROUTE_SERVO_RETURN_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
        }

        board_uart1_write(
                    "H7,ROUTE,TASK3,COMPLETE,POST_REVERSE_DONE,DISTANCE=650mm\r\n");
#endif

#if ROUTE_TASK1_ONLY || ROUTE_STOP_AFTER_WHITE_LINE
route_task1_only_shutdown:
#endif
route_test_shutdown:
#if !ROUTE_TASK1_ONLY
    /* The test shutdown tail repeats the post sequence for the independent
     * task-two/task-three runs; the formal route already completed it inside
     * the field branch above, so it must not run twice. */
    if (task2_test || task3_test) {
        board_uart1_write(
            "H7,ROUTE,TASK3,POST_REVERSE,DONE,DISTANCE=650mm\r\n");

        g_run_state = RUN_AUX_ZP_S23;
        if (!route_controller_run_zp_aux(
                ROUTE_AUX_ZP_S23_CHANNEL, ROUTE_TASK3_POST_S23_PULSE,
                ROUTE_AUX_ZP_MG90S_TIME_MS, 0U)) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT
                                                    : g_fault_code);
        }
        route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
        if (g_run_state == RUN_FAULT) {
            enter_fault(g_fault_code);
        }

        g_run_state = RUN_AUX_ZP_S23;
        if (!route_controller_run_zp_aux(
                ROUTE_AUX_ZP_S23_CHANNEL, ROUTE_AUX_ZP_S23_PULSE,
                ROUTE_AUX_ZP_MG90S_TIME_MS, 0U)) {
            enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT
                                                    : g_fault_code);
        }
        route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
        if (g_run_state == RUN_FAULT) {
            enter_fault(g_fault_code);
        }

        if (field_profile.is_red != 0U) {
            g_run_state = RUN_PLATFORM_SHIFT_LEFT;
            if (!route_controller_run_translation_profile(
                    0.0f, field_profile.strafe_sign,
                    ROUTE_TASK3_POST_RED_LEFT_SHIFT_DISTANCE_M,
                    ROUTE_TRANSLATION_SPEED_M_S,
                    ROUTE_TRANSLATION_ACCEL_M_S2)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND
                                                        : g_fault_code);
            }
            route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }

            g_run_state = RUN_AUX_ZP_S12;
            if (!route_controller_run_zp_aux(
                    ROUTE_AUX_ZP_S12_CHANNEL, ROUTE_TASK3_POST_S12_PULSE,
                    ROUTE_AUX_ZP_MG90S_TIME_MS, 0U)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT
                                                        : g_fault_code);
            }
            route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }

            g_run_state = RUN_AUX_ZP_S12;
            if (!route_controller_run_zp_aux(
                    ROUTE_AUX_ZP_S12_CHANNEL,
                    ROUTE_TASK3_POST_S12_RESTORE_PULSE,
                    ROUTE_AUX_ZP_MG90S_TIME_MS, 0U)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT
                                                        : g_fault_code);
            }
            route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
            if (g_run_state == RUN_FAULT) {
                enter_fault(g_fault_code);
            }
        }

        board_uart1_write(
            field_profile.is_red != 0U
                ? "H7,ROUTE,TASK3,POST_DONE,S23=500->1000,LEFT=300mm,S12=1500->600\r\n"
                : "H7,ROUTE,TASK3,POST_DONE,S23=500->1000\r\n");
    }
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
