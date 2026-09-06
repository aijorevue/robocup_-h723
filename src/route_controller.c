#include "app_config.h"
#include "board.h"
#include "BMI088driver.h"
#include "lcd_display.h"
#include "route_controller.h"
#include "mecanum.h"
#include "motor_output.h"
#include "rc_override.h"
#include "run_log.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

volatile run_state_t g_run_state = RUN_BOOT;
volatile uint32_t g_fault_code = FAULT_NONE;
volatile uint8_t g_bmi088_init_error;
volatile float g_gyro_z_bias_rad_s;
volatile float g_gyro_z_rad_s;
volatile float g_yaw_rad;
volatile float g_command_speed_m_s;
volatile float g_heading_correction_rad_s;
volatile float g_estimated_distance_m;
volatile float g_imu_temperature_c;
volatile float g_cross_track_m;
volatile float g_cross_track_command_m_s;
volatile float g_actual_cross_speed_m_s;
static float g_route_heading_target_rad;
static float g_accel_body_bias_m_s2[2];
static float g_accel_body_filtered_m_s2[2];
static volatile uint8_t g_rk_arm_link_ready;
static volatile uint8_t g_rk_disc_prep_high_ack;
static uint8_t g_rk_disc_prep_high_requested;
static uint32_t g_rk_disc_prep_high_last_send_ms;
static uint8_t g_rk_last_task_bypassed;
static uint8_t g_rk_last_task_soft_timed_out;
static uint8_t g_first_arm_station_reached;
static uint8_t g_start_confirmed_from_fault;
static uint8_t g_route_field_is_red;
static uint8_t g_rk_reset_pending;
static uint32_t g_rk_pretask_last_sync_ms;
static uint32_t g_rk_task_sequence_counter;
static uint32_t g_rk_async_task_sequence;
static char g_rk_pretask_line[96];
static uint32_t g_rk_pretask_line_len;
static uint8_t g_task2_test_pending;
static uint8_t g_task2_test_is_red;
static uint32_t g_task2_test_sequence;
static char g_task2_test_letter1;
static char g_task2_test_letter2;
static uint32_t g_task2_test_step;
static uint32_t g_task2_test_active_sequence;
static uint8_t g_task2_test_stop_requested;
static uint8_t g_task2_test_white_line_active;
static uint8_t g_task3_test_pending;
static uint8_t g_task3_test_is_red;
static uint32_t g_task3_test_sequence;
static uint32_t g_task3_test_active_sequence;
static uint8_t g_task3_test_pause_requested;
static uint8_t g_task3_test_resume_requested;
static uint8_t g_task3_test_stop_requested;
static uint8_t g_task3_test_paused;
static uint8_t g_task3_test_slow_requested;

typedef enum {
    ROUTE_WHITE_LINE_PHASE_TASK1_AFTER_ARC = 1,
    ROUTE_WHITE_LINE_PHASE_TASK2_AFTER_SHIFT = 2
} route_white_line_phase_t;

static void service_rk_link_before_first_station(void);
static bool service_disc_prep_high_during_arc(void);
static bool service_task2_test_command(void);
static bool service_task3_test_command(void);
static bool handle_local_mg90s_command(const char *line);
static bool run_disc_visual_alignment(void);
static bool run_disc_visual_alignment_at_speed(float forward_speed_m_s,
                                               long reference_y10,
                                               long tolerance_y10,
                                               float acceleration_m_s2,
                                               route_white_line_phase_t phase);
/* A route cycle sends its command first, then samples feedback for the next
 * cycle. The first command in a route (and the first command after a TASK3
 * resume) may proceed without the normal three-fresh-channel gate. */
static bool route_motor_feedback_update_after_command(float wheel_rad_s[4],
                                                       bool *allow_missing_once);
static bool enable_motors(void);

static void request_rk_arm_reset(void)
{
    g_rk_reset_pending = 1U;
    g_rk_arm_link_ready = 0U;
    g_rk_disc_prep_high_ack = 0U;
    g_rk_disc_prep_high_requested = 0U;
    g_rk_disc_prep_high_last_send_ms = 0U;
    g_first_arm_station_reached = 0U;
    g_rk_pretask_line_len = 0U;
    g_rk_pretask_last_sync_ms = HAL_GetTick() - RK_ARM_PRETASK_SYNC_PERIOD_MS;
    board_uart1_write(g_route_field_is_red != 0U
                          ? "H7,ARM,RESET_REQUESTED,FIELD=RED\r\n"
                          : "H7,ARM,RESET_REQUESTED,FIELD=BLUE\r\n");
    board_usb_write(g_route_field_is_red != 0U
                        ? "ARM,SYNC,RESET,FIELD,RED\r\n"
                        : "ARM,SYNC,RESET,FIELD,BLUE\r\n");
}

_Static_assert(IMU_ACCEL_BODY_X_INDEX < 3U,
               "IMU body X axis index must be 0, 1, or 2");
_Static_assert(IMU_ACCEL_BODY_Y_INDEX < 3U,
               "IMU body Y axis index must be 0, 1, or 2");
_Static_assert(IMU_ACCEL_BODY_X_INDEX != IMU_ACCEL_BODY_Y_INDEX,
               "IMU body X and Y axes must use different sensor axes");
_Static_assert(IMU_VELOCITY_PREDICTION_WEIGHT >= 0.0f &&
                   IMU_VELOCITY_PREDICTION_WEIGHT <= 1.0f,
               "IMU velocity prediction weight must be between 0 and 1");

uint8_t route_controller_rk_link_ready(void)
{
    return g_rk_arm_link_ready;
}

#if ROUTE_WAIT_USER_KEY_ON_BOOT
static void wait_for_user_start_key_release(const char *status)
{
    uint32_t last_status_ms = HAL_GetTick() - 1000U;

    while (board_user_start_active() != 0U) {
        const uint32_t now_ms = HAL_GetTick();

        lcd_display_update();
        /* Standalone TASK2/TASK3 START is an independent launch source. Poll
         * it even while the RC key is held so a test never depends on
         * releasing or moving the joystick first. */
        if (service_task2_test_command()) {
            lcd_display_set_start_status("RUN");
            board_uart1_write_only("H7,START,TASK2_TEST,FIELD_SELECTED\r\n");
            return;
        }
        if (g_task3_test_pending != 0U) {
            lcd_display_set_start_status("RUN");
            board_uart1_write_only("H7,START,TASK3_TEST,FIELD_SELECTED\r\n");
            return;
        }
        if ((uint32_t)(now_ms - last_status_ms) >= 1000U) {
            last_status_ms = now_ms;
            board_uart1_write("H7,START,WAIT_KEY_RELEASE\r\n");
        }
        if (rc_override_service()) {
            lcd_display_set_start_status(status);
        }
        HAL_Delay(10U);
    }
}

static void wait_for_user_start_key(void)
{
    uint32_t last_status_ms = HAL_GetTick() - 1000U;
    char status_line[96];

    g_run_state = RUN_WAIT_USB_RUN;
    lcd_display_set_start_status("WAIT");
    board_uart1_write("H7,START,WAIT_FIELD,joystick=RIGHT_RED_OR_DOWN_BLUE\r\n");
    wait_for_user_start_key_release("WAIT");
    for (;;) {
        uint32_t now_ms = HAL_GetTick();

        lcd_display_update();
        if ((uint32_t)(now_ms - last_status_ms) >= 1000U) {
            const board_lcd_joystick_direction_t direction =
                board_lcd_joystick_direction();
            const uint32_t raw = board_lcd_joystick_raw();

            last_status_ms = now_ms;
            (void)snprintf(status_line, sizeof(status_line),
                           "H7,START,WAIT_USER_KEY,joy=%u,raw=%lu\r\n",
                           (unsigned int)direction, (unsigned long)raw);
            board_uart1_write(status_line);
        }
        if (rc_override_service()) {
            lcd_display_set_start_status("WAIT");
            continue;
        }
        if (service_task2_test_command()) {
            lcd_display_set_start_status("RUN");
            board_uart1_write("H7,START,TASK2_TEST,FIELD_SELECTED\r\n");
            return;
        }
        if (g_task3_test_pending != 0U) {
            lcd_display_set_start_status("RUN");
            board_uart1_write("H7,START,TASK3_TEST,FIELD_SELECTED\r\n");
            return;
        }
        if (board_user_start_pressed() != 0U &&
            board_selected_field() != BOARD_FIELD_UNKNOWN) {
            lcd_display_set_start_status("RUN");
            lcd_display_refresh_input_status();
            switch (board_selected_field()) {
            case BOARD_FIELD_RED:
                board_uart1_write("H7,START,USER_KEY,field=RED\r\n");
                break;
            case BOARD_FIELD_BLUE:
                board_uart1_write("H7,START,USER_KEY,field=BLUE\r\n");
                break;
            case BOARD_FIELD_UNKNOWN:
            default:
                board_uart1_write("H7,START,USER_KEY,field=UNKNOWN\r\n");
                break;
            }
            return;
        }
        HAL_Delay(10U);
    }
}
#endif

static const mecanum_config_t chassis = {
    .wheel_radius_m = WHEEL_RADIUS_M,
    .half_length_m = CHASSIS_HALF_LENGTH_M,
    .half_width_m = CHASSIS_HALF_WIDTH_M,
    .max_wheel_rad_s = MOTOR_MAX_WHEEL_RAD_S,
    .direction = {WHEEL_FL_SIGN, WHEEL_FR_SIGN, WHEEL_RL_SIGN, WHEEL_RR_SIGN}
};

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float mapped_body_accel(const float accel[3], uint32_t axis,
                               float sign)
{
    return accel[axis] * sign;
}

static float filtered_linear_accel(float value)
{
    value = clampf(value, -IMU_ACCEL_MAX_M_S2, IMU_ACCEL_MAX_M_S2);
    if (fabsf(value) <= IMU_ACCEL_DEADBAND_M_S2) {
        return 0.0f;
    }
    return value > 0.0f ? value - IMU_ACCEL_DEADBAND_M_S2
                        : value + IMU_ACCEL_DEADBAND_M_S2;
}

static void drain_motor_feedback(void)
{
    /* Zero, enable, and disable commands still produce motor replies.  Drain
     * them even when the route is not consuming feedback for odometry so the
     * hardware RX FIFO cannot fill during long arm-task or settle waits. */
    motor_feedback_drain(HAL_GetTick());
}

static bool route_motor_send_zero_all(void)
{
    service_rk_link_before_first_station();
    if (!rc_override_is_running() && rc_override_service()) {
        request_rk_arm_reset();
        g_fault_code = FAULT_RC_OVERRIDE;
        return false;
    }
    drain_motor_feedback();
    if (motor_send_zero_all()) {
        return true;
    }
#if ROUTE_REQUIRE_MOTOR_TX_SUCCESS
    return false;
#else
    return true;
#endif
}

static bool keep_chassis_stopped_for_arm_task(void)
{
    return route_motor_send_zero_all();
}

static void preserve_rc_or_set_motor_fault(void)
{
    if (g_fault_code != FAULT_RC_OVERRIDE) {
        g_fault_code = FAULT_MOTOR_COMMAND;
    }
}

static bool route_motor_send_wheel_speeds(const float wheel_rad_s[4])
{
    service_rk_link_before_first_station();
    if (!rc_override_is_running() && rc_override_service()) {
        request_rk_arm_reset();
        g_fault_code = FAULT_RC_OVERRIDE;
        return false;
    }
    if (motor_send_wheel_speeds(wheel_rad_s)) {
        return true;
    }
#if ROUTE_REQUIRE_MOTOR_TX_SUCCESS
    return false;
#else
    return true;
#endif
}

static void hold_zero(uint32_t duration_ms)
{
    uint32_t started_ms = HAL_GetTick();
    uint32_t last_send_ms = started_ms - CONTROL_PERIOD_MS;

    while ((uint32_t)(HAL_GetTick() - started_ms) < duration_ms) {
        uint32_t now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - last_send_ms) >= CONTROL_PERIOD_MS) {
            last_send_ms = now_ms;
            if (!route_motor_send_zero_all()) {
                preserve_rc_or_set_motor_fault();
                g_run_state = RUN_FAULT;
                return;
            }
        }
        HAL_Delay(1U);
    }
}

static void enter_fault_wait_restart(uint32_t code)
{
    static const float stopped[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t start_release_seen;
    uint32_t last_safe_refresh_ms;

    g_fault_code = code;
    g_run_state = RUN_FAULT;
    drain_motor_feedback();
    (void)motor_send_zero_all();
    (void)motor_disable_all();
    run_log_sample(HAL_GetTick(), (uint32_t)g_run_state, g_fault_code,
                   g_gyro_z_rad_s, g_yaw_rad, g_command_speed_m_s,
                   g_heading_correction_rad_s, g_estimated_distance_m,
                   g_imu_temperature_c, stopped, g_cross_track_m,
                   g_cross_track_command_m_s, g_actual_cross_speed_m_s,
                   RUN_LOG_EVENT_FAULT);
    (void)run_log_save((uint32_t)g_run_state, g_fault_code);
    run_log_dump_stored();
    /* A failed standalone task-two run must not poison the next formal or
     * standalone start with its old active sequence. */
    g_task2_test_pending = 0U;
    g_task2_test_active_sequence = 0U;
    g_task2_test_stop_requested = 0U;
    g_task2_test_step = 0U;
    g_task3_test_pending = 0U;
    g_task3_test_active_sequence = 0U;
    g_task3_test_pause_requested = 0U;
    g_task3_test_resume_requested = 0U;
    g_task3_test_stop_requested = 0U;
    g_task3_test_paused = 0U;
    request_rk_arm_reset();
    lcd_display_set_start_status("FAULT");
    board_uart1_write("H7,FAULT,SAVED,WAIT_START_TO_RESET\r\n");
    board_clear_selected_field();
    start_release_seen = board_user_start_active() == 0U ? 1U : 0U;
    last_safe_refresh_ms = HAL_GetTick();
    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        if ((uint32_t)(now_ms - last_safe_refresh_ms) >=
            ROUTE_FAULT_SAFE_REFRESH_MS) {
            last_safe_refresh_ms = now_ms;
            drain_motor_feedback();
            (void)motor_send_zero_all();
            (void)motor_disable_all();
        }
        service_rk_link_before_first_station();
        lcd_display_update();
        if (rc_override_service()) {
            g_start_confirmed_from_fault = 0U;
            start_release_seen = 0U;
            lcd_display_set_start_status("FAULT");
            continue;
        }
        if (board_user_start_active() == 0U) {
            start_release_seen = 1U;
        }
        if (start_release_seen != 0U && board_user_start_pressed() != 0U) {
            g_start_confirmed_from_fault = 1U;
            lcd_display_set_start_status("RUN");
            lcd_display_refresh_input_status();
            board_uart1_write("H7,FAULT,USER_KEY_RESTART\r\n");
            return;
        }
        HAL_Delay(20U);
    }
}

static bool calibrate_gyro(void)
{
    float gyro[3] = {0.0f};
    float accel[3] = {0.0f};
    float temperature = 0.0f;
    float sum = 0.0f;
    float sum_square = 0.0f;
    float accel_x_sum = 0.0f;
    float accel_y_sum = 0.0f;
    uint32_t sample;

    for (sample = 0U; sample < GYRO_CALIBRATION_SAMPLES; ++sample) {
        float z;
        BMI088_read(gyro, accel, &temperature);
        z = gyro[2] * GYRO_Z_SIGN;
        sum += z;
        sum_square += z * z;
        accel_x_sum += mapped_body_accel(
            accel, IMU_ACCEL_BODY_X_INDEX, IMU_ACCEL_BODY_X_SIGN);
        accel_y_sum += mapped_body_accel(
            accel, IMU_ACCEL_BODY_Y_INDEX, IMU_ACCEL_BODY_Y_SIGN);
        g_imu_temperature_c = temperature;
        service_rk_link_before_first_station();
        if (rc_override_service()) {
            g_fault_code = FAULT_RC_OVERRIDE;
            return false;
        }
        HAL_Delay(GYRO_CALIBRATION_PERIOD_MS);
    }

    g_gyro_z_bias_rad_s = sum / (float)GYRO_CALIBRATION_SAMPLES;
    g_accel_body_bias_m_s2[0] =
        accel_x_sum / (float)GYRO_CALIBRATION_SAMPLES;
    g_accel_body_bias_m_s2[1] =
        accel_y_sum / (float)GYRO_CALIBRATION_SAMPLES;
    g_accel_body_filtered_m_s2[0] = 0.0f;
    g_accel_body_filtered_m_s2[1] = 0.0f;
    {
        float variance = sum_square / (float)GYRO_CALIBRATION_SAMPLES -
                         g_gyro_z_bias_rad_s * g_gyro_z_bias_rad_s;
        float limit = GYRO_STATIONARY_STDDEV_MAX_RAD_S;
        if (variance < 0.0f) {
            variance = 0.0f;
        }
        if (variance > limit * limit) {
            return false;
        }
    }
    return true;
}

static bool enable_motors(void)
{
    drain_motor_feedback();
    if (!motor_clear_errors_all() ||
        !board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS)) {
        return false;
    }
    drain_motor_feedback();
    if (!motor_enable_all() ||
        !board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS)) {
        return false;
    }
    drain_motor_feedback();
    return motor_send_zero_all() &&
           board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS);
}

static bool wait_for_can_startup(void)
{
    uint32_t started_ms;

    started_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_ms) < ROUTE_POWER_ON_SETTLE_MS) {
        service_rk_link_before_first_station();
        if (rc_override_service()) {
            g_fault_code = FAULT_RC_OVERRIDE;
            return false;
        }
        HAL_Delay(20U);
    }
    started_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_ms) < CAN_STARTUP_RETRY_TIMEOUT_MS) {
        service_rk_link_before_first_station();
        if (rc_override_service()) {
            g_fault_code = FAULT_RC_OVERRIDE;
            return false;
        }
        (void)board_fdcan1_abort_all_pending();
        if (motor_send_zero_all() &&
            board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS)) {
            return true;
        }
        HAL_Delay(CAN_STARTUP_RETRY_GAP_MS);
    }
    return false;
}

#if ROUTE_AUTO_RUN_ON_BOOT == 0U
static bool command_matches(const char *command, const char *target)
{
    size_t index = 0U;

    while (target[index] != '\0') {
        char c = command[index];
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 'a' + 'A');
        }
        if (c != target[index]) {
            return false;
        }
        ++index;
    }
    return command[index] == '\0' || command[index] == '\r' ||
           command[index] == '\n' || command[index] == ' ';
}
#endif

static bool task2_test_letter(char value)
{
    return value == 'A' || value == 'B' || value == 'C' || value == 'D';
}

static bool parse_task3_test_start(const char *line, uint8_t *is_red,
                                   uint32_t *sequence)
{
    char copy[160];
    char *tokens[12];
    char *token;
    char *end;
    size_t count = 0U;
    unsigned long parsed_sequence;
    unsigned long parsed_radius;
    unsigned long parsed_angle;

    if (line == NULL || is_red == NULL || sequence == NULL) {
        return false;
    }
    (void)snprintf(copy, sizeof(copy), "%s", line);
    token = strtok(copy, ",");
    while (token != NULL && count < (sizeof(tokens) / sizeof(tokens[0]))) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }
    if (count != 12U || strcmp(tokens[0], "RK") != 0 ||
        strcmp(tokens[1], "TEST") != 0 || strcmp(tokens[2], "TASK3") != 0 ||
        strcmp(tokens[3], "START") != 0 || strcmp(tokens[4], "SEQ") != 0 ||
        strcmp(tokens[6], "FIELD") != 0 ||
        strcmp(tokens[8], "RADIUS_MM") != 0 ||
        strcmp(tokens[10], "ANGLE_DEG") != 0) {
        return false;
    }
    parsed_sequence = strtoul(tokens[5], &end, 10);
    if (*tokens[5] == '\0' || *end != '\0' || parsed_sequence == 0UL ||
        parsed_sequence > 0xFFFFFFFFUL) {
        return false;
    }
    if (strcmp(tokens[7], "RED") == 0) {
        *is_red = 1U;
    } else if (strcmp(tokens[7], "BLUE") == 0) {
        *is_red = 0U;
    } else {
        return false;
    }
    parsed_radius = strtoul(tokens[9], &end, 10);
    if (*tokens[9] == '\0' || *end != '\0' ||
        parsed_radius != ROUTE_TASK3_TEST_ORBIT_RADIUS_MM) {
        return false;
    }
    parsed_angle = strtoul(tokens[11], &end, 10);
    if (*tokens[11] == '\0' || *end != '\0' ||
        parsed_angle != ROUTE_TASK3_TEST_ORBIT_ANGLE_DEG) {
        return false;
    }
    *sequence = (uint32_t)parsed_sequence;
    return true;
}

static bool parse_task3_test_control(const char *line, const char *command,
                                     uint32_t *sequence, uint8_t *is_red)
{
    char copy[128];
    char *tokens[8];
    char *token;
    char *end;
    size_t count = 0U;
    unsigned long parsed_sequence;

    if (line == NULL || command == NULL || sequence == NULL ||
        is_red == NULL) {
        return false;
    }
    (void)snprintf(copy, sizeof(copy), "%s", line);
    token = strtok(copy, ",");
    while (token != NULL && count < (sizeof(tokens) / sizeof(tokens[0]))) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }
    if (count != 8U || strcmp(tokens[0], "RK") != 0 ||
        strcmp(tokens[1], "TEST") != 0 || strcmp(tokens[2], "TASK3") != 0 ||
        strcmp(tokens[3], command) != 0 || strcmp(tokens[4], "SEQ") != 0 ||
        strcmp(tokens[6], "FIELD") != 0 ||
        (strcmp(tokens[7], "RED") != 0 && strcmp(tokens[7], "BLUE") != 0)) {
        return false;
    }
    parsed_sequence = strtoul(tokens[5], &end, 10);
    if (*tokens[5] == '\0' || *end != '\0' || parsed_sequence == 0UL ||
        parsed_sequence > 0xFFFFFFFFUL) {
        return false;
    }
    *sequence = (uint32_t)parsed_sequence;
    *is_red = strcmp(tokens[7], "RED") == 0 ? 1U : 0U;
    return true;
}

static bool parse_task2_test_start(const char *line, uint8_t *is_red,
                                   uint32_t *sequence, char *letter1,
                                   char *letter2)
{
    char copy[160];
    char *tokens[12];
    char *token;
    char *end;
    size_t count = 0U;
    unsigned long parsed_sequence;

    if (line == NULL || is_red == NULL || sequence == NULL ||
        letter1 == NULL || letter2 == NULL) {
        return false;
    }
    (void)snprintf(copy, sizeof(copy), "%s", line);
    token = strtok(copy, ",");
    while (token != NULL && count < (sizeof(tokens) / sizeof(tokens[0]))) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }
    if (count != 12U || strcmp(tokens[0], "RK") != 0 ||
        strcmp(tokens[1], "TEST") != 0 || strcmp(tokens[2], "TASK2") != 0 ||
        strcmp(tokens[3], "START") != 0 || strcmp(tokens[4], "SEQ") != 0 ||
        strcmp(tokens[6], "FIELD") != 0 || strcmp(tokens[8], "L1") != 0 ||
        strcmp(tokens[10], "L2") != 0) {
        return false;
    }
    parsed_sequence = strtoul(tokens[5], &end, 10);
    if (*tokens[5] == '\0' || *end != '\0' || parsed_sequence == 0UL ||
        parsed_sequence > 0xFFFFFFFFUL || tokens[9][1] != '\0' ||
        tokens[11][1] != '\0') {
        return false;
    }
    if (strcmp(tokens[7], "RED") == 0) {
        *is_red = 1U;
    } else if (strcmp(tokens[7], "BLUE") == 0) {
        *is_red = 0U;
    } else {
        return false;
    }
    *letter1 = tokens[9][0];
    *letter2 = tokens[11][0];
    if (!task2_test_letter(*letter1) || !task2_test_letter(*letter2) ||
        *letter1 == *letter2) {
        return false;
    }
    *sequence = (uint32_t)parsed_sequence;
    return true;
}

static bool parse_task2_test_stop(const char *line, uint32_t *sequence,
                                  uint8_t *is_red)
{
    char copy[160];
    char *tokens[8];
    char *token;
    char *end;
    size_t count = 0U;
    unsigned long parsed_sequence;

    if (line == NULL || sequence == NULL || is_red == NULL) {
        return false;
    }
    (void)snprintf(copy, sizeof(copy), "%s", line);
    token = strtok(copy, ",");
    while (token != NULL && count < (sizeof(tokens) / sizeof(tokens[0]))) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }
    if (count != 8U || strcmp(tokens[0], "RK") != 0 ||
        strcmp(tokens[1], "TEST") != 0 || strcmp(tokens[2], "TASK2") != 0 ||
        strcmp(tokens[3], "STOP") != 0 || strcmp(tokens[4], "SEQ") != 0 ||
        strcmp(tokens[6], "FIELD") != 0 ||
        (strcmp(tokens[7], "RED") != 0 && strcmp(tokens[7], "BLUE") != 0)) {
        return false;
    }
    parsed_sequence = strtoul(tokens[5], &end, 10);
    if (*tokens[5] == '\0' || *end != '\0' || parsed_sequence == 0UL ||
        parsed_sequence > 0xFFFFFFFFUL) {
        return false;
    }
    *sequence = (uint32_t)parsed_sequence;
    *is_red = strcmp(tokens[7], "RED") == 0 ? 1U : 0U;
    return true;
}

static bool parse_task2_test_next(const char *line, uint32_t *sequence,
                                  uint32_t *step, uint8_t *is_red)
{
    char copy[160];
    char *tokens[10];
    char *token;
    char *end;
    size_t count = 0U;
    unsigned long parsed_sequence;
    unsigned long parsed_step;

    if (line == NULL || sequence == NULL || step == NULL || is_red == NULL) {
        return false;
    }
    (void)snprintf(copy, sizeof(copy), "%s", line);
    token = strtok(copy, ",");
    while (token != NULL && count < (sizeof(tokens) / sizeof(tokens[0]))) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }
    if (count != 10U || strcmp(tokens[0], "RK") != 0 ||
        strcmp(tokens[1], "TEST") != 0 || strcmp(tokens[2], "TASK2") != 0 ||
        strcmp(tokens[3], "NEXT") != 0 || strcmp(tokens[4], "SEQ") != 0 ||
        strcmp(tokens[6], "FIELD") != 0 || strcmp(tokens[8], "STEP") != 0 ||
        (strcmp(tokens[7], "RED") != 0 && strcmp(tokens[7], "BLUE") != 0)) {
        return false;
    }
    parsed_sequence = strtoul(tokens[5], &end, 10);
    if (*tokens[5] == '\0' || *end != '\0' || parsed_sequence == 0UL ||
        parsed_sequence > 0xFFFFFFFFUL) {
        return false;
    }
    parsed_step = strtoul(tokens[9], &end, 10);
    if (*tokens[9] == '\0' || *end != '\0' || parsed_step < 1UL ||
        parsed_step > 7UL) {
        return false;
    }
    *sequence = (uint32_t)parsed_sequence;
    *step = (uint32_t)parsed_step;
    *is_red = strcmp(tokens[7], "RED") == 0 ? 1U : 0U;
    return true;
}

static bool service_task2_test_command(void)
{
    static char line[160];
    static uint32_t line_len;
    static uint8_t rx_evidence_saved;
    uint8_t rx[96];
    uint32_t read_len = CDC_Read_HS(rx, sizeof(rx));
    uint32_t index;

    if (read_len > 0U && rx_evidence_saved == 0U) {
        rx_evidence_saved = 1U;
        if (run_log_save_event(RUN_WAIT_USB_RUN, FAULT_NONE,
                               RUN_LOG_EVENT_TASK2_RX_BYTES)) {
            board_uart1_write_only("H7,LOG,TASK2_RX_BYTES,SAVED\r\n");
        } else {
            board_uart1_write_only("H7,LOG,TASK2_RX_BYTES,SAVE_FAIL\r\n");
        }
    }

    for (index = 0U; index < read_len; ++index) {
        const char c = (char)rx[index];

        if (c == '\r' || c == '\n') {
            uint8_t is_red;
            uint32_t sequence;
            char letter1;
            char letter2;
            char *frame_start;

            line[line_len] = '\0';
            /* A CDC reconnect can leave a partial old frame in the device
             * ring. Resynchronize at either standalone test header before
             * applying strict token validation below. */
            frame_start = strstr(line, "RK,TEST,TASK2,");
            if (frame_start == NULL) {
                frame_start = strstr(line, "RK,TEST,TASK3,");
            }
            if (frame_start != NULL && frame_start != line) {
                const size_t normalized_len = strlen(frame_start);
                memmove(line, frame_start, normalized_len + 1U);
                line_len = (uint32_t)normalized_len;
            }
            if (line_len > 0U && g_run_state == RUN_WAIT_USB_RUN &&
                handle_local_mg90s_command(line)) {
                line_len = 0U;
                return false;
            }
            if (line_len > 0U &&
                parse_task3_test_start(line, &is_red, &sequence)) {
                if (g_task2_test_pending != 0U ||
                    g_task2_test_active_sequence != 0U ||
                    g_task3_test_pending != 0U ||
                    g_task3_test_active_sequence != 0U) {
                    board_usb_write("H7,TEST,TASK3,BUSY\r\n");
                } else {
                    char response[128];

                    g_task3_test_pending = 1U;
                    g_task3_test_is_red = is_red;
                    g_task3_test_sequence = sequence;
                    g_task3_test_pause_requested = 0U;
                    g_task3_test_resume_requested = 0U;
                    g_task3_test_stop_requested = 0U;
                    g_task3_test_paused = 0U;
                    g_task3_test_slow_requested = 0U;
                    (void)snprintf(
                        response, sizeof(response),
                        "H7,TEST,TASK3,ACK,SEQ,%lu,FIELD,%s\r\n",
                        (unsigned long)sequence,
                        is_red != 0U ? "RED" : "BLUE");
                    board_usb_write(response);
                    board_uart1_write_only("H7,TEST,TASK3,ACK_ACCEPTED\r\n");
                }
                line_len = 0U;
                return false;
            }
            if (line_len > 0U && parse_task2_test_start(
                    line, &is_red, &sequence, &letter1, &letter2)) {
                if (g_task2_test_pending != 0U ||
                    g_task2_test_active_sequence != 0U) {
                    board_usb_write("H7,TEST,TASK2,BUSY\r\n");
                } else {
                    g_task2_test_pending = 1U;
                    g_task2_test_is_red = is_red;
                    g_task2_test_sequence = sequence;
                    g_task2_test_letter1 = letter1;
                    g_task2_test_letter2 = letter2;
                    g_task2_test_step = 0U;
                    if (run_log_save_event(RUN_WAIT_USB_RUN, FAULT_NONE,
                                           RUN_LOG_EVENT_TASK2_RX_START)) {
                        board_uart1_write_only(
                            "H7,LOG,TASK2_RX_START,SAVED\r\n");
                    } else {
                        board_uart1_write_only(
                            "H7,LOG,TASK2_RX_START,SAVE_FAIL\r\n");
                    }
                    {
                        char response[96];
                        (void)snprintf(response, sizeof(response),
                                       "H7,TEST,TASK2,ACK,SEQ,%lu,FIELD,%s\r\n",
                                       (unsigned long)sequence,
                                       is_red != 0U ? "RED" : "BLUE");
                        board_usb_write(response);
                    }
                    /* Keep the protocol ACK sequence-aware.  The old generic
                     * logger emitted a second ACK without SEQ, which could
                     * confuse a strict RK transaction matcher. */
                    board_uart1_write_only("H7,TEST,TASK2,ACK_ACCEPTED\r\n");
                    line_len = 0U;
                    return true;
                    }
            } else if (line_len > 0U) {
                uint32_t next_sequence;
                uint32_t next_step;
                uint8_t next_is_red;
                if (parse_task2_test_next(line, &next_sequence, &next_step,
                                          &next_is_red) &&
                    g_task2_test_pending == 0U &&
                    g_task2_test_active_sequence == next_sequence &&
                    g_task2_test_is_red == next_is_red &&
                    next_step >= 1U && next_step <= 7U) {
                    g_task2_test_pending = 1U;
                    g_task2_test_sequence = next_sequence;
                    g_task2_test_step = next_step;
                    {
                        char response[96];
                        (void)snprintf(response, sizeof(response),
                                       "H7,TEST,TASK2,ACK,SEQ,%lu,FIELD,%s,STEP,%lu\r\n",
                                       (unsigned long)next_sequence,
                                       g_task2_test_is_red != 0U ? "RED" : "BLUE",
                                       (unsigned long)next_step);
                        board_usb_write(response);
                    }
                    line_len = 0U;
                    return true;
                }
                uint32_t stop_sequence;
                uint8_t stop_is_red;
                if (parse_task2_test_stop(line, &stop_sequence, &stop_is_red) &&
                    g_task2_test_active_sequence == stop_sequence &&
                    g_task2_test_is_red == stop_is_red) {
                    g_task2_test_stop_requested = 1U;
                } else {
                    /* Keep malformed/irrelevant input diagnosable without
                     * echoing it through the USB protocol channel. */
                    board_uart1_write_only("H7,TEST,TASK2,ERR,REASON,PARSE\r\n");
                    {
                        char response[96];
                        (void)snprintf(response, sizeof(response),
                                       "H7,TEST,TASK2,ERR,REASON,PARSE,LEN,%lu\r\n",
                                       (unsigned long)line_len);
                        board_usb_write(response);
                    }
                }
            }
            line_len = 0U;
        } else if (line_len + 1U < sizeof(line)) {
            line[line_len++] = c;
        } else {
            line_len = 0U;
            board_usb_write("H7,TEST,TASK2,ERR,REASON,LINE_TOO_LONG\r\n");
        }
    }
    return false;
}

static void task2_test_send_status(const char *status, uint32_t sequence)
{
    char response[128];

    (void)snprintf(response, sizeof(response),
                   "H7,TEST,TASK2,%s,SEQ,%lu,FIELD,%s\r\n", status,
                   (unsigned long)sequence,
                   g_task2_test_is_red != 0U ? "RED" : "BLUE");
    board_usb_write(response);
}

static void task2_test_send_error(const char *reason, uint32_t sequence)
{
    char response[128];

    (void)snprintf(response, sizeof(response),
                   "H7,TEST,TASK2,ERR,SEQ,%lu,REASON,%s,FIELD,%s\r\n",
                   (unsigned long)sequence, reason,
                   g_task2_test_is_red != 0U ? "RED" : "BLUE");
    board_usb_write(response);
}

static void task3_test_send_status(const char *status, uint32_t sequence)
{
    char response[128];

    (void)snprintf(response, sizeof(response),
                   "H7,TEST,TASK3,%s,SEQ,%lu,FIELD,%s\r\n", status,
                   (unsigned long)sequence,
                   g_task3_test_is_red != 0U ? "RED" : "BLUE");
    board_usb_write(response);
}

static void task3_test_send_ack(const char *command, uint32_t sequence)
{
    char response[160];

    (void)snprintf(response, sizeof(response),
                   "H7,TEST,TASK3,ACK,SEQ,%lu,FIELD,%s,COMMAND,%s\r\n",
                   (unsigned long)sequence,
                   g_task3_test_is_red != 0U ? "RED" : "BLUE", command);
    board_usb_write(response);
}

static void task3_test_send_error(const char *reason, uint32_t sequence)
{
    char response[160];

    (void)snprintf(response, sizeof(response),
                   "H7,TEST,TASK3,ERR,SEQ,%lu,REASON,%s,FIELD,%s\r\n",
                   (unsigned long)sequence, reason,
                   g_task3_test_is_red != 0U ? "RED" : "BLUE");
    board_usb_write(response);
}

static bool service_task3_test_command(void)
{
    static char line[128];
    static uint32_t line_len;
    uint8_t rx[96];
    uint32_t read_len = CDC_Read_HS(rx, sizeof(rx));
    uint32_t index;

    for (index = 0U; index < read_len; ++index) {
        const char c = (char)rx[index];

        if (c == '\r' || c == '\n') {
            uint32_t sequence;
            uint8_t is_red;
            char *frame_start;

            line[line_len] = '\0';
            /* A reconnect can leave a partial old frame in the CDC ring.
             * Resynchronize at the TASK3 header before parsing PAUSE,
             * RESUME, or STOP so one stale prefix cannot lose a control
             * command. */
            frame_start = strstr(line, "RK,TEST,TASK3,");
            if (frame_start != NULL && frame_start != line) {
                const size_t normalized_len = strlen(frame_start);
                memmove(line, frame_start, normalized_len + 1U);
                line_len = (uint32_t)normalized_len;
            }
            if (line_len > 0U && g_task3_test_active_sequence != 0U) {
                if (parse_task3_test_control(line, "PAUSE", &sequence,
                                              &is_red) &&
                    sequence == g_task3_test_active_sequence &&
                    is_red == g_task3_test_is_red) {
                    if (g_task3_test_paused == 0U) {
                        g_task3_test_pause_requested = 1U;
                    }
                    board_uart1_write_only(
                        "H7,TEST,TASK3,CONTROL,PAUSE_ACCEPTED\r\n");
                    task3_test_send_ack("PAUSE", sequence);
                } else if (parse_task3_test_control(
                               line, "RESUME", &sequence, &is_red) &&
                           sequence == g_task3_test_active_sequence &&
                           is_red == g_task3_test_is_red) {
                    g_task3_test_resume_requested = 1U;
                    board_uart1_write_only(
                        "H7,TEST,TASK3,CONTROL,RESUME_ACCEPTED\r\n");
                    task3_test_send_ack("RESUME", sequence);
                } else if (parse_task3_test_control(
                               line, "STOP", &sequence, &is_red) &&
                           sequence == g_task3_test_active_sequence &&
                           is_red == g_task3_test_is_red) {
                    g_task3_test_stop_requested = 1U;
                    board_uart1_write_only(
                        "H7,TEST,TASK3,CONTROL,STOP_ACCEPTED\r\n");
                    task3_test_send_ack("STOP", sequence);
                } else if (parse_task3_test_control(
                               line, "SLOW", &sequence, &is_red) &&
                           sequence == g_task3_test_active_sequence &&
                           is_red == g_task3_test_is_red) {
                    g_task3_test_slow_requested = 1U;
                    board_uart1_write_only(
                        "H7,TEST,TASK3,CONTROL,SLOW_ACCEPTED\r\n");
                    task3_test_send_ack("SLOW", sequence);
                } else if (parse_task3_test_control(
                               line, "NORMAL", &sequence, &is_red) &&
                           sequence == g_task3_test_active_sequence &&
                           is_red == g_task3_test_is_red) {
                    g_task3_test_slow_requested = 0U;
                    board_uart1_write_only(
                        "H7,TEST,TASK3,CONTROL,NORMAL_ACCEPTED\r\n");
                    task3_test_send_ack("NORMAL", sequence);
                } else {
                    task3_test_send_error("PARSE", g_task3_test_active_sequence);
                }
            }
            line_len = 0U;
        } else if (line_len + 1U < sizeof(line)) {
            line[line_len++] = c;
        } else {
            line_len = 0U;
            task3_test_send_error("LINE_TOO_LONG", g_task3_test_active_sequence);
        }
    }
    return read_len > 0U;
}

/* Return 1 to keep orbiting, 0 when a valid STOP completed the test, 2 when a
 * valid RESUME was accepted, and -1 when the motor/RC safety path failed
 * while holding the chassis. */
static int task3_test_service_orbit_pause(uint32_t sequence)
{
    uint32_t last_paused_status_ms;

    /* TASK3 owns the CDC stream while orbiting. Poll here before checking the
     * request flags so a PAUSE/RESUME/STOP sent during motion is not stranded
     * until the orbit loop exits. */
    (void)service_task3_test_command();

    if (g_task3_test_stop_requested != 0U) {
        if (!route_motor_send_zero_all()) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return -1;
        }
        task3_test_send_status("STOPPED", sequence);
        g_task3_test_active_sequence = 0U;
        g_task3_test_pause_requested = 0U;
        g_task3_test_resume_requested = 0U;
        g_task3_test_stop_requested = 0U;
        g_task3_test_paused = 0U;
        g_task3_test_slow_requested = 0U;
        g_fault_code = FAULT_NONE;
        return 0;
    }
    if (g_task3_test_pause_requested == 0U) {
        return 1;
    }

    if (!route_motor_send_zero_all()) {
        g_fault_code = FAULT_MOTOR_COMMAND;
        return -1;
    }
    g_task3_test_pause_requested = 0U;
    g_task3_test_paused = 1U;
    task3_test_send_status("PAUSED", sequence);
    last_paused_status_ms = HAL_GetTick();
    board_uart1_write_only("H7,TEST,TASK3,PAUSE_HOLD\r\n");

    while (g_task3_test_paused != 0U) {
        const uint32_t now_ms = HAL_GetTick();

        (void)service_task3_test_command();
        if (rc_override_service()) {
            (void)route_motor_send_zero_all();
            task3_test_send_status("STOPPED", sequence);
            g_task3_test_active_sequence = 0U;
            g_task3_test_pause_requested = 0U;
            g_task3_test_resume_requested = 0U;
            g_task3_test_stop_requested = 0U;
            g_task3_test_paused = 0U;
            g_task3_test_slow_requested = 0U;
            g_fault_code = FAULT_NONE;
            return 0;
        }
        if (g_task3_test_stop_requested != 0U) {
            if (!route_motor_send_zero_all()) {
                g_fault_code = FAULT_MOTOR_COMMAND;
                return -1;
            }
            task3_test_send_status("STOPPED", sequence);
            g_task3_test_active_sequence = 0U;
            g_task3_test_pause_requested = 0U;
            g_task3_test_resume_requested = 0U;
            g_task3_test_stop_requested = 0U;
            g_task3_test_paused = 0U;
            g_fault_code = FAULT_NONE;
            return 0;
        }
        if (g_task3_test_resume_requested != 0U) {
            g_task3_test_resume_requested = 0U;
            g_task3_test_paused = 0U;
            g_task3_test_slow_requested = 0U;
            task3_test_send_status("RESUMED", sequence);
            board_uart1_write_only("H7,TEST,TASK3,RESUME_ACCEPTED\r\n");
            return 2;
        }
        /* CDC IN can be busy when another status line is emitted. Keep the
         * confirmed stopped state observable without sending a burst. */
        if ((uint32_t)(now_ms - last_paused_status_ms) >=
            ROUTE_TASK3_TEST_STATUS_RETRY_MS) {
            last_paused_status_ms = now_ms;
            task3_test_send_status("PAUSED", sequence);
        }
        lcd_display_update();
        HAL_Delay(10U);
    }
    return 1;
}

#if ROUTE_AUTO_RUN_ON_BOOT == 0U
static void wait_for_usb_run_command(void)
{
    uint8_t rx[32];
    char line[32];
    uint32_t line_len = 0U;
    uint32_t last_hello_ms = HAL_GetTick() - 1000U;

    g_run_state = RUN_WAIT_USB_RUN;
    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

        if ((uint32_t)(now_ms - last_hello_ms) >= 1000U) {
            last_hello_ms = now_ms;
            board_uart1_write_only("H7,USB,READY,send RUN to start\r\n");
        }
        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (i = 0U; i < read_len; ++i) {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    if (command_matches(line, "PING")) {
                        board_usb_write("H7,PONG\r\n");
                    } else if (command_matches(line, "LOG")) {
                        run_log_dump_stored();
                    } else if (command_matches(line, "RUN")) {
                        board_uart1_write("H7,USB,RUN\r\n");
                        return;
                    } else if (command_matches(line, "STOP")) {
                        board_uart1_write("H7,USB,STOPPED\r\n");
                    } else {
                        board_uart1_write_only("H7,ERR,UNKNOWN_CMD\r\n");
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write_only("H7,ERR,CMD_TOO_LONG\r\n");
            }
        }
    }
}
#endif

static void update_imu(float dt)
{
    float gyro[3] = {0.0f};
    float accel[3] = {0.0f};
    float temperature = 0.0f;

    BMI088_read(gyro, accel, &temperature);
    g_imu_temperature_c = temperature;
    g_gyro_z_rad_s = gyro[2] * GYRO_Z_SIGN - g_gyro_z_bias_rad_s;
    g_yaw_rad += g_gyro_z_rad_s * dt;
    {
        const float accel_body_x =
            mapped_body_accel(accel, IMU_ACCEL_BODY_X_INDEX,
                              IMU_ACCEL_BODY_X_SIGN) -
            g_accel_body_bias_m_s2[0];
        const float accel_body_y =
            mapped_body_accel(accel, IMU_ACCEL_BODY_Y_INDEX,
                              IMU_ACCEL_BODY_Y_SIGN) -
            g_accel_body_bias_m_s2[1];

        g_accel_body_filtered_m_s2[0] += IMU_ACCEL_FILTER_ALPHA *
            (accel_body_x - g_accel_body_filtered_m_s2[0]);
        g_accel_body_filtered_m_s2[1] += IMU_ACCEL_FILTER_ALPHA *
            (accel_body_y - g_accel_body_filtered_m_s2[1]);
    }
}

typedef struct {
    float route_vx_m_s;
    float route_vy_m_s;
    bool initialized;
} translation_velocity_observer_t;

static void update_translation_velocity_observer(
    translation_velocity_observer_t *observer,
    float encoder_route_vx_m_s, float encoder_route_vy_m_s,
    float heading_cos, float heading_sin, float dt)
{
    float accel_body_x;
    float accel_body_y;
    float accel_route_x;
    float accel_route_y;
    float predicted_vx;
    float predicted_vy;
    float innovation_vx;
    float innovation_vy;

    if (observer == NULL) {
        return;
    }
    if (!observer->initialized) {
        observer->route_vx_m_s = encoder_route_vx_m_s;
        observer->route_vy_m_s = encoder_route_vy_m_s;
        observer->initialized = true;
        return;
    }

    accel_body_x = filtered_linear_accel(g_accel_body_filtered_m_s2[0]);
    accel_body_y = filtered_linear_accel(g_accel_body_filtered_m_s2[1]);
    accel_route_x = heading_cos * accel_body_x + heading_sin * accel_body_y;
    accel_route_y = -heading_sin * accel_body_x + heading_cos * accel_body_y;
    predicted_vx = observer->route_vx_m_s + accel_route_x * dt;
    predicted_vy = observer->route_vy_m_s + accel_route_y * dt;
    innovation_vx = clampf(
        predicted_vx - encoder_route_vx_m_s,
        -IMU_VELOCITY_MAX_ENCODER_DELTA_M_S,
        IMU_VELOCITY_MAX_ENCODER_DELTA_M_S);
    innovation_vy = clampf(
        predicted_vy - encoder_route_vy_m_s,
        -IMU_VELOCITY_MAX_ENCODER_DELTA_M_S,
        IMU_VELOCITY_MAX_ENCODER_DELTA_M_S);
    observer->route_vx_m_s = encoder_route_vx_m_s +
        IMU_VELOCITY_PREDICTION_WEIGHT * innovation_vx;
    observer->route_vy_m_s = encoder_route_vy_m_s +
        IMU_VELOCITY_PREDICTION_WEIGHT * innovation_vy;

    if (fabsf(encoder_route_vx_m_s) <= IMU_ZERO_VELOCITY_THRESHOLD_M_S &&
        fabsf(encoder_route_vy_m_s) <= IMU_ZERO_VELOCITY_THRESHOLD_M_S &&
        fabsf(g_command_speed_m_s) <= IMU_ZERO_VELOCITY_THRESHOLD_M_S) {
        observer->route_vx_m_s = 0.0f;
        observer->route_vy_m_s = 0.0f;
    }
}

static void log_route_sample(uint32_t now_ms, const float wheel_speed[4])
{
    run_log_sample(now_ms, (uint32_t)g_run_state, g_fault_code,
                   g_gyro_z_rad_s, g_yaw_rad, g_command_speed_m_s,
                   g_heading_correction_rad_s, g_estimated_distance_m,
                   g_imu_temperature_c, wheel_speed, g_cross_track_m,
                   g_cross_track_command_m_s, g_actual_cross_speed_m_s,
                   RUN_LOG_EVENT_SAMPLE);
}

static void log_route_event(uint32_t event)
{
    static const float stopped[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    run_log_sample(HAL_GetTick(), (uint32_t)g_run_state, g_fault_code,
                   g_gyro_z_rad_s, g_yaw_rad, g_command_speed_m_s,
                   g_heading_correction_rad_s, g_estimated_distance_m,
                   g_imu_temperature_c, stopped, g_cross_track_m,
                   g_cross_track_command_m_s, g_actual_cross_speed_m_s,
                   event);
}
static bool line_starts_with(const char *line, const char *prefix)
{
    size_t index = 0U;

    while (prefix[index] != '\0') {
        char a = line[index];
        char b = prefix[index];
        if (a >= 'a' && a <= 'z') {
            a = (char)(a - 'a' + 'A');
        }
        if (b >= 'a' && b <= 'z') {
            b = (char)(b - 'a' + 'A');
        }
        if (a != b) {
            return false;
        }
        ++index;
    }
    return true;
}

static bool line_matches_token_prefix(const char *line, const char *prefix)
{
    const size_t prefix_length = strlen(prefix);

    if (!line_starts_with(line, prefix)) {
        return false;
    }
    return line[prefix_length] == '\0' || line[prefix_length] == ',';
}

/* PA0/TIM2_CH1 is a local H7 PWM output, not a ZP20S bus servo.  Accept
 * manual commands only while the chassis is idle or waiting in fault state;
 * the autonomous route never grants this command path control of the servo. */
static bool handle_local_mg90s_command(const char *line)
{
    char copy[128];
    char *tokens[6];
    char *token;
    char *end;
    char *index_end;
    size_t count = 0U;
    unsigned long servo_index;
    float angle_deg;
    char response[128];

    if (line == NULL || !line_starts_with(line, "LOCAL_SERVO,")) {
        return false;
    }

    (void)snprintf(copy, sizeof(copy), "%s", line);
    token = strtok(copy, ",");
    while (token != NULL && count < (sizeof(tokens) / sizeof(tokens[0]))) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }

    if (count != 6U || strcmp(tokens[0], "LOCAL_SERVO") != 0 ||
        strcmp(tokens[1], "SET") != 0 || strcmp(tokens[2], "INDEX") != 0 ||
        strcmp(tokens[4], "ANGLE_DEG") != 0) {
        board_usb_write("H7,LOCAL_SERVO,ERR,REASON,PARSE\r\n");
        board_uart1_write_only("H7,LOCAL_SERVO,ERR,REASON,PARSE\r\n");
        return true;
    }

    servo_index = strtoul(tokens[3], &index_end, 10);
    if (*tokens[3] == '\0' || *index_end != '\0' ||
        (servo_index != SERVO_MG90S_PA0_INDEX &&
         servo_index != SERVO_MG90S_PA2_INDEX)) {
        board_usb_write("H7,LOCAL_SERVO,ERR,REASON,SERVO_INDEX\r\n");
        board_uart1_write_only("H7,LOCAL_SERVO,ERR,REASON,SERVO_INDEX\r\n");
        return true;
    }

    angle_deg = strtof(tokens[5], &end);
    if (*tokens[5] == '\0' || *end != '\0' || !isfinite(angle_deg) ||
        angle_deg < 0.0f || angle_deg > SERVO_MG90S_MAX_ANGLE_DEG) {
        board_usb_write("H7,LOCAL_SERVO,ERR,REASON,ANGLE_RANGE\r\n");
        board_uart1_write_only("H7,LOCAL_SERVO,ERR,REASON,ANGLE_RANGE\r\n");
        return true;
    }

    board_servo_set_angle_deg_index((uint8_t)servo_index, angle_deg);
    (void)snprintf(response, sizeof(response),
                   "H7,LOCAL_SERVO,ACK,INDEX,%lu,ANGLE_DEG,%.1f\r\n",
                   servo_index, (double)angle_deg);
    board_usb_write(response);
    board_uart1_write_only(response);
    return true;
}

static uint32_t next_rk_task_sequence(void)
{
    ++g_rk_task_sequence_counter;
    if (g_rk_task_sequence_counter == 0U) {
        g_rk_task_sequence_counter = 1U;
    }
    return g_rk_task_sequence_counter;
}

static void rk_arm_handle_line(const char *line)
{
    if (line_starts_with(line, "RK,ARM,DISC_CATCH,PREP_HIGH_ACK")) {
        g_rk_disc_prep_high_ack = 1U;
        g_rk_arm_link_ready = 1U;
        board_uart1_write_only("H7,ARM,DISC_CATCH,PREP_HIGH_CONFIRMED\r\n");
        return;
    }
    if (line_starts_with(line, "RK,ARM,RESET,DONE")) {
        g_rk_reset_pending = 0U;
        g_rk_arm_link_ready = 1U;
        board_uart1_write_only("H7,ARM,RK_RESET_DONE\r\n");
        return;
    }
    if (line_starts_with(line, "RK,ARM,RESET,ACK")) {
        board_uart1_write_only("H7,ARM,RK_RESET_ACK\r\n");
        return;
    }
    if (line_starts_with(line, "RK,ARM,READY")) {
        g_rk_arm_link_ready = 1U;
        board_uart1_write_only("H7,ARM,RK_READY\r\n");
    }
}

static bool service_disc_prep_high_during_arc(void)
{
    const uint32_t now_ms = HAL_GetTick();
    char command[96];
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";

    if (g_rk_disc_prep_high_requested == 0U) {
        g_rk_disc_prep_high_requested = 1U;
        g_rk_disc_prep_high_last_send_ms =
            now_ms - ROUTE_DISC_PREP_RETRY_PERIOD_MS;
        g_rk_disc_prep_high_ack = 0U;
        board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_ASYNC_START\r\n");
    }

    if ((uint32_t)(now_ms - g_rk_disc_prep_high_last_send_ms) >=
        ROUTE_DISC_PREP_RETRY_PERIOD_MS) {
        g_rk_disc_prep_high_last_send_ms = now_ms;
        (void)snprintf(command, sizeof(command),
                       "ARM,DISC_CATCH,PREP_HIGH,FIELD,%s,ID1,%d,ID2,%d,ID6,%d\r\n",
                       field_name, ROUTE_DISC_PREP_HIGH_ID1_TICK,
                       ROUTE_DISC_PREP_HIGH_ID2_TICK,
                       ROUTE_DISC_PREP_HIGH_ID6_TICK);
        board_usb_write(command);
    }

    /* This is deliberately non-blocking: the arc controller keeps running
     * while RK raises ID1/ID6 first, then starts ID2 after the configured
     * 200 ms mechanical-clearance delay. */
    service_rk_link_before_first_station();
    return g_rk_disc_prep_high_ack != 0U;
}

static void service_rk_link_before_first_station(void)
{
    uint8_t rx[64];
    uint32_t now_ms;
    uint32_t read_len;
    uint32_t i;

    if (g_task3_test_active_sequence != 0U) {
        /* TASK3 owns the CDC command stream while its orbit is active. */
        return;
    }
    if (g_task2_test_active_sequence != 0U) {
        /* The standalone task-two route owns CDC while it is active.  Do not
         * let the formal ARM parser consume its NEXT/STOP frames. */
        if (g_task2_test_white_line_active == 0U) {
            (void)service_task2_test_command();
        }
        return;
    }
    if (g_first_arm_station_reached != 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    /* Once PREP_HIGH starts, do not let the older background SYNC/RESET
     * traffic overwrite the arm pose while the chassis is on the arc. */
    if (g_rk_disc_prep_high_requested == 0U &&
        (uint32_t)(now_ms - g_rk_pretask_last_sync_ms) >=
            RK_ARM_PRETASK_SYNC_PERIOD_MS) {
        g_rk_pretask_last_sync_ms = now_ms;
        if (g_rk_reset_pending != 0U) {
            board_usb_write(g_route_field_is_red != 0U
                                ? "ARM,SYNC,RESET,FIELD,RED\r\n"
                                : "ARM,SYNC,RESET,FIELD,BLUE\r\n");
        } else {
            board_usb_write(g_route_field_is_red != 0U
                                ? "ARM,SYNC,FIELD,RED\r\n"
                                : "ARM,SYNC,FIELD,BLUE\r\n");
        }
    }

    read_len = CDC_Read_HS(rx, sizeof(rx));
    for (i = 0U; i < read_len; ++i) {
        char c = (char)rx[i];

        if (c == '\r' || c == '\n') {
            g_rk_pretask_line[g_rk_pretask_line_len] = '\0';
            if (g_rk_pretask_line_len > 0U) {
                board_uart1_write_only("H7,USB,RX,");
                board_uart1_write_only(g_rk_pretask_line);
                board_uart1_write_only("\r\n");
                if (!(g_run_state == RUN_WAIT_USB_RUN ||
                      g_run_state == RUN_FAULT) ||
                    !handle_local_mg90s_command(g_rk_pretask_line)) {
                    rk_arm_handle_line(g_rk_pretask_line);
                }
            }
            g_rk_pretask_line_len = 0U;
        } else if (g_rk_pretask_line_len + 1U < sizeof(g_rk_pretask_line)) {
            g_rk_pretask_line[g_rk_pretask_line_len++] = c;
        } else {
            g_rk_pretask_line_len = 0U;
            board_uart1_write("H7,ERR,PRETASK_ARM_LINE_TOO_LONG\r\n");
        }
    }
}

#if ROUTE_WAIT_RK_READY_ON_BOOT
static void wait_for_rk_ready_on_boot(void)
{
    uint8_t rx[64];
    char line[96];
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t last_sync_ms = started_ms - RK_ARM_START_RETRY_MS;

    board_uart1_write("H7,ARM,BOOT_WAIT_RK\r\n");
    while ((uint32_t)(HAL_GetTick() - started_ms) < RK_ARM_BOOT_READY_TIMEOUT_MS) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len = CDC_Read_HS(rx, sizeof(rx));
        uint32_t i;

        if ((uint32_t)(now_ms - last_sync_ms) >= RK_ARM_START_RETRY_MS) {
            last_sync_ms = now_ms;
            board_usb_write(g_route_field_is_red != 0U
                                ? "ARM,SYNC,FIELD,RED\r\n"
                                : "ARM,SYNC,FIELD,BLUE\r\n");
        }

        for (i = 0U; i < read_len; ++i) {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    board_uart1_write_only("H7,USB,RX,");
                    board_uart1_write_only(line);
                    board_uart1_write_only("\r\n");
                    if (!(g_run_state == RUN_WAIT_USB_RUN ||
                          g_run_state == RUN_FAULT) ||
                        !handle_local_mg90s_command(line)) {
                        rk_arm_handle_line(line);
                    }
                    if (g_rk_arm_link_ready != 0U) {
                        board_uart1_write("H7,ARM,BOOT_RK_READY\r\n");
                        return;
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write("H7,ERR,BOOT_ARM_LINE_TOO_LONG\r\n");
            }
        }
        HAL_Delay(20U);
    }

    board_uart1_write("H7,ARM,BOOT_RK_READY_TIMEOUT\r\n");
}
#endif

static bool wait_for_rk_arm_task(const char *task)
{
    uint8_t rx[64];
    char line[96];
    char log_line[96];
    char start_command[64];
    char status_command[64];
    char ack_prefix[64];
    char done_prefix[64];
    char idle_prefix[64];
    char busy_prefix[64];
    char error_prefix[64];
    bool rk_acknowledged = false;
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t task_started_ms = 0U;
    const uint32_t sequence = next_rk_task_sequence();
    uint32_t ack_timeout_ms = (g_rk_arm_link_ready != 0U) ?
                              RK_ARM_ACK_TIMEOUT_MS :
                              RK_ARM_PROBE_ACK_TIMEOUT_MS;
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_status_ms = started_ms;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";
    const uint32_t task_timeout_ms =
        strcmp(task, "DISC_CATCH") == 0
            ? RK_ARM_DISC_CATCH_TASK_TIMEOUT_MS
            : RK_ARM_TASK_TIMEOUT_MS;

    g_rk_last_task_bypassed = 0U;
    g_rk_last_task_soft_timed_out = 0U;

    (void)snprintf(start_command, sizeof(start_command),
                    "ARM,%s,START,SEQ,%lu,FIELD,%s\r\n", task,
                    (unsigned long)sequence, field_name);
    (void)snprintf(status_command, sizeof(status_command),
                    "ARM,%s,STATUS,SEQ,%lu\r\n", task,
                    (unsigned long)sequence);
    (void)snprintf(ack_prefix, sizeof(ack_prefix),
                    "RK,ARM,%s,ACK,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(done_prefix, sizeof(done_prefix),
                    "RK,ARM,%s,DONE,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(idle_prefix, sizeof(idle_prefix),
                    "RK,ARM,%s,IDLE,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(busy_prefix, sizeof(busy_prefix),
                    "RK,ARM,%s,BUSY,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(error_prefix, sizeof(error_prefix),
                    "RK,ARM,%s,ERR,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(log_line, sizeof(log_line),
                   "H7,ARM,%s,WAIT_RK,SEQ=%lu\r\n", task,
                   (unsigned long)sequence);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

#if RK_ARM_TASK_TIMEOUT_MS > 0U
        if (rk_acknowledged &&
            task_timeout_ms > 0U &&
            (uint32_t)(now_ms - task_started_ms) >= task_timeout_ms) {
            char stop_command[64];

            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,SOFT_TIMEOUT,SEQ=%lu\r\n", task,
                           (unsigned long)sequence);
            board_uart1_write(log_line);
            (void)snprintf(stop_command, sizeof(stop_command),
                           "ARM,%s,STOP,SEQ,%lu\r\n", task,
                           (unsigned long)sequence);
            board_usb_write(stop_command);
            board_usb_write(stop_command);
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,STOP_SENT_AFTER_SOFT_TIMEOUT,SEQ=%lu\r\n",
                           task, (unsigned long)sequence);
            board_uart1_write(log_line);
            g_rk_last_task_bypassed = 1U;
            g_rk_last_task_soft_timed_out = 1U;
            return true;
        }
#endif
        if ((uint32_t)(now_ms - last_zero_ms) >= CONTROL_PERIOD_MS) {
            last_zero_ms = now_ms;
            if (!keep_chassis_stopped_for_arm_task()) {
                preserve_rc_or_set_motor_fault();
                return false;
            }
        }
        if (!rk_acknowledged &&
            (uint32_t)(now_ms - last_send_ms) >= RK_ARM_START_RETRY_MS) {
            last_send_ms = now_ms;
            board_usb_write(start_command);
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,START_SENT\r\n", task);
            board_uart1_write_only(log_line);
        }
        if (rk_acknowledged &&
            (uint32_t)(now_ms - last_status_ms) >= RK_ARM_STATUS_PERIOD_MS) {
            last_status_ms = now_ms;
            board_usb_write(status_command);
        }

        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (i = 0U; i < read_len; ++i) {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    board_uart1_write_only("H7,USB,RX,");
                    board_uart1_write_only(line);
                    board_uart1_write_only("\r\n");
                    rk_arm_handle_line(line);
                    if (g_rk_arm_link_ready != 0U &&
                        ack_timeout_ms < RK_ARM_ACK_TIMEOUT_MS) {
                        ack_timeout_ms = RK_ARM_ACK_TIMEOUT_MS;
                    }
                    if (line_matches_token_prefix(line, ack_prefix)) {
                        if (!rk_acknowledged) {
                            task_started_ms = HAL_GetTick();
                        }
                        rk_acknowledged = true;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,ACK,SEQ=%lu\r\n", task,
                                       (unsigned long)sequence);
                        board_uart1_write(log_line);
                    }
                    if (line_matches_token_prefix(line, done_prefix)) {
                        g_rk_last_task_bypassed =
                            (strcmp(task, "DISC_CATCH") == 0 &&
                             strstr(line, ",REASON,RECOVERED_") != NULL)
                                ? 1U
                                : 0U;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,DONE,SEQ=%lu\r\n", task,
                                       (unsigned long)sequence);
                        board_uart1_write(log_line);
                        if (g_rk_last_task_bypassed != 0U) {
                            board_uart1_write(
                                "H7,WARN,TASK1,DISC_CATCH_RECOVERED,"
                                "ARM_HOME=OK,CONTINUE\r\n");
                        }
                        return true;
                    }
                    if (line_matches_token_prefix(line, error_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,REMOTE_ERROR,SEQ=%lu\r\n", task,
                                       (unsigned long)sequence);
                        board_uart1_write(log_line);
                        g_fault_code = FAULT_ARM_REMOTE;
                        return false;
                    }
                    if (line_matches_token_prefix(line, busy_prefix)) {
                        if (ack_timeout_ms < RK_ARM_BUSY_TIMEOUT_MS) {
                            ack_timeout_ms = RK_ARM_BUSY_TIMEOUT_MS;
                            (void)snprintf(log_line, sizeof(log_line),
                                           "H7,ARM,%s,RK_BUSY,SEQ=%lu\r\n", task,
                                           (unsigned long)sequence);
                            board_uart1_write(log_line);
                        }
                    }
                    if (rk_acknowledged &&
                        line_matches_token_prefix(line, idle_prefix)) {
                        rk_acknowledged = false;
                        started_ms = HAL_GetTick();
                        task_started_ms = 0U;
                        ack_timeout_ms = RK_ARM_ACK_TIMEOUT_MS;
                        last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,RESTART_AFTER_RK_IDLE,SEQ=%lu\r\n",
                                       task, (unsigned long)sequence);
                        board_uart1_write(log_line);
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write("H7,ERR,ARM_LINE_TOO_LONG\r\n");
            }
        }
#if RK_ARM_WAIT_FOREVER_FOR_ACK == 0U
        now_ms = HAL_GetTick();
        if (!rk_acknowledged &&
            (uint32_t)(now_ms - started_ms) >= ack_timeout_ms) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,BYPASS_NO_RK,SEQ=%lu\r\n", task,
                           (unsigned long)sequence);
            board_uart1_write(log_line);
            g_rk_arm_link_ready = 0U;
            g_rk_last_task_bypassed = 1U;
#if RK_ARM_REQUIRED
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return true;
#endif
        }
#endif
        HAL_Delay(1U);
    }
}

static bool wait_for_rk_platform_transaction(uint32_t slot, bool preselect)
{
    uint8_t rx[64];
    char line[128];
    char log_line[128];
    char command[128];
    char status_command[96];
    char ack_prefix[96];
    char done_prefix[96];
    char error_prefix[96];
    bool rk_acknowledged = false;
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t task_started_ms = 0U;
    uint32_t sequence = next_rk_task_sequence();
    uint32_t ack_timeout_ms = (g_rk_arm_link_ready != 0U)
                                  ? RK_ARM_ACK_TIMEOUT_MS
                                  : RK_ARM_PROBE_ACK_TIMEOUT_MS;
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";

    if (preselect) {
        (void)snprintf(
            command, sizeof(command),
            "ARM,PLATFORM_PICK,PRESELECT,SEQ,%lu,FIELD,%s,COUNT,%u\r\n",
            (unsigned long)sequence, field_name,
            (unsigned)ROUTE_TASK2_PLATFORM_PRESELECT_COUNT);
        (void)snprintf(
            ack_prefix, sizeof(ack_prefix),
            "RK,ARM,PLATFORM_PICK,PRESELECT_ACK,SEQ,%lu",
            (unsigned long)sequence);
        (void)snprintf(
            done_prefix, sizeof(done_prefix),
            "RK,ARM,PLATFORM_PICK,PRESELECT_DONE,SEQ,%lu",
            (unsigned long)sequence);
    } else {
        (void)snprintf(
            command, sizeof(command),
            "ARM,PLATFORM_PICK,START,SEQ,%lu,FIELD,%s,SLOT,%lu\r\n",
            (unsigned long)sequence, field_name, (unsigned long)slot);
        (void)snprintf(
            ack_prefix, sizeof(ack_prefix),
            "RK,ARM,PLATFORM_PICK,ACK,SEQ,%lu",
            (unsigned long)sequence);
        (void)snprintf(
            done_prefix, sizeof(done_prefix),
            "RK,ARM,PLATFORM_PICK,DONE,SEQ,%lu",
            (unsigned long)sequence);
    }
    (void)snprintf(
        error_prefix, sizeof(error_prefix),
        "RK,ARM,PLATFORM_PICK,ERR,SEQ,%lu",
        (unsigned long)sequence);
    (void)snprintf(
        status_command, sizeof(status_command),
        "ARM,PLATFORM_PICK,STATUS,SEQ,%lu\r\n",
        (unsigned long)sequence);

    (void)snprintf(
        log_line, sizeof(log_line),
        "H7,ARM,PLATFORM_PICK,%s_WAIT,SEQ=%lu,FIELD=%s,SLOT=%lu\r\n",
        preselect ? "PRESELECT" : "SLOT",
        (unsigned long)sequence, field_name, (unsigned long)slot);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

        if (rk_acknowledged &&
            (uint32_t)(now_ms - task_started_ms) >= RK_ARM_TASK_TIMEOUT_MS) {
            (void)snprintf(
                log_line, sizeof(log_line),
                "H7,ARM,PLATFORM_PICK,%s_TIMEOUT,SEQ=%lu,SLOT=%lu\r\n",
                preselect ? "PRESELECT" : "SLOT",
                (unsigned long)sequence, (unsigned long)slot);
            board_uart1_write(log_line);
            g_rk_last_task_bypassed = 1U;
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
        }
        if ((uint32_t)(now_ms - last_zero_ms) >= CONTROL_PERIOD_MS) {
            last_zero_ms = now_ms;
            if (!keep_chassis_stopped_for_arm_task()) {
                preserve_rc_or_set_motor_fault();
                return false;
            }
        }
        if (!rk_acknowledged &&
            (uint32_t)(now_ms - last_send_ms) >= RK_ARM_START_RETRY_MS) {
            last_send_ms = now_ms;
            board_usb_write(command);
            (void)snprintf(
                log_line, sizeof(log_line),
                "H7,ARM,PLATFORM_PICK,%s_SENT,SEQ=%lu,FIELD=%s,SLOT=%lu\r\n",
                preselect ? "PRESELECT" : "SLOT",
                (unsigned long)sequence, field_name, (unsigned long)slot);
            board_uart1_write_only(log_line);
        }
        if (rk_acknowledged &&
            (uint32_t)(now_ms - last_send_ms) >= RK_ARM_STATUS_PERIOD_MS) {
            last_send_ms = now_ms;
            board_usb_write(status_command);
        }

        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (i = 0U; i < read_len; ++i) {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    board_uart1_write_only("H7,USB,RX,");
                    board_uart1_write_only(line);
                    board_uart1_write_only("\r\n");
                    rk_arm_handle_line(line);
                    if (g_rk_arm_link_ready != 0U &&
                        ack_timeout_ms < RK_ARM_ACK_TIMEOUT_MS) {
                        ack_timeout_ms = RK_ARM_ACK_TIMEOUT_MS;
                    }
                    if (line_matches_token_prefix(line, ack_prefix)) {
                        if (!rk_acknowledged) {
                            task_started_ms = HAL_GetTick();
                        }
                        rk_acknowledged = true;
                        (void)snprintf(
                            log_line, sizeof(log_line),
                            "H7,ARM,PLATFORM_PICK,%s_ACK,SEQ=%lu,SLOT=%lu\r\n",
                            preselect ? "PRESELECT" : "SLOT",
                            (unsigned long)sequence, (unsigned long)slot);
                        board_uart1_write(log_line);
                    }
                    if (line_matches_token_prefix(line, done_prefix)) {
                        g_rk_last_task_bypassed = 0U;
                        (void)snprintf(
                            log_line, sizeof(log_line),
                            "H7,ARM,PLATFORM_PICK,%s_DONE,SEQ=%lu,SLOT=%lu\r\n",
                            preselect ? "PRESELECT" : "SLOT",
                            (unsigned long)sequence, (unsigned long)slot);
                        board_uart1_write(log_line);
                        return true;
                    }
                    if (line_matches_token_prefix(line, error_prefix)) {
                        (void)snprintf(
                            log_line, sizeof(log_line),
                            "H7,ARM,PLATFORM_PICK,%s_ERR,SEQ=%lu,SLOT=%lu\r\n",
                            preselect ? "PRESELECT" : "SLOT",
                            (unsigned long)sequence, (unsigned long)slot);
                        board_uart1_write(log_line);
                        g_fault_code = FAULT_ARM_REMOTE;
                        return false;
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write("H7,ERR,PLATFORM_PICK_LINE_TOO_LONG\r\n");
            }
        }
#if RK_ARM_WAIT_FOREVER_FOR_ACK == 0U
        now_ms = HAL_GetTick();
        if (!rk_acknowledged &&
            (uint32_t)(now_ms - started_ms) >= ack_timeout_ms) {
            (void)snprintf(
                log_line, sizeof(log_line),
                "H7,ARM,PLATFORM_PICK,%s_BYPASS_NO_RK,SEQ=%lu,SLOT=%lu\r\n",
                preselect ? "PRESELECT" : "SLOT",
                (unsigned long)sequence, (unsigned long)slot);
            board_uart1_write(log_line);
            g_rk_arm_link_ready = 0U;
            g_rk_last_task_bypassed = 1U;
#if RK_ARM_REQUIRED
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return true;
#endif
        }
#endif
        HAL_Delay(1U);
    }
}

static bool start_rk_arm_task(const char *task)
{
    uint8_t rx[64];
    char line[96];
    char log_line[96];
    char start_command[64];
    char ack_prefix[64];
    char busy_prefix[64];
    char error_prefix[64];
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    const uint32_t sequence = next_rk_task_sequence();
    uint32_t ack_timeout_ms = (g_rk_arm_link_ready != 0U) ?
                              RK_ARM_ACK_TIMEOUT_MS :
                              RK_ARM_PROBE_ACK_TIMEOUT_MS;
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";

    g_rk_last_task_bypassed = 0U;

    (void)snprintf(start_command, sizeof(start_command),
                    "ARM,%s,START,SEQ,%lu,FIELD,%s\r\n", task,
                    (unsigned long)sequence, field_name);
    (void)snprintf(ack_prefix, sizeof(ack_prefix),
                    "RK,ARM,%s,ACK,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(busy_prefix, sizeof(busy_prefix),
                    "RK,ARM,%s,BUSY,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(error_prefix, sizeof(error_prefix),
                    "RK,ARM,%s,ERR,SEQ,%lu", task, (unsigned long)sequence);
    (void)snprintf(log_line, sizeof(log_line),
                   "H7,ARM,%s,START_ASYNC,SEQ=%lu\r\n", task,
                   (unsigned long)sequence);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

        if ((uint32_t)(now_ms - last_zero_ms) >= CONTROL_PERIOD_MS) {
            last_zero_ms = now_ms;
            if (!keep_chassis_stopped_for_arm_task()) {
                preserve_rc_or_set_motor_fault();
                return false;
            }
        }
        if ((uint32_t)(now_ms - last_send_ms) >= RK_ARM_START_RETRY_MS) {
            last_send_ms = now_ms;
            board_usb_write(start_command);
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,START_SENT_ASYNC\r\n", task);
            board_uart1_write_only(log_line);
        }

        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (i = 0U; i < read_len; ++i) {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    board_uart1_write_only("H7,USB,RX,");
                    board_uart1_write_only(line);
                    board_uart1_write_only("\r\n");
                    rk_arm_handle_line(line);
                    if (g_rk_arm_link_ready != 0U &&
                        ack_timeout_ms < RK_ARM_ACK_TIMEOUT_MS) {
                        ack_timeout_ms = RK_ARM_ACK_TIMEOUT_MS;
                    }
                    if (line_matches_token_prefix(line, ack_prefix)) {
                        g_rk_async_task_sequence = sequence;
                        g_rk_last_task_bypassed = 0U;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,ACK_ASYNC,SEQ=%lu\r\n", task,
                                       (unsigned long)sequence);
                        board_uart1_write(log_line);
                        return true;
                    }
                    if (line_matches_token_prefix(line, error_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,REMOTE_ERROR_ASYNC,SEQ=%lu\r\n",
                                       task, (unsigned long)sequence);
                        board_uart1_write(log_line);
                        g_fault_code = FAULT_ARM_REMOTE;
                        return false;
                    }
                    if (line_matches_token_prefix(line, busy_prefix) &&
                        ack_timeout_ms < RK_ARM_BUSY_TIMEOUT_MS) {
                        ack_timeout_ms = RK_ARM_BUSY_TIMEOUT_MS;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,RK_BUSY_ASYNC,SEQ=%lu\r\n",
                                       task, (unsigned long)sequence);
                        board_uart1_write(log_line);
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write("H7,ERR,ARM_LINE_TOO_LONG\r\n");
            }
        }
#if RK_ARM_WAIT_FOREVER_FOR_ACK == 0U
        now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - started_ms) >= ack_timeout_ms) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,BYPASS_NO_RK_ASYNC,SEQ=%lu\r\n", task,
                           (unsigned long)sequence);
            board_uart1_write(log_line);
            g_rk_arm_link_ready = 0U;
            g_rk_last_task_bypassed = 1U;
#if RK_ARM_REQUIRED
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return false;
#endif
        }
#endif
        HAL_Delay(1U);
    }
}

static bool stop_rk_arm_task(const char *task)
{
    uint8_t rx[64];
    char line[96];
    char log_line[96];
    char stop_command[64];
    char done_prefix[64];
    char error_prefix[64];
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;

    if (g_rk_async_task_sequence == 0U) {
        board_uart1_write("H7,ARM,STOP_WITHOUT_ACTIVE_SEQUENCE\r\n");
        g_fault_code = FAULT_ARM_REMOTE;
        return false;
    }

    (void)snprintf(stop_command, sizeof(stop_command),
                    "ARM,%s,STOP,SEQ,%lu\r\n", task,
                    (unsigned long)g_rk_async_task_sequence);
    (void)snprintf(done_prefix, sizeof(done_prefix),
                    "RK,ARM,%s,DONE,SEQ,%lu", task,
                    (unsigned long)g_rk_async_task_sequence);
    (void)snprintf(error_prefix, sizeof(error_prefix),
                    "RK,ARM,%s,ERR,SEQ,%lu", task,
                    (unsigned long)g_rk_async_task_sequence);
    (void)snprintf(log_line, sizeof(log_line),
                   "H7,ARM,%s,STOP_WAIT,SEQ=%lu\r\n", task,
                   (unsigned long)g_rk_async_task_sequence);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

        if ((uint32_t)(now_ms - last_zero_ms) >= CONTROL_PERIOD_MS) {
            last_zero_ms = now_ms;
            if (!keep_chassis_stopped_for_arm_task()) {
                preserve_rc_or_set_motor_fault();
                return false;
            }
        }
        if ((uint32_t)(now_ms - last_send_ms) >= RK_ARM_START_RETRY_MS) {
            last_send_ms = now_ms;
            board_usb_write(stop_command);
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,STOP_SENT\r\n", task);
            board_uart1_write_only(log_line);
        }

        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (i = 0U; i < read_len; ++i) {
            char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    board_uart1_write_only("H7,USB,RX,");
                    board_uart1_write_only(line);
                    board_uart1_write_only("\r\n");
                    rk_arm_handle_line(line);
                    if (line_matches_token_prefix(line, done_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,DONE_STOP,SEQ=%lu\r\n", task,
                                       (unsigned long)g_rk_async_task_sequence);
                        board_uart1_write(log_line);
                        g_rk_async_task_sequence = 0U;
                        return true;
                    }
                    if (line_matches_token_prefix(line, error_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,REMOTE_ERROR_STOP,SEQ=%lu\r\n",
                                       task,
                                       (unsigned long)g_rk_async_task_sequence);
                        board_uart1_write(log_line);
                        g_fault_code = FAULT_ARM_REMOTE;
                        return false;
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write("H7,ERR,ARM_LINE_TOO_LONG\r\n");
            }
        }
        now_ms = HAL_GetTick();
        if ((uint32_t)(now_ms - started_ms) >= RK_ARM_STOP_TIMEOUT_MS) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,STOP_TIMEOUT,SEQ=%lu\r\n", task,
                           (unsigned long)g_rk_async_task_sequence);
            board_uart1_write(log_line);
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
        }
        HAL_Delay(1U);
    }
}

static bool run_zp_aux(uint32_t channel, uint32_t pulse, uint32_t time_ms,
                       uint8_t servo_id)
{
    uint8_t rx[96];
    char line[160];
    char command[160];
    char ack_prefix[64];
    char done_prefix[64];
    char error_prefix[64];
    char log_line[128];
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t ack_ms = 0U;
    uint32_t last_send_ms = started_ms - ROUTE_AUX_RETRY_MS;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;
    const uint32_t sequence = next_rk_task_sequence();
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";
    const bool valid_channel = channel == ROUTE_AUX_ZP_S12_CHANNEL ||
                               channel == ROUTE_AUX_ZP_S23_CHANNEL;
    const bool valid_servo = servo_id == ROUTE_AUX_ZP_ID3_SERVO_ID;

    if ((!valid_channel && !valid_servo) || (valid_channel && servo_id != 0U) ||
        (valid_servo && channel != 0U) || pulse < 500U || pulse > 2500U ||
        time_ms > 9999U) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    if (valid_channel) {
        (void)snprintf(
            command, sizeof(command),
            "ARM,AUX_ZP,SET,SEQ,%lu,FIELD,%s,CHANNEL,%lu,PULSE,%lu,TIME,%lu\r\n",
            (unsigned long)sequence, field_name, (unsigned long)channel,
            (unsigned long)pulse, (unsigned long)time_ms);
    } else {
        (void)snprintf(
            command, sizeof(command),
            "ARM,AUX_ZP,SET,SEQ,%lu,FIELD,%s,SERVO_ID,%u,PULSE,%lu,TIME,%lu\r\n",
            (unsigned long)sequence, field_name, (unsigned int)servo_id,
            (unsigned long)pulse, (unsigned long)time_ms);
    }
    (void)snprintf(ack_prefix, sizeof(ack_prefix),
                   "RK,AUX_ZP,ACK,SEQ,%lu", (unsigned long)sequence);
    (void)snprintf(done_prefix, sizeof(done_prefix),
                   "RK,AUX_ZP,DONE,SEQ,%lu", (unsigned long)sequence);
    (void)snprintf(error_prefix, sizeof(error_prefix),
                   "RK,AUX_ZP,ERR,SEQ,%lu", (unsigned long)sequence);
    (void)snprintf(log_line, sizeof(log_line),
                   "H7,AUX_ZP,WAIT,SEQ=%lu,TARGET=%s%lu,PULSE=%lu,TIME=%lu\r\n",
                   (unsigned long)sequence,
                   valid_channel ? "S" : "ID", valid_channel
                       ? (unsigned long)channel
                       : (unsigned long)servo_id,
                   (unsigned long)pulse, (unsigned long)time_ms);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t index;

        if ((uint32_t)(now_ms - last_zero_ms) >= CONTROL_PERIOD_MS) {
            last_zero_ms = now_ms;
            if (!keep_chassis_stopped_for_arm_task()) {
                preserve_rc_or_set_motor_fault();
                return false;
            }
        }
        if (ack_ms == 0U &&
            (uint32_t)(now_ms - last_send_ms) >= ROUTE_AUX_RETRY_MS) {
            last_send_ms = now_ms;
            board_usb_write(command);
        }
        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (index = 0U; index < read_len; ++index) {
            const char c = (char)rx[index];

            if (c == '\r' || c == '\n') {
                line[line_len] = '\0';
                if (line_len > 0U) {
                    board_uart1_write_only("H7,USB,RX,");
                    board_uart1_write_only(line);
                    board_uart1_write_only("\r\n");
                    if (line_matches_token_prefix(line, ack_prefix)) {
                        if (ack_ms == 0U) {
                            ack_ms = HAL_GetTick();
                            board_uart1_write_only("H7,AUX_ZP,ACK\r\n");
                        }
                    } else if (line_matches_token_prefix(line, done_prefix)) {
                        board_uart1_write("H7,AUX_ZP,DONE\r\n");
                        return true;
                    } else if (line_matches_token_prefix(line, error_prefix)) {
                        board_uart1_write("H7,AUX_ZP,REMOTE_ERROR\r\n");
                        g_fault_code = FAULT_ARM_REMOTE;
                        return false;
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                board_uart1_write("H7,ERR,AUX_ZP_LINE_TOO_LONG\r\n");
            }
        }
        now_ms = HAL_GetTick();
        if (ack_ms == 0U) {
            if ((uint32_t)(now_ms - started_ms) >= ROUTE_AUX_ACK_TIMEOUT_MS) {
                board_uart1_write("H7,AUX_ZP,ACK_TIMEOUT\r\n");
                g_fault_code = FAULT_ARM_TIMEOUT;
                return false;
            }
        } else if ((uint32_t)(now_ms - ack_ms) >= ROUTE_AUX_DONE_TIMEOUT_MS) {
            board_uart1_write("H7,AUX_ZP,DONE_TIMEOUT\r\n");
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
        }
        HAL_Delay(1U);
    }
}

static bool settle_translation_cross_track(float along_x, float along_y,
                                             float route_frame_heading_rad,
                                             float heading_target_rad,
                                             float heading_kp,
                                             float heading_kd,
                                             float *cross_track_m)
{
    float wheel_speed[4] = {0.0f};
    float measured_wheel_speed[4] = {0.0f};
    translation_velocity_observer_t velocity_observer = {0};
    const float cross_x = -along_y;
    const float cross_y = along_x;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    uint32_t settled_since_ms = 0U;
    const uint32_t started_ms = previous_ms;
    bool first_feedback_cycle = true;

    if (cross_track_m == NULL) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    g_command_speed_m_s = 0.0f;
    g_cross_track_m = *cross_track_m;
    g_cross_track_command_m_s = 0.0f;
    log_route_event(RUN_LOG_EVENT_CROSS_SETTLE_START);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        float dt;
        float actual_vx_m_s;
        float actual_vy_m_s;
        float actual_wz_rad_s;
        float heading_error_for_transform;
        float heading_cos;
        float heading_sin;
        float actual_route_vx_m_s;
        float actual_route_vy_m_s;
        float command_route_vx_m_s;
        float command_route_vy_m_s;
        float command_body_vx_m_s;
        float command_body_vy_m_s;

        if ((uint32_t)(now_ms - started_ms) >=
            TRANSLATION_CROSS_SETTLE_TIMEOUT_MS) {
            g_cross_track_command_m_s = 0.0f;
            log_route_event(RUN_LOG_EVENT_CROSS_SETTLE_TIMEOUT);
            return route_motor_send_zero_all();
        }
        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms += CONTROL_PERIOD_MS;
        if ((uint32_t)(now_ms - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
        }
        dt = (float)(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;
        dt = clampf(dt, 0.001f, 0.050f);

        update_imu(dt);
        if (!mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s,
                             &actual_wz_rad_s)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        (void)actual_wz_rad_s;

        /* Cross-track distance is expressed in the fixed route frame captured
         * when the translation segment began.  The commanded heading may
         * rotate through 180 degrees, so it must not rotate this frame. */
        heading_error_for_transform =
            g_yaw_rad - route_frame_heading_rad;
        heading_cos = cosf(heading_error_for_transform);
        heading_sin = sinf(heading_error_for_transform);
        actual_route_vx_m_s =
            heading_cos * actual_vx_m_s + heading_sin * actual_vy_m_s;
        actual_route_vy_m_s =
            -heading_sin * actual_vx_m_s + heading_cos * actual_vy_m_s;
        update_translation_velocity_observer(
            &velocity_observer, actual_route_vx_m_s, actual_route_vy_m_s,
            heading_cos, heading_sin, dt);
        g_actual_cross_speed_m_s =
            velocity_observer.route_vx_m_s * cross_x +
            velocity_observer.route_vy_m_s * cross_y;
        *cross_track_m +=
            g_actual_cross_speed_m_s * dt * DRIVE_DISTANCE_SCALE;
        g_cross_track_m = *cross_track_m;

        g_heading_correction_rad_s = clampf(
            -heading_kp * (g_yaw_rad - heading_target_rad) -
                heading_kd * g_gyro_z_rad_s,
            -TRANSLATION_SETTLE_HEADING_MAX_CORRECTION_RAD_S,
            TRANSLATION_SETTLE_HEADING_MAX_CORRECTION_RAD_S);
        g_cross_track_command_m_s = clampf(
            -TRANSLATION_CROSS_TRACK_KP * *cross_track_m -
                TRANSLATION_CROSS_TRACK_KD * g_actual_cross_speed_m_s,
            -TRANSLATION_CROSS_TRACK_SETTLE_MAX_M_S,
            TRANSLATION_CROSS_TRACK_SETTLE_MAX_M_S);

        command_route_vx_m_s = g_cross_track_command_m_s * cross_x;
        command_route_vy_m_s = g_cross_track_command_m_s * cross_y;
        command_body_vx_m_s =
            heading_cos * command_route_vx_m_s -
            heading_sin * command_route_vy_m_s;
        command_body_vy_m_s =
            heading_sin * command_route_vx_m_s +
            heading_cos * command_route_vy_m_s;

        if (!mecanum_inverse(&chassis, command_body_vx_m_s,
                             command_body_vy_m_s,
                             g_heading_correction_rad_s, wheel_speed)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        if (!route_motor_send_wheel_speeds(wheel_speed)) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        if (!route_motor_feedback_update_after_command(
                measured_wheel_speed, &first_feedback_cycle)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();

        /* A segment with an integrated turn must leave this settle only when
         * the final heading has actually converged; otherwise the residual
         * turn leaks into the next segment's heading hold. */
        if (fabsf(*cross_track_m) <= TRANSLATION_CROSS_TRACK_TOLERANCE_M &&
            fabsf(g_actual_cross_speed_m_s) <=
                TRANSLATION_CROSS_SPEED_TOLERANCE_M_S &&
            fabsf(g_yaw_rad - heading_target_rad) <=
                ROUTE_TURN_TOLERANCE_RAD &&
            fabsf(g_gyro_z_rad_s) <= ROUTE_TURN_RATE_TOLERANCE_RAD_S) {
            if (settled_since_ms == 0U) {
                settled_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - settled_since_ms) >=
                       TRANSLATION_CROSS_SETTLE_MS) {
                g_cross_track_command_m_s = 0.0f;
                log_route_event(RUN_LOG_EVENT_CROSS_SETTLE_DONE);
                return route_motor_send_zero_all();
            }
        } else {
            settled_since_ms = 0U;
        }
    }
}

static bool run_translation_profile_with_turn(float vx_direction,
                                              float vy_direction,
                                              float target_distance_m,
                                              float maximum_speed_m_s,
                                              float acceleration_m_s2,
                                              float heading_delta_rad)
{
    float wheel_speed[4] = {0.0f};
    float measured_wheel_speed[4] = {0.0f};
    translation_velocity_observer_t velocity_observer = {0};
    float actual_vx_m_s = 0.0f;
    float actual_vy_m_s = 0.0f;
    float actual_wz_rad_s = 0.0f;
    float segment_distance_m = 0.0f;
    float commanded_distance_m = 0.0f;
    float profile_speed_m_s = 0.0f;
    float along_speed_integral_m_s = 0.0f;
    float cross_track_m = 0.0f;
    const float direction_norm = sqrtf(vx_direction * vx_direction +
                                       vy_direction * vy_direction);
    float along_x;
    float along_y;
    float cross_x;
    float cross_y;
    const float route_frame_heading_rad = g_yaw_rad;
    const float start_heading_target_rad = g_route_heading_target_rad;
    const float final_heading_target_rad =
        start_heading_target_rad + heading_delta_rad;
    const bool is_strafe = fabsf(vy_direction) > fabsf(vx_direction);
    const float heading_kp = is_strafe ? STRAFE_HEADING_KP : HEADING_KP;
    const float heading_kd = is_strafe ? STRAFE_HEADING_KD : HEADING_KD;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    const uint32_t started_ms = previous_ms;
    uint32_t settled_since_ms = 0U;
    bool first_feedback_cycle = true;
    bool heading_settle_logged = false;

    if (direction_norm < 0.001f || maximum_speed_m_s <= 0.0f ||
        acceleration_m_s2 <= 0.0f) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }
    along_x = vx_direction / direction_norm;
    along_y = vy_direction / direction_norm;
    cross_x = -along_y;
    cross_y = along_x;

    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        float dt;
        float feedback_remaining;
        float command_remaining;
        float remaining;
        float stopping_speed;
        float desired_speed;
        float max_delta;
        float along_position_error_m;
        float along_speed_reference_m_s;
        float along_speed_error_m_s;
        float along_speed_command_m_s;
        float correction_limit_rad_s;
        float actual_translation_speed_m_s;
        float actual_cross_speed_m_s;
        float heading_target_rad;
        float heading_progress;
        float heading_smooth;
        float heading_error_for_transform;
        float heading_cos;
        float heading_sin;
        float actual_route_vx_m_s;
        float actual_route_vy_m_s;
        float cross_track_command_m_s;
        float command_route_vx_m_s;
        float command_route_vy_m_s;
        float command_body_vx_m_s;
        float command_body_vy_m_s;
        float heading_rate_feedforward_rad_s;
        bool translation_endpoint_done;
        bool heading_endpoint_done;
        bool segment_done;

        if ((uint32_t)(now_ms - started_ms) >= ROUTE_TRANSLATION_TIMEOUT_MS) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if (g_task2_test_active_sequence != 0U &&
            g_task2_test_stop_requested != 0U) {
            (void)route_motor_send_zero_all();
            g_command_speed_m_s = 0.0f;
            g_heading_correction_rad_s = 0.0f;
            return false;
        }
        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms += CONTROL_PERIOD_MS;
        if ((uint32_t)(now_ms - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
        }
        dt = (float)(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;
        dt = clampf(dt, 0.001f, 0.050f);

        heading_progress = clampf(commanded_distance_m / target_distance_m,
                                  0.0f, 1.0f);
        heading_smooth = heading_progress * heading_progress *
                         (3.0f - 2.0f * heading_progress);
        heading_target_rad = start_heading_target_rad +
                             heading_delta_rad * heading_smooth;
        g_route_heading_target_rad = heading_target_rad;

        update_imu(dt);
        if (!mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s,
                             &actual_wz_rad_s)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }

        /* Positive yaw is a right turn on this chassis.  Keep measured
         * translation in the segment-start route frame while the chassis
         * heading target progresses independently through the turn. */
        heading_error_for_transform =
            g_yaw_rad - route_frame_heading_rad;
        heading_cos = cosf(heading_error_for_transform);
        heading_sin = sinf(heading_error_for_transform);
        actual_route_vx_m_s =
            heading_cos * actual_vx_m_s + heading_sin * actual_vy_m_s;
        actual_route_vy_m_s =
            -heading_sin * actual_vx_m_s + heading_cos * actual_vy_m_s;
        update_translation_velocity_observer(
            &velocity_observer, actual_route_vx_m_s, actual_route_vy_m_s,
            heading_cos, heading_sin, dt);
        actual_translation_speed_m_s =
            velocity_observer.route_vx_m_s * along_x +
            velocity_observer.route_vy_m_s * along_y;
        actual_cross_speed_m_s =
            velocity_observer.route_vx_m_s * cross_x +
            velocity_observer.route_vy_m_s * cross_y;
        segment_distance_m +=
            actual_translation_speed_m_s * dt * DRIVE_DISTANCE_SCALE;
        g_estimated_distance_m +=
            fabsf(actual_translation_speed_m_s) * dt * DRIVE_DISTANCE_SCALE;
        cross_track_m += actual_cross_speed_m_s * dt * DRIVE_DISTANCE_SCALE;
        g_cross_track_m = cross_track_m;
        g_actual_cross_speed_m_s = actual_cross_speed_m_s;

        feedback_remaining = target_distance_m - segment_distance_m;
        if (feedback_remaining < 0.0f) {
            feedback_remaining = 0.0f;
        }
        command_remaining = target_distance_m - commanded_distance_m;
        if (command_remaining < 0.0f) {
            command_remaining = 0.0f;
        }
        remaining = feedback_remaining < command_remaining ?
                    feedback_remaining : command_remaining;
        stopping_speed = sqrtf(2.0f * acceleration_m_s2 * remaining);
        desired_speed = stopping_speed < maximum_speed_m_s ?
                        stopping_speed : maximum_speed_m_s;
        max_delta = acceleration_m_s2 * dt;
        if (profile_speed_m_s < desired_speed) {
            profile_speed_m_s += max_delta;
            if (profile_speed_m_s > desired_speed) {
                profile_speed_m_s = desired_speed;
            }
        } else {
            profile_speed_m_s -= max_delta;
            if (profile_speed_m_s < desired_speed) {
                profile_speed_m_s = desired_speed;
            }
        }
        commanded_distance_m += profile_speed_m_s * dt;
        if (commanded_distance_m > target_distance_m) {
            commanded_distance_m = target_distance_m;
        }

        heading_rate_feedforward_rad_s = 0.0f;
        if (fabsf(heading_delta_rad) > 0.0001f) {
            const float progress_rate =
                profile_speed_m_s / target_distance_m;
            heading_rate_feedforward_rad_s =
                heading_delta_rad * 6.0f * heading_progress *
                (1.0f - heading_progress) * progress_rate;
        }

        along_position_error_m = commanded_distance_m - segment_distance_m;
        along_speed_reference_m_s = profile_speed_m_s + clampf(
            ODOM_ALONG_POSITION_KP * along_position_error_m,
            -ODOM_ALONG_CORRECTION_MAX_M_S,
            ODOM_ALONG_CORRECTION_MAX_M_S);
        along_speed_reference_m_s = clampf(
            along_speed_reference_m_s,
            -ODOM_ALONG_CORRECTION_MAX_M_S, maximum_speed_m_s);

        along_speed_error_m_s =
            along_speed_reference_m_s - actual_translation_speed_m_s;
        along_speed_integral_m_s = clampf(
            along_speed_integral_m_s +
                ODOM_ALONG_SPEED_KI * along_speed_error_m_s * dt,
            -ODOM_ALONG_SPEED_INTEGRAL_MAX_M_S,
            ODOM_ALONG_SPEED_INTEGRAL_MAX_M_S);
        along_speed_command_m_s = clampf(
            along_speed_reference_m_s +
                ODOM_ALONG_SPEED_KP * along_speed_error_m_s +
                along_speed_integral_m_s,
            -ODOM_ALONG_CORRECTION_MAX_M_S, maximum_speed_m_s);
        g_command_speed_m_s = along_speed_command_m_s;

        if (fabsf(heading_delta_rad) > 0.0001f) {
            /* A combined translation/turn must still be able to finish its
             * heading after the distance profile decelerates to zero. The
             * old speed-proportional limit collapsed to zero at the endpoint,
             * leaving the final turn for the following settle loop. */
            correction_limit_rad_s = HEADING_MAX_CORRECTION_RAD_S;
        } else {
            correction_limit_rad_s =
                fabsf(along_speed_command_m_s) * HEADING_CORRECTION_SPEED_RATIO /
                (CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M);
            if (correction_limit_rad_s > HEADING_MAX_CORRECTION_RAD_S) {
                correction_limit_rad_s = HEADING_MAX_CORRECTION_RAD_S;
            }
        }
        g_heading_correction_rad_s = clampf(
            heading_rate_feedforward_rad_s -
                heading_kp * (g_yaw_rad - heading_target_rad) -
                heading_kd * g_gyro_z_rad_s,
            -correction_limit_rad_s,
            correction_limit_rad_s);

        cross_track_command_m_s = clampf(
            -TRANSLATION_CROSS_TRACK_KP * cross_track_m -
                TRANSLATION_CROSS_TRACK_KD * actual_cross_speed_m_s,
            -TRANSLATION_CROSS_TRACK_MAX_M_S,
            TRANSLATION_CROSS_TRACK_MAX_M_S);
        g_cross_track_command_m_s = cross_track_command_m_s;
        command_route_vx_m_s =
            along_speed_command_m_s * along_x +
            cross_track_command_m_s * cross_x;
        command_route_vy_m_s =
            along_speed_command_m_s * along_y +
            cross_track_command_m_s * cross_y;

        translation_endpoint_done =
            fabsf(target_distance_m - segment_distance_m) <=
                ODOM_ALONG_POSITION_TOLERANCE_M &&
            fabsf(actual_translation_speed_m_s) <=
                ODOM_ALONG_SPEED_TOLERANCE_M_S;
        heading_endpoint_done =
            fabsf(final_heading_target_rad - g_yaw_rad) <=
                ROUTE_TURN_TOLERANCE_RAD &&
            fabsf(g_gyro_z_rad_s) <= ROUTE_TURN_RATE_TOLERANCE_RAD_S;
        segment_done = translation_endpoint_done &&
                       (fabsf(heading_delta_rad) <= 0.0001f ||
                        heading_endpoint_done);

        /* Freeze translation at its endpoint while the requested heading is
         * still pending. The angular command continues in this same segment,
         * so the following route segment cannot inherit a partial turn. */
        if (translation_endpoint_done && !heading_endpoint_done &&
            fabsf(heading_delta_rad) > 0.0001f) {
            command_route_vx_m_s = 0.0f;
            command_route_vy_m_s = 0.0f;
            g_command_speed_m_s = 0.0f;
            g_cross_track_command_m_s = 0.0f;
            if (!heading_settle_logged) {
                heading_settle_logged = true;
                board_uart1_write(
                    "H7,ROUTE,TRANSLATION_WITH_TURN,TRANSLATION_DONE,"
                    "HEADING_SETTLE\r\n");
            }
        }

        /* Convert the fixed-frame command back to the current body frame. */
        command_body_vx_m_s =
            heading_cos * command_route_vx_m_s -
            heading_sin * command_route_vy_m_s;
        command_body_vy_m_s =
            heading_sin * command_route_vx_m_s +
            heading_cos * command_route_vy_m_s;

        if (!mecanum_inverse(&chassis, command_body_vx_m_s,
                             command_body_vy_m_s,
                             g_heading_correction_rad_s, wheel_speed)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        if (!route_motor_send_wheel_speeds(wheel_speed)) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        if (!route_motor_feedback_update_after_command(
                measured_wheel_speed, &first_feedback_cycle)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();

        if (segment_done) {
            if (settled_since_ms == 0U) {
                settled_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - settled_since_ms) >=
                       ODOM_ALONG_SETTLE_MS) {
                break;
            }
        } else {
            settled_since_ms = 0U;
        }
    }
    g_command_speed_m_s = 0.0f;
    g_route_heading_target_rad = final_heading_target_rad;
    return settle_translation_cross_track(along_x, along_y,
                                          route_frame_heading_rad,
                                          final_heading_target_rad, heading_kp,
                                          heading_kd, &cross_track_m);
}

static bool run_translation_profile(float vx_direction, float vy_direction,
                                    float target_distance_m,
                                    float maximum_speed_m_s,
                                    float acceleration_m_s2)
{
    return run_translation_profile_with_turn(
        vx_direction, vy_direction, target_distance_m, maximum_speed_m_s,
        acceleration_m_s2, 0.0f);
}

#define DISC_ARC_LENGTH_SAMPLES 64U

typedef struct {
    float cumulative_m[DISC_ARC_LENGTH_SAMPLES + 1U];
    float total_m;
} disc_arc_length_table_t;

static void disc_arc_point(float u, float lateral_sign, float *x, float *y)
{
    const float one_minus_u = 1.0f - u;
    const float p1_x =
        ROUTE_FORWARD_SIGN * ROUTE_DISC_ARC_CONTROL1_FORWARD_M;
    const float p1_y =
        lateral_sign * ROUTE_DISC_ARC_CONTROL1_LATERAL_M;
    const float p2_x =
        ROUTE_FORWARD_SIGN * ROUTE_DISC_ARC_CONTROL2_FORWARD_M;
    const float p2_y =
        lateral_sign * ROUTE_DISC_ARC_CONTROL2_LATERAL_M;
    const float p3_x = ROUTE_FORWARD_SIGN * ROUTE_FORWARD_DISTANCE_M;
    const float p3_y = lateral_sign * ROUTE_STRAFE_DISTANCE_M;
    const float one_minus_u_sq = one_minus_u * one_minus_u;
    const float u_sq = u * u;

    *x = 3.0f * one_minus_u_sq * u * p1_x +
         3.0f * one_minus_u * u_sq * p2_x + u_sq * u * p3_x;
    *y = 3.0f * one_minus_u_sq * u * p1_y +
         3.0f * one_minus_u * u_sq * p2_y + u_sq * u * p3_y;
}

static void disc_arc_derivative(float u, float lateral_sign, float *dx, float *dy)
{
    const float one_minus_u = 1.0f - u;
    const float p1_x =
        ROUTE_FORWARD_SIGN * ROUTE_DISC_ARC_CONTROL1_FORWARD_M;
    const float p1_y =
        lateral_sign * ROUTE_DISC_ARC_CONTROL1_LATERAL_M;
    const float p2_x =
        ROUTE_FORWARD_SIGN * ROUTE_DISC_ARC_CONTROL2_FORWARD_M;
    const float p2_y =
        lateral_sign * ROUTE_DISC_ARC_CONTROL2_LATERAL_M;
    const float p3_x = ROUTE_FORWARD_SIGN * ROUTE_FORWARD_DISTANCE_M;
    const float p3_y = lateral_sign * ROUTE_STRAFE_DISTANCE_M;

    *dx = 3.0f * one_minus_u * one_minus_u * p1_x +
          6.0f * one_minus_u * u * (p2_x - p1_x) +
          3.0f * u * u * (p3_x - p2_x);
    *dy = 3.0f * one_minus_u * one_minus_u * p1_y +
          6.0f * one_minus_u * u * (p2_y - p1_y) +
          3.0f * u * u * (p3_y - p2_y);
}

static void disc_arc_build_length_table(float lateral_sign,
                                        disc_arc_length_table_t *table)
{
    float previous_x = 0.0f;
    float previous_y = 0.0f;
    uint32_t i;

    table->cumulative_m[0] = 0.0f;
    for (i = 1U; i <= DISC_ARC_LENGTH_SAMPLES; ++i) {
        float x;
        float y;
        const float u = (float)i / (float)DISC_ARC_LENGTH_SAMPLES;

        disc_arc_point(u, lateral_sign, &x, &y);
        table->cumulative_m[i] = table->cumulative_m[i - 1U] +
            sqrtf((x - previous_x) * (x - previous_x) +
                  (y - previous_y) * (y - previous_y));
        previous_x = x;
        previous_y = y;
    }
    table->total_m = table->cumulative_m[DISC_ARC_LENGTH_SAMPLES];
}

static float disc_arc_parameter_at_distance(
    const disc_arc_length_table_t *table, float distance_m)
{
    uint32_t i;

    if (distance_m <= 0.0f || table->total_m <= 0.0f) {
        return 0.0f;
    }
    if (distance_m >= table->total_m) {
        return 1.0f;
    }
    for (i = 1U; i <= DISC_ARC_LENGTH_SAMPLES; ++i) {
        if (table->cumulative_m[i] >= distance_m) {
            const float segment_start_m = table->cumulative_m[i - 1U];
            const float segment_length_m =
                table->cumulative_m[i] - segment_start_m;
            const float fraction = segment_length_m > 0.000001f
                ? (distance_m - segment_start_m) / segment_length_m
                : 0.0f;

            return ((float)(i - 1U) + fraction) /
                   (float)DISC_ARC_LENGTH_SAMPLES;
        }
    }
    return 1.0f;
}

static float smoothstep01(float value)
{
    const float u = clampf(value, 0.0f, 1.0f);

    return u * u * (3.0f - 2.0f * u);
}

bool route_controller_wait_for_disc_prep_high(void)
{
    const uint32_t wait_started_ms = HAL_GetTick();

    (void)service_disc_prep_high_during_arc();
    if (g_rk_disc_prep_high_ack != 0U) {
        board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_ACKED_DURING_ARC\r\n");
        return true;
    }

    board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_WAIT_AFTER_ARC\r\n");
    while ((uint32_t)(HAL_GetTick() - wait_started_ms) <
           RK_ARM_ACK_TIMEOUT_MS) {
        if (service_disc_prep_high_during_arc()) {
            board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_ACKED_AFTER_ARC\r\n");
            return true;
        }
        if (!keep_chassis_stopped_for_arm_task()) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        HAL_Delay(1U);
    }

    board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_TIMEOUT\r\n");
    g_fault_code = FAULT_ARM_TIMEOUT;
    return false;
}

bool route_controller_run_task2_prep_high(void)
{
    /* DISC_CATCH completion has already homed the arm. This transaction is
     * intentionally sent after the task-two transfer and only raises the
     * task-two arm joints; RK keeps ZP ID5 at CATCHER_HOME_TICK. Temporarily
     * reopen the RK receive path so the sequence-aware ACK is consumed. */
    g_first_arm_station_reached = 0U;
    g_rk_disc_prep_high_requested = 0U;
    g_rk_disc_prep_high_ack = 0U;
    g_rk_disc_prep_high_last_send_ms = 0U;
    board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_AFTER_TASK2_TRANSFER\r\n");
    if (!route_controller_wait_for_disc_prep_high()) {
        return false;
    }
    g_first_arm_station_reached = 1U;
    board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_TASK2_CONFIRMED\r\n");
    return true;
}

static bool run_disc_arc_entry(float lateral_sign, float turn_sign)
{
    disc_arc_length_table_t arc_length_table;
    float wheel_speed[4] = {0.0f};
    float measured_wheel_speed[4] = {0.0f};
    translation_velocity_observer_t velocity_observer = {0};
    float actual_vx_m_s = 0.0f;
    float actual_vy_m_s = 0.0f;
    float actual_wz_rad_s = 0.0f;
    float route_x_m = 0.0f;
    float route_y_m = 0.0f;
    float actual_arc_distance_m = 0.0f;
    float commanded_distance_m = 0.0f;
    float profile_speed_m_s = 0.0f;
    float along_speed_integral_m_s = 0.0f;
    float turn_command_rad_s = 0.0f;
    const float start_heading_rad = g_route_heading_target_rad;
    const float target_turn_rad = turn_sign * ROUTE_TURN_ANGLE_RAD *
                                  ROUTE_GYRO_TURN_SCALE;
    const float target_heading_rad = start_heading_rad + target_turn_rad;
    const float final_x_m = ROUTE_FORWARD_SIGN * ROUTE_FORWARD_DISTANCE_M;
    const float final_y_m = lateral_sign * ROUTE_STRAFE_DISTANCE_M;
    float arc_length_m;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    const uint32_t started_ms = previous_ms;
    uint32_t settled_since_ms = 0U;
    bool first_feedback_cycle = true;
    bool heading_settle_logged = false;

    disc_arc_build_length_table(lateral_sign, &arc_length_table);
    arc_length_m = arc_length_table.total_m;
    if (arc_length_m <= 0.001f || ROUTE_DISC_ARC_MAX_SPEED_M_S <= 0.0f ||
        ROUTE_DISC_ARC_ACCEL_M_S2 <= 0.0f) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;
    (void)service_disc_prep_high_during_arc();
    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        float dt;
        float feedback_remaining;
        float command_remaining;
        float remaining;
        float stopping_speed;
        float desired_speed;
        float max_delta;
        float u;
        float path_progress;
        float desired_x;
        float desired_y;
        float derivative_x;
        float derivative_y;
        float derivative_norm;
        float tangent_x;
        float tangent_y;
        float normal_x;
        float normal_y;
        float heading_error_for_transform;
        float heading_cos;
        float heading_sin;
        float actual_route_vx_m_s;
        float actual_route_vy_m_s;
        float odom_route_vx_m_s;
        float odom_route_vy_m_s;
        float actual_along_speed_m_s;
        float actual_cross_speed_m_s;
        float along_position_error_m;
        float along_speed_reference_m_s;
        float along_speed_error_m_s;
        float along_speed_command_m_s;
        float cross_error_m;
        float cross_track_command_m_s;
        float endpoint_error_x_m;
        float endpoint_error_y_m;
        float endpoint_forward_remaining_m;
        float endpoint_forward_command_m;
        float endpoint_distance_m;
        bool endpoint_capture_active;
        float command_route_vx_m_s;
        float command_route_vy_m_s;
        float command_body_vx_m_s;
        float command_body_vy_m_s;
        float smooth_heading;
        float smooth_heading_derivative;
        float target_heading_now_rad;
        float heading_feedforward_rad_s;
        float desired_turn_rad_s;
        float max_turn_delta_rad_s;
        float heading_error_rad;
        float final_heading_error_rad;
        float endpoint_done_distance_m;
        float actual_route_speed_m_s;
        bool translation_endpoint_done;
        bool heading_endpoint_done;
        bool segment_done;

        (void)service_disc_prep_high_during_arc();

        if ((uint32_t)(now_ms - started_ms) >= ROUTE_DISC_ARC_TIMEOUT_MS) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms += CONTROL_PERIOD_MS;
        if ((uint32_t)(now_ms - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
        }
        dt = (float)(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;
        dt = clampf(dt, 0.001f, 0.050f);

        update_imu(dt);
        if (!mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s,
                             &actual_wz_rad_s)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }

        heading_error_for_transform = g_yaw_rad - start_heading_rad;
        heading_cos = cosf(heading_error_for_transform);
        heading_sin = sinf(heading_error_for_transform);
        actual_route_vx_m_s =
            heading_cos * actual_vx_m_s + heading_sin * actual_vy_m_s;
        actual_route_vy_m_s =
            -heading_sin * actual_vx_m_s + heading_cos * actual_vy_m_s;
        update_translation_velocity_observer(
            &velocity_observer, actual_route_vx_m_s, actual_route_vy_m_s,
            heading_cos, heading_sin, dt);
        actual_route_vx_m_s = velocity_observer.route_vx_m_s;
        actual_route_vy_m_s = velocity_observer.route_vy_m_s;
        odom_route_vx_m_s =
            actual_route_vx_m_s * ROUTE_DISC_ARC_ODOM_FORWARD_SCALE;
        odom_route_vy_m_s =
            actual_route_vy_m_s * ROUTE_DISC_ARC_ODOM_LATERAL_SCALE;
        route_x_m += odom_route_vx_m_s * dt * DRIVE_DISTANCE_SCALE;
        route_y_m += odom_route_vy_m_s * dt * DRIVE_DISTANCE_SCALE;

        path_progress =
            clampf(commanded_distance_m / arc_length_m, 0.0f, 1.0f);
        u = disc_arc_parameter_at_distance(&arc_length_table,
                                           commanded_distance_m);
        disc_arc_point(u, lateral_sign, &desired_x, &desired_y);
        disc_arc_derivative(u, lateral_sign, &derivative_x, &derivative_y);
        derivative_norm = sqrtf(derivative_x * derivative_x +
                                derivative_y * derivative_y);
        if (derivative_norm <= 0.001f) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        tangent_x = derivative_x / derivative_norm;
        tangent_y = derivative_y / derivative_norm;
        normal_x = -tangent_y;
        normal_y = tangent_x;
        actual_along_speed_m_s =
            odom_route_vx_m_s * tangent_x + odom_route_vy_m_s * tangent_y;
        actual_cross_speed_m_s =
            odom_route_vx_m_s * normal_x + odom_route_vy_m_s * normal_y;
        actual_arc_distance_m +=
            actual_along_speed_m_s * dt * DRIVE_DISTANCE_SCALE;
        g_estimated_distance_m +=
            fabsf(actual_along_speed_m_s) * dt * DRIVE_DISTANCE_SCALE;

        feedback_remaining = arc_length_m - actual_arc_distance_m;
        if (feedback_remaining < 0.0f) {
            feedback_remaining = 0.0f;
        }
        command_remaining = arc_length_m - commanded_distance_m;
        if (command_remaining < 0.0f) {
            command_remaining = 0.0f;
        }
        remaining = feedback_remaining < command_remaining ?
                    feedback_remaining : command_remaining;
        stopping_speed = sqrtf(2.0f * ROUTE_DISC_ARC_ACCEL_M_S2 * remaining);
        desired_speed = stopping_speed < ROUTE_DISC_ARC_MAX_SPEED_M_S ?
                        stopping_speed : ROUTE_DISC_ARC_MAX_SPEED_M_S;
        max_delta = ROUTE_DISC_ARC_ACCEL_M_S2 * dt;
        if (profile_speed_m_s < desired_speed) {
            profile_speed_m_s += max_delta;
            if (profile_speed_m_s > desired_speed) {
                profile_speed_m_s = desired_speed;
            }
        } else {
            profile_speed_m_s -= max_delta;
            if (profile_speed_m_s < desired_speed) {
                profile_speed_m_s = desired_speed;
            }
        }
        commanded_distance_m += profile_speed_m_s * dt;
        if (commanded_distance_m > arc_length_m) {
            commanded_distance_m = arc_length_m;
        }

        along_position_error_m = commanded_distance_m - actual_arc_distance_m;
        along_speed_reference_m_s = profile_speed_m_s + clampf(
            ODOM_ALONG_POSITION_KP * along_position_error_m,
            -ODOM_ALONG_CORRECTION_MAX_M_S,
            ODOM_ALONG_CORRECTION_MAX_M_S);
        along_speed_reference_m_s = clampf(
            along_speed_reference_m_s,
            -ODOM_ALONG_CORRECTION_MAX_M_S, ROUTE_DISC_ARC_MAX_SPEED_M_S);
        along_speed_error_m_s =
            along_speed_reference_m_s - actual_along_speed_m_s;
        along_speed_integral_m_s = clampf(
            along_speed_integral_m_s +
                ODOM_ALONG_SPEED_KI * along_speed_error_m_s * dt,
            -ODOM_ALONG_SPEED_INTEGRAL_MAX_M_S,
            ODOM_ALONG_SPEED_INTEGRAL_MAX_M_S);
        along_speed_command_m_s = clampf(
            along_speed_reference_m_s +
                ODOM_ALONG_SPEED_KP * along_speed_error_m_s +
                along_speed_integral_m_s,
            -ODOM_ALONG_CORRECTION_MAX_M_S, ROUTE_DISC_ARC_MAX_SPEED_M_S);
        g_command_speed_m_s = along_speed_command_m_s;

        cross_error_m =
            (route_x_m - desired_x) * normal_x +
            (route_y_m - desired_y) * normal_y;
        cross_track_command_m_s = clampf(
            -TRANSLATION_CROSS_TRACK_KP * cross_error_m -
                TRANSLATION_CROSS_TRACK_KD * actual_cross_speed_m_s,
            -TRANSLATION_CROSS_TRACK_MAX_M_S,
            TRANSLATION_CROSS_TRACK_MAX_M_S);
        g_cross_track_m = cross_error_m;
        g_cross_track_command_m_s = cross_track_command_m_s;
        g_actual_cross_speed_m_s = actual_cross_speed_m_s;

        endpoint_error_x_m = final_x_m - route_x_m;
        endpoint_error_y_m = final_y_m - route_y_m;
        endpoint_forward_remaining_m = endpoint_error_x_m * ROUTE_FORWARD_SIGN;
        endpoint_forward_command_m =
            endpoint_forward_remaining_m > 0.0f ? endpoint_forward_remaining_m
                                                 : 0.0f;
        endpoint_distance_m =
            sqrtf(endpoint_forward_command_m * endpoint_forward_command_m +
                  endpoint_error_y_m * endpoint_error_y_m);
        endpoint_capture_active =
            path_progress >= ROUTE_DISC_ARC_ENDPOINT_CAPTURE_U;
        if (endpoint_capture_active) {
            float endpoint_speed_m_s;
            float endpoint_command_norm_m_s;

            if (endpoint_distance_m <=
                    ROUTE_DISC_ARC_ENDPOINT_TOLERANCE_M) {
                endpoint_speed_m_s = 0.0f;
                command_route_vx_m_s = 0.0f;
                command_route_vy_m_s = 0.0f;
            } else if (endpoint_distance_m > 0.001f) {
                /* In endpoint capture, converge to the measured station side
                 * offset.  Lateral correction may go either direction; forward
                 * correction only fills remaining forward distance so a small
                 * x overshoot does not cause a visible back-up near the disc. */
                command_route_vx_m_s =
                    ROUTE_FORWARD_SIGN * ROUTE_DISC_ARC_ENDPOINT_KP *
                    endpoint_forward_command_m;
                command_route_vy_m_s =
                    ROUTE_DISC_ARC_ENDPOINT_KP * endpoint_error_y_m;
                endpoint_command_norm_m_s =
                    sqrtf(command_route_vx_m_s * command_route_vx_m_s +
                          command_route_vy_m_s * command_route_vy_m_s);
                if (endpoint_command_norm_m_s >
                    ROUTE_DISC_ARC_ENDPOINT_MAX_SPEED_M_S) {
                    const float scale =
                        ROUTE_DISC_ARC_ENDPOINT_MAX_SPEED_M_S /
                        endpoint_command_norm_m_s;
                    command_route_vx_m_s *= scale;
                    command_route_vy_m_s *= scale;
                    endpoint_command_norm_m_s =
                        ROUTE_DISC_ARC_ENDPOINT_MAX_SPEED_M_S;
                } else if (
                    endpoint_command_norm_m_s <
                    ROUTE_DISC_ARC_ENDPOINT_MIN_SPEED_M_S) {
                    const float scale =
                        ROUTE_DISC_ARC_ENDPOINT_MIN_SPEED_M_S /
                        endpoint_command_norm_m_s;
                    command_route_vx_m_s *= scale;
                    command_route_vy_m_s *= scale;
                    endpoint_command_norm_m_s =
                        ROUTE_DISC_ARC_ENDPOINT_MIN_SPEED_M_S;
                }
                endpoint_speed_m_s = endpoint_command_norm_m_s;
            } else {
                endpoint_speed_m_s = 0.0f;
                command_route_vx_m_s = 0.0f;
                command_route_vy_m_s = 0.0f;
            }
            g_command_speed_m_s = endpoint_speed_m_s;
        } else {
            command_route_vx_m_s =
                along_speed_command_m_s * tangent_x +
                cross_track_command_m_s * normal_x;
            command_route_vy_m_s =
                along_speed_command_m_s * tangent_y +
                cross_track_command_m_s * normal_y;
        }
        command_body_vx_m_s =
            heading_cos * command_route_vx_m_s -
            heading_sin * command_route_vy_m_s;
        command_body_vy_m_s =
            heading_sin * command_route_vx_m_s +
            heading_cos * command_route_vy_m_s;

        smooth_heading = smoothstep01(path_progress);
        smooth_heading_derivative =
            6.0f * path_progress * (1.0f - path_progress);
        target_heading_now_rad =
            start_heading_rad + target_turn_rad * smooth_heading;
        heading_feedforward_rad_s =
            target_turn_rad * smooth_heading_derivative *
            (along_speed_command_m_s / arc_length_m);
        heading_error_rad = target_heading_now_rad - g_yaw_rad;
        desired_turn_rad_s = clampf(
            heading_feedforward_rad_s +
                ROUTE_TURN_KP * heading_error_rad -
                ROUTE_TURN_KD * g_gyro_z_rad_s,
            -ROUTE_TURN_MAX_SPEED_RAD_S,
            ROUTE_TURN_MAX_SPEED_RAD_S);
        max_turn_delta_rad_s = ROUTE_TURN_ACCEL_RAD_S2 * dt;
        turn_command_rad_s = clampf(
            desired_turn_rad_s,
            turn_command_rad_s - max_turn_delta_rad_s,
            turn_command_rad_s + max_turn_delta_rad_s);
        g_heading_correction_rad_s = turn_command_rad_s;
        g_route_heading_target_rad = target_heading_now_rad;
        final_heading_error_rad = target_heading_rad - g_yaw_rad;
        endpoint_done_distance_m =
            endpoint_capture_active ? endpoint_distance_m :
            fabsf(arc_length_m - actual_arc_distance_m);
        actual_route_speed_m_s = sqrtf(
            actual_route_vx_m_s * actual_route_vx_m_s +
            actual_route_vy_m_s * actual_route_vy_m_s);
        if (endpoint_capture_active) {
            /* At the endpoint, use the measured station error and actual
             * speed directly. The old combination of forward-only limits
             * could reject a valid endpoint after the command had already
             * reduced translation to zero, causing a timeout at the marker.
             * Capture only activates from 85% path progress onward, so a
             * command-progress gate is redundant and starves this done
             * condition when the scaled odometry crawls the profile. */
            translation_endpoint_done =
                endpoint_done_distance_m <=
                    ROUTE_DISC_ARC_DONE_DISTANCE_TOLERANCE_M &&
                actual_route_speed_m_s <=
                    ROUTE_DISC_ARC_DONE_SPEED_TOLERANCE_M_S;
        } else {
            translation_endpoint_done =
                fabsf(arc_length_m - actual_arc_distance_m) <=
                    ODOM_ALONG_POSITION_TOLERANCE_M &&
                fabsf(actual_along_speed_m_s) <=
                    ODOM_ALONG_SPEED_TOLERANCE_M_S;
        }
        heading_endpoint_done =
            fabsf(final_heading_error_rad) <= ROUTE_TURN_TOLERANCE_RAD &&
            fabsf(g_gyro_z_rad_s) <= ROUTE_TURN_RATE_TOLERANCE_RAD_S;
        segment_done = translation_endpoint_done && heading_endpoint_done;

        /* Stop translating as soon as the endpoint is stable, but keep the
         * angular controller active until the requested 90/180-degree turn
         * is actually complete inside this same arc segment. */
        if (translation_endpoint_done && !heading_endpoint_done) {
            command_route_vx_m_s = 0.0f;
            command_route_vy_m_s = 0.0f;
            g_command_speed_m_s = 0.0f;
            g_cross_track_command_m_s = 0.0f;
            if (!heading_settle_logged) {
                heading_settle_logged = true;
                board_uart1_write(
                    "H7,ROUTE,DISC_ARC,TRANSLATION_DONE,HEADING_SETTLE\r\n");
            }
        }

        if (!mecanum_inverse(&chassis, command_body_vx_m_s,
                             command_body_vy_m_s,
                             turn_command_rad_s, wheel_speed)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        if (!route_motor_send_wheel_speeds(wheel_speed)) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        if (!route_motor_feedback_update_after_command(
                measured_wheel_speed, &first_feedback_cycle)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        lcd_display_update();

        if (segment_done) {
            if (settled_since_ms == 0U) {
                settled_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - settled_since_ms) >=
                       ODOM_ALONG_SETTLE_MS) {
                g_route_heading_target_rad = target_heading_rad;
                g_command_speed_m_s = 0.0f;
                g_heading_correction_rad_s = 0.0f;
                return route_motor_send_zero_all();
            } else {
                settled_since_ms = 0U;
            }
        }
    }
}

static bool run_disc_visual_alignment_at_speed(float forward_speed_m_s,
                                               long reference_y10,
                                               long tolerance_y10,
                                               float acceleration_m_s2,
                                               route_white_line_phase_t phase)
{
    uint8_t rx[64];
    char line[128];
    char query[96];
    char log_line[160];
    uint32_t line_len = 0U;
    const uint32_t sequence = next_rk_task_sequence();
    const uint32_t started_ms = HAL_GetTick();
    uint32_t previous_ms = started_ms;
    uint32_t last_control_ms = started_ms - CONTROL_PERIOD_MS;
    uint32_t last_query_ms = started_ms - ROUTE_DISC_LINE_QUERY_PERIOD_MS;
    uint32_t last_measurement_ms = 0U;
    float error_angle_deg = 0.0f;
    bool measurement_valid = false;
    long y10_history[ROUTE_DISC_LINE_Y10_FILTER_SAMPLES] = {0};
    uint32_t y10_history_count = 0U;
    long last_accepted_y10 = 0L;
    long filtered_y10 = 0L;
    bool have_accepted_y10 = false;
    bool filtered_y10_valid = false;
    uint32_t angle_stable_samples = 0U;
    bool angle_aligned = false;
    uint32_t reference_stable_samples = 0U;
    uint32_t not_found_samples = 0U;
    bool reverse_search_logged = false;
    bool forward_search_logged = false;
    float measured_wheel_speed[4] = {0.0f};
    float wheel_speed[4] = {0.0f};
    bool first_feedback_cycle = true;
    float commanded_forward_speed_m_s = 0.0f;

    /* Only task-one post-arc and task-two post-secondary-shift entry may
     * emit white-line queries. Keep the restriction beside the emitter. */
    if (phase != ROUTE_WHITE_LINE_PHASE_TASK1_AFTER_ARC &&
        phase != ROUTE_WHITE_LINE_PHASE_TASK2_AFTER_SHIFT) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    (void)snprintf(query, sizeof(query),
                   "VISION,WHITE_LINE,QUERY,SEQ,%lu,PHASE,%s\r\n",
                   (unsigned long)sequence,
                   phase == ROUTE_WHITE_LINE_PHASE_TASK1_AFTER_ARC
                       ? "TASK1_AFTER_ARC"
                       : "TASK2_AFTER_SECONDARY_SHIFT");
    (void)snprintf(log_line, sizeof(log_line),
                   "H7,VISION,WHITE_LINE,START,SEQ=%lu,PHASE=%s,REF_Y10=%ld,REF_A100=%d\r\n",
                   (unsigned long)sequence,
                   phase == ROUTE_WHITE_LINE_PHASE_TASK1_AFTER_ARC
                       ? "TASK1_AFTER_ARC"
                       : "TASK2_AFTER_SECONDARY_SHIFT",
                   reference_y10,
                   ROUTE_DISC_LINE_REFERENCE_A100);
    board_uart1_write(log_line);

    if (forward_speed_m_s <= 0.0f || acceleration_m_s2 <= 0.0f ||
        tolerance_y10 < 0L) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    /* During the standalone task-two white-line phase this function is the
     * sole CDC reader. Motor-command helpers must not run the task-two frame
     * parser concurrently and consume RK white-line replies. */
    g_task2_test_white_line_active =
        (phase == ROUTE_WHITE_LINE_PHASE_TASK2_AFTER_SHIFT &&
         g_task2_test_active_sequence != 0U) ? 1U : 0U;

    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

        if ((uint32_t)(now_ms - started_ms) >= ROUTE_DISC_LINE_TIMEOUT_MS) {
            (void)route_motor_send_zero_all();
            g_command_speed_m_s = 0.0f;
            g_heading_correction_rad_s = 0.0f;
            g_fault_code = FAULT_WHITE_LINE_NOT_FOUND;
            board_uart1_write("H7,VISION,WHITE_LINE,TIMEOUT,FAULT\r\n");
            g_task2_test_white_line_active = 0U;
            return false;
        }

        if ((uint32_t)(now_ms - last_query_ms) >=
            ROUTE_DISC_LINE_QUERY_PERIOD_MS) {
            last_query_ms = now_ms;
            board_usb_write(query);
        }

        read_len = CDC_Read_HS(rx, sizeof(rx));
        for (i = 0U; i < read_len; ++i) {
            const char c = (char)rx[i];

            if (c == '\r' || c == '\n') {
                unsigned long response_sequence;
                long y10;
                long a100;
                long frame_width;
                long frame_height;

                line[line_len] = '\0';
                if (line_len > 0U) {
                    rk_arm_handle_line(line);
                    if (sscanf(
                            line,
                            "RK,VISION,WHITE_LINE,FOUND,SEQ,%lu,Y10,%ld,A100,%ld,W,%ld,H,%ld",
                            &response_sequence, &y10, &a100,
                            &frame_width, &frame_height) == 5 &&
                        response_sequence == (unsigned long)sequence &&
                        frame_width == ROUTE_DISC_LINE_FRAME_WIDTH &&
                        frame_height == ROUTE_DISC_LINE_FRAME_HEIGHT) {
                        bool accept_y10 = y10 >= 0L && y10 <=
                                          (long)frame_height * 10L;
                        /* Reject physically implausible jumps, then use a
                         * three-sample median for the crossing decision. */
                        if (accept_y10 && have_accepted_y10 &&
                            labs(y10 - last_accepted_y10) >
                                ROUTE_DISC_LINE_Y10_MAX_JUMP) {
                            accept_y10 = false;
                        }
                        if (accept_y10) {
                            last_accepted_y10 = y10;
                            have_accepted_y10 = true;
                            y10_history[y10_history_count %
                                        ROUTE_DISC_LINE_Y10_FILTER_SAMPLES] = y10;
                            ++y10_history_count;
                            if (y10_history_count >=
                                ROUTE_DISC_LINE_Y10_FILTER_SAMPLES) {
                                long sorted[ROUTE_DISC_LINE_Y10_FILTER_SAMPLES];
                                uint32_t j;
                                for (j = 0U; j < ROUTE_DISC_LINE_Y10_FILTER_SAMPLES;
                                     ++j) {
                                    sorted[j] = y10_history[j];
                                }
                                for (j = 0U;
                                     j + 1U < ROUTE_DISC_LINE_Y10_FILTER_SAMPLES;
                                     ++j) {
                                    uint32_t k;
                                    for (k = j + 1U;
                                         k < ROUTE_DISC_LINE_Y10_FILTER_SAMPLES;
                                         ++k) {
                                        if (sorted[k] < sorted[j]) {
                                            long swap = sorted[j];
                                            sorted[j] = sorted[k];
                                            sorted[k] = swap;
                                        }
                                    }
                                }
                                filtered_y10 = sorted[
                                    ROUTE_DISC_LINE_Y10_FILTER_SAMPLES / 2U];
                                filtered_y10_valid = true;
                            }

                            error_angle_deg = ((float)a100 -
                                               ROUTE_DISC_LINE_REFERENCE_A100) *
                                              0.01f;
                            if (fabsf(error_angle_deg) <=
                                ROUTE_DISC_LINE_ANGLE_ALIGN_TOLERANCE_DEG) {
                                if (angle_stable_samples <
                                    ROUTE_DISC_LINE_ANGLE_STABLE_SAMPLES) {
                                    ++angle_stable_samples;
                                }
                            } else {
                                angle_stable_samples = 0U;
                            }
                            angle_aligned = angle_stable_samples >=
                                            ROUTE_DISC_LINE_ANGLE_STABLE_SAMPLES;
                            last_measurement_ms = now_ms;
                            measurement_valid = true;
                            (void)snprintf(
                                log_line, sizeof(log_line),
                                "H7,VISION,WHITE_LINE,TRACK,Y10=%ld,FY10=%ld,DA=%.2f,ALIGN=%u\r\n",
                                y10, filtered_y10, (double)error_angle_deg,
                                angle_aligned ? 1U : 0U);
                            board_uart1_write_only(log_line);
                            if (y10_history_count >=
                                    ROUTE_DISC_LINE_Y10_FILTER_SAMPLES &&
                                angle_aligned &&
                                 filtered_y10_valid &&
                                 labs(filtered_y10 - reference_y10) <=
                                     tolerance_y10) {
                                if (reference_stable_samples <
                                    ROUTE_DISC_LINE_REFERENCE_STABLE_SAMPLES) {
                                    ++reference_stable_samples;
                                }
                            } else {
                                reference_stable_samples = 0U;
                            }
                            if (reference_stable_samples >=
                                ROUTE_DISC_LINE_REFERENCE_STABLE_SAMPLES) {
                                g_route_heading_target_rad = g_yaw_rad;
                                board_uart1_write(
                                    "H7,VISION,WHITE_LINE,CROSSED,CONTINUE\r\n");
                                g_task2_test_white_line_active = 0U;
                                return true;
                            }
                        } else {
                            board_uart1_write_only(
                                "H7,VISION,WHITE_LINE,REJECT_Y10_JUMP\r\n");
                        }
                    } else if (sscanf(
                                   line,
                                   "RK,VISION,WHITE_LINE,NOT_FOUND,SEQ,%lu",
                                   &response_sequence) == 1 &&
                               response_sequence ==
                                   (unsigned long)sequence) {
                        /* A single camera miss is expected while the chassis
                         * is moving. Keep the current forward command and,
                         * if available, the last fresh heading correction.
                         * The overall search timeout remains the only failure
                         * path for a line that never appears. */
                        ++not_found_samples;
                        if ((not_found_samples % 5U) == 1U) {
                            board_uart1_write_only(
                                "H7,VISION,WHITE_LINE,MISS,CONTINUE\r\n");
                        }
                    }
                }
                line_len = 0U;
            } else if (line_len + 1U < sizeof(line)) {
                line[line_len++] = c;
            } else {
                line_len = 0U;
                measurement_valid = false;
                board_uart1_write("H7,ERR,WHITE_LINE_RESPONSE_TOO_LONG\r\n");
            }
        }

        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms = now_ms;
        {
            float dt = (float)(now_ms - previous_ms) * 0.001f;
            /* Before a line is acquired, search backward for five seconds,
             * then search forward. Once acquired, only a fresh line result
             * is allowed to select the signed correction direction. */
            float desired_forward_speed_m_s = 0.0f;
            float command_vx_m_s;
            float command_vy_m_s = 0.0f;
            float command_wz_rad_s = 0.0f;

            previous_ms = now_ms;
            dt = clampf(dt, 0.001f, 0.050f);
            update_imu(dt);

            if (measurement_valid &&
                (uint32_t)(now_ms - last_measurement_ms) <=
                    ROUTE_DISC_LINE_STALE_MS) {
                if (fabsf(error_angle_deg) >
                    ROUTE_DISC_LINE_ANGLE_DEADBAND_DEG) {
                    command_wz_rad_s = clampf(
                        ROUTE_RIGHT_TURN_SIGN *
                            ROUTE_DISC_LINE_TURN_KP_RAD_S_PER_DEG *
                            error_angle_deg -
                            ROUTE_DISC_LINE_TURN_KD * g_gyro_z_rad_s,
                        -ROUTE_DISC_LINE_MAX_TURN_RAD_S,
                        ROUTE_DISC_LINE_MAX_TURN_RAD_S);
                }
                if (filtered_y10_valid) {
                    const long y10_error = filtered_y10 - reference_y10;
                    if (y10_error > tolerance_y10) {
                        desired_forward_speed_m_s = -forward_speed_m_s;
                    } else if (y10_error < -tolerance_y10) {
                        desired_forward_speed_m_s = forward_speed_m_s;
                    } else {
                        desired_forward_speed_m_s = 0.0f;
                    }
                }
            } else {
                measurement_valid = false;
                if (!filtered_y10_valid) {
                    if ((uint32_t)(now_ms - started_ms) <
                        ROUTE_DISC_LINE_REVERSE_SEARCH_MS) {
                        desired_forward_speed_m_s = -forward_speed_m_s;
                        if (!reverse_search_logged) {
                            reverse_search_logged = true;
                            board_uart1_write_only(
                                "H7,VISION,WHITE_LINE,SEARCH,DIR=REVERSE,SPEED=0.08,T=3000ms\r\n");
                        }
                    } else {
                        desired_forward_speed_m_s = forward_speed_m_s;
                        if (!forward_search_logged) {
                            forward_search_logged = true;
                            board_uart1_write_only(
                                "H7,VISION,WHITE_LINE,SEARCH,DIR=FORWARD,SPEED=0.08\r\n");
                        }
                    }
                }
            }

            /* Slew the signed command so a direction change passes smoothly
             * through zero instead of reversing abruptly. */
            {
                const float max_speed_delta = acceleration_m_s2 * dt;
                const float speed_delta =
                    desired_forward_speed_m_s - commanded_forward_speed_m_s;
                if (speed_delta > max_speed_delta) {
                    commanded_forward_speed_m_s += max_speed_delta;
                } else if (speed_delta < -max_speed_delta) {
                    commanded_forward_speed_m_s -= max_speed_delta;
                } else {
                    commanded_forward_speed_m_s = desired_forward_speed_m_s;
                }
            }
            command_vx_m_s = ROUTE_FORWARD_SIGN * commanded_forward_speed_m_s;

            g_command_speed_m_s = sqrtf(
                commanded_forward_speed_m_s * commanded_forward_speed_m_s +
                command_vy_m_s * command_vy_m_s);
            g_heading_correction_rad_s = command_wz_rad_s;
            if (!mecanum_inverse(&chassis, command_vx_m_s, command_vy_m_s,
                                 command_wz_rad_s, wheel_speed)) {
                g_fault_code = FAULT_KINEMATICS;
                g_task2_test_white_line_active = 0U;
                return false;
            }
            if (!route_motor_send_wheel_speeds(wheel_speed)) {
                preserve_rc_or_set_motor_fault();
                g_task2_test_white_line_active = 0U;
                return false;
            }
            if (!route_motor_feedback_update_after_command(
                    measured_wheel_speed, &first_feedback_cycle)) {
                g_fault_code = FAULT_MOTOR_COMMAND;
                g_task2_test_white_line_active = 0U;
                return false;
            }
            lcd_display_update();
        }
    }
}

static bool run_translation(float vx_direction, float vy_direction,
                            float target_distance_m)
{
    return run_translation_profile(vx_direction, vy_direction,
                                   target_distance_m,
                                   ROUTE_TRANSLATION_SPEED_M_S,
                                   ROUTE_TRANSLATION_ACCEL_M_S2);
}

static bool run_timed_forward(float speed_m_s, uint32_t duration_ms)
{
    float measured_wheel_speed[4] = {0.0f};
    float wheel_speed[4] = {0.0f};
    float actual_vx_m_s = 0.0f;
    float actual_vy_m_s = 0.0f;
    float actual_wz_rad_s = 0.0f;
    const float heading_target_rad = g_route_heading_target_rad;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms - CONTROL_PERIOD_MS;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    const uint32_t started_ms = previous_ms;
    bool first_feedback_cycle = true;

    if (speed_m_s <= 0.0f || duration_ms == 0U) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    g_command_speed_m_s = speed_m_s;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;

    while ((uint32_t)(HAL_GetTick() - started_ms) < duration_ms) {
        const uint32_t now_ms = HAL_GetTick();
        float dt;
        float command_wz_rad_s;
        float correction_limit_rad_s;

        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms = now_ms;
        dt = clampf((float)(now_ms - previous_ms) * 0.001f, 0.001f, 0.050f);
        previous_ms = now_ms;

        update_imu(dt);
        if (!mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s,
                             &actual_wz_rad_s)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        (void)actual_vy_m_s;
        (void)actual_wz_rad_s;
        g_estimated_distance_m += fabsf(actual_vx_m_s) * dt * DRIVE_DISTANCE_SCALE;

        correction_limit_rad_s =
            speed_m_s * HEADING_CORRECTION_SPEED_RATIO /
            (CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M);
        if (correction_limit_rad_s > HEADING_MAX_CORRECTION_RAD_S) {
            correction_limit_rad_s = HEADING_MAX_CORRECTION_RAD_S;
        }
        command_wz_rad_s = clampf(
            -HEADING_KP * (g_yaw_rad - heading_target_rad) -
                HEADING_KD * g_gyro_z_rad_s,
            -correction_limit_rad_s, correction_limit_rad_s);
        g_heading_correction_rad_s = command_wz_rad_s;
        if (!mecanum_inverse(&chassis,
                             ROUTE_FORWARD_SIGN * speed_m_s, 0.0f,
                             command_wz_rad_s, wheel_speed)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        if (!route_motor_send_wheel_speeds(wheel_speed)) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        if (!route_motor_feedback_update_after_command(
                measured_wheel_speed, &first_feedback_cycle)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        lcd_display_update();
    }

    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    return route_motor_send_zero_all();
}

static bool run_relative_turn(float angle_rad)
{
    float wheel_speed[4] = {0.0f};
    float turn_command_rad_s = 0.0f;
    float route_yaw_rad = g_yaw_rad;
    const float target_yaw_rad = g_route_heading_target_rad + angle_rad;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    uint32_t settled_since_ms = 0U;
    const uint32_t started_ms = previous_ms;
    float measured_wheel_speed[4] = {0.0f};
    bool first_feedback_cycle = true;

    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;
    g_route_heading_target_rad = target_yaw_rad;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        float dt;
        float error_rad;
        float desired_rad_s;
        float max_delta_rad_s;

        if ((uint32_t)(now_ms - started_ms) >= ROUTE_TURN_TIMEOUT_MS) {
            g_fault_code = FAULT_TURN_TIMEOUT;
            return false;
        }
        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms += CONTROL_PERIOD_MS;
        if ((uint32_t)(now_ms - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
        }
        dt = (float)(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;
        dt = clampf(dt, 0.001f, 0.050f);

        update_imu(dt);
        error_rad = target_yaw_rad - route_yaw_rad;
        desired_rad_s = clampf(ROUTE_TURN_KP * error_rad -
                                   ROUTE_TURN_KD * g_gyro_z_rad_s,
                               -ROUTE_TURN_MAX_SPEED_RAD_S,
                               ROUTE_TURN_MAX_SPEED_RAD_S);
        max_delta_rad_s = ROUTE_TURN_ACCEL_RAD_S2 * dt;
        turn_command_rad_s = clampf(desired_rad_s,
                                    turn_command_rad_s - max_delta_rad_s,
                                    turn_command_rad_s + max_delta_rad_s);
        g_heading_correction_rad_s = turn_command_rad_s;

        if (!mecanum_inverse(&chassis, 0.0f, 0.0f,
                             turn_command_rad_s, wheel_speed)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        if (!route_motor_send_wheel_speeds(wheel_speed)) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        if (!route_motor_feedback_update_after_command(
                measured_wheel_speed, &first_feedback_cycle)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        route_yaw_rad = g_yaw_rad;
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();

        if (fabsf(error_rad) <= ROUTE_TURN_TOLERANCE_RAD &&
            fabsf(g_gyro_z_rad_s) <= ROUTE_TURN_RATE_TOLERANCE_RAD_S) {
            if (settled_since_ms == 0U) {
                settled_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - settled_since_ms) >=
                       ROUTE_TURN_SETTLE_MS) {
                return route_motor_send_zero_all();
            }
        } else {
            settled_since_ms = 0U;
        }
    }
}

static bool run_front_center_orbit(float angle_rad, float center_distance_m)
{
    float wheel_speed[4] = {0.0f};
    float measured_wheel_speed[4] = {0.0f};
    float actual_vx_m_s = 0.0f;
    float actual_vy_m_s = 0.0f;
    float actual_wz_rad_s = 0.0f;
    float turn_command_rad_s = 0.0f;
    const float turn_sign = angle_rad < 0.0f ? -1.0f : 1.0f;
    const float target_angle_rad = fabsf(angle_rad);
    float previous_yaw_rad = g_yaw_rad;
    float accumulated_angle_rad = 0.0f;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    uint32_t settled_since_ms = 0U;
    const uint32_t started_ms = previous_ms;
    bool first_feedback_cycle = true;
    const uint8_t task3_test =
        g_task3_test_active_sequence != 0U ? 1U : 0U;
    const uint32_t timeout_ms = task3_test != 0U
                                    ? ROUTE_TASK3_TEST_ORBIT_TIMEOUT_MS
                                    : ROUTE_FRONT_CENTER_ORBIT_TIMEOUT_MS;
    const float max_speed_rad_s = task3_test != 0U
                                      ? ROUTE_TASK3_TEST_ORBIT_MAX_SPEED_RAD_S
                                      : ROUTE_ORBIT_MAX_SPEED_RAD_S;
    const float acceleration_rad_s2 = task3_test != 0U
                                         ? ROUTE_TASK3_TEST_ORBIT_ACCEL_RAD_S2
                                         : ROUTE_ORBIT_ACCEL_RAD_S2;
    const float slow_speed_rad_s = task3_test != 0U
                                       ? ROUTE_TASK3_TEST_ORBIT_SLOW_SPEED_RAD_S
                                       : max_speed_rad_s;
    const float speed_ramp_rad_s2 = task3_test != 0U
                                        ? ROUTE_TASK3_TEST_ORBIT_SPEED_RAMP_RAD_S2
                                        : 0.0f;
    float orbit_speed_limit_rad_s = max_speed_rad_s;

    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;
    g_route_heading_target_rad = g_yaw_rad;

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        float dt;
        float yaw_delta_rad;
        float remaining_angle_rad;
        float stopping_speed_rad_s;
        float desired_speed_rad_s;
        float max_delta_rad_s;
        float orbit_vy_m_s;
        float requested_speed_limit_rad_s;

        if ((uint32_t)(now_ms - started_ms) >= timeout_ms) {
            g_fault_code = FAULT_TURN_TIMEOUT;
            return false;
        }
        if (task3_test != 0U) {
            const int pause_action =
                task3_test_service_orbit_pause(g_task3_test_active_sequence);

            if (pause_action == 0) {
                g_route_heading_target_rad = g_yaw_rad;
                return true;
            }
            if (pause_action < 0) {
                return false;
            }
            if (pause_action == 2) {
                /* The first command after RESUME must be emitted before the
                 * normal fresh-feedback gate is reinstated. */
                first_feedback_cycle = true;
            }
        }
        if ((uint32_t)(now_ms - last_control_ms) < CONTROL_PERIOD_MS) {
            HAL_Delay(1U);
            continue;
        }
        last_control_ms += CONTROL_PERIOD_MS;
        if ((uint32_t)(now_ms - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
        }
        dt = (float)(now_ms - previous_ms) * 0.001f;
        previous_ms = now_ms;
        dt = clampf(dt, 0.001f, 0.050f);

        update_imu(dt);
        if (!mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s,
                             &actual_wz_rad_s)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }

        /* Count the signed, unwrapped gyro delta.  A final-heading error is
         * unsuitable for a full circle because 0 and 2*pi represent the same
         * heading and can make the route finish early or hunt at the end. */
        yaw_delta_rad = g_yaw_rad - previous_yaw_rad;
        previous_yaw_rad = g_yaw_rad;
        if (fabsf(yaw_delta_rad) <= 0.25f) {
            accumulated_angle_rad += turn_sign * yaw_delta_rad;
            if (accumulated_angle_rad < 0.0f) {
                accumulated_angle_rad = 0.0f;
            }
        }
        remaining_angle_rad = target_angle_rad - accumulated_angle_rad;
        if (remaining_angle_rad < 0.0f) {
            remaining_angle_rad = 0.0f;
        }
        requested_speed_limit_rad_s =
            task3_test != 0U && g_task3_test_slow_requested != 0U
                ? slow_speed_rad_s
                : max_speed_rad_s;
        if (task3_test != 0U) {
            const float speed_limit_delta = speed_ramp_rad_s2 * dt;

            if (orbit_speed_limit_rad_s < requested_speed_limit_rad_s) {
                orbit_speed_limit_rad_s = fminf(
                    requested_speed_limit_rad_s,
                    orbit_speed_limit_rad_s + speed_limit_delta);
            } else if (orbit_speed_limit_rad_s > requested_speed_limit_rad_s) {
                orbit_speed_limit_rad_s = fmaxf(
                    requested_speed_limit_rad_s,
                    orbit_speed_limit_rad_s - speed_limit_delta);
            }
        } else {
            orbit_speed_limit_rad_s = max_speed_rad_s;
        }
        stopping_speed_rad_s = sqrtf(2.0f * acceleration_rad_s2 *
                                     remaining_angle_rad);
        desired_speed_rad_s = stopping_speed_rad_s < orbit_speed_limit_rad_s
                                  ? stopping_speed_rad_s
                                  : orbit_speed_limit_rad_s;
        max_delta_rad_s = acceleration_rad_s2 * dt;
        turn_command_rad_s = clampf(turn_sign * desired_speed_rad_s,
                                    turn_command_rad_s - max_delta_rad_s,
                                    turn_command_rad_s + max_delta_rad_s);
        orbit_vy_m_s = -center_distance_m * turn_command_rad_s;

        g_command_speed_m_s = fabsf(orbit_vy_m_s);
        g_heading_correction_rad_s = turn_command_rad_s;

        if (!mecanum_inverse(&chassis, 0.0f, orbit_vy_m_s,
                             turn_command_rad_s, wheel_speed)) {
            g_fault_code = FAULT_KINEMATICS;
            return false;
        }
        if (!route_motor_send_wheel_speeds(wheel_speed)) {
            preserve_rc_or_set_motor_fault();
            return false;
        }
        if (!route_motor_feedback_update_after_command(
                measured_wheel_speed, &first_feedback_cycle)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        g_estimated_distance_m +=
            fabsf(actual_wz_rad_s) * center_distance_m * dt * DRIVE_DISTANCE_SCALE;
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();

        if (accumulated_angle_rad >= target_angle_rad -
                ROUTE_TURN_TOLERANCE_RAD &&
            fabsf(g_gyro_z_rad_s) <= ROUTE_TURN_RATE_TOLERANCE_RAD_S) {
            if (settled_since_ms == 0U) {
                settled_since_ms = now_ms;
            } else if ((uint32_t)(now_ms - settled_since_ms) >=
                       ROUTE_TURN_SETTLE_MS) {
                g_route_heading_target_rad = g_yaw_rad;
                return route_motor_send_zero_all();
            }
        } else {
            settled_since_ms = 0U;
        }
    }
}

void route_controller_init(void)
{
    board_init();
    lcd_display_init();
    rc_override_init();
}

void route_controller_wait_for_start(void)
{
#if ROUTE_WAIT_USER_KEY_ON_BOOT
    if (g_start_confirmed_from_fault != 0U) {
        g_start_confirmed_from_fault = 0U;
        lcd_display_set_start_status("RUN");
    } else {
        board_clear_selected_field();
        wait_for_user_start_key();
    }
#endif
}

/* A motor command must be sent before its feedback is evaluated.  Some motor
 * firmware starts publishing only after that first command, so the first
 * post-command sample is allowed to be missing.  All later samples keep the
 * normal three-fresh-channel closed-loop requirement. */
static bool route_motor_feedback_update_after_command(
    float wheel_rad_s[4], bool *allow_missing_once)
{
    const bool feedback_valid = motor_feedback_update(HAL_GetTick(), wheel_rad_s);

    if (feedback_valid) {
        if (allow_missing_once != NULL) {
            *allow_missing_once = false;
        }
        return true;
    }

    if (allow_missing_once != NULL && *allow_missing_once) {
        *allow_missing_once = false;
        if (wheel_rad_s != NULL) {
            memset(wheel_rad_s, 0, 4U * sizeof(wheel_rad_s[0]));
        }
        board_uart1_write("H7,MOTOR,FEEDBACK_PRIME_BYPASS\r\n");
        return true;
    }

    board_uart1_write("H7,MOTOR,FEEDBACK_INSUFFICIENT,required=3\r\n");
#if ROUTE_REQUIRE_MOTOR_FEEDBACK
    return false;
#else
    if (wheel_rad_s != NULL) {
        memset(wheel_rad_s, 0, 4U * sizeof(wheel_rad_s[0]));
    }
    return true;
#endif
}

bool route_controller_take_task2_test(uint8_t *is_red, uint32_t *sequence,
                                      char *letter1, char *letter2,
                                      size_t letter_capacity)
{
    if (is_red == NULL || sequence == NULL || letter1 == NULL ||
        letter2 == NULL || letter_capacity < 2U ||
        g_task2_test_pending == 0U) {
        return false;
    }

    *is_red = g_task2_test_is_red;
    *sequence = g_task2_test_sequence;
    letter1[0] = g_task2_test_letter1;
    letter2[0] = g_task2_test_letter2;
    letter1[1] = '\0';
    letter2[1] = '\0';
    g_task2_test_pending = 0U;
    g_task2_test_active_sequence = *sequence;
    g_task2_test_stop_requested = 0U;
    return true;
}

uint32_t route_controller_task2_test_step(void)
{
    return g_task2_test_step;
}

bool route_controller_wait_for_task2_test_next(uint32_t sequence)
{
    for (;;) {
        (void)service_task2_test_command();
        rc_override_service();
        lcd_display_update();

        if (g_task2_test_active_sequence != sequence) {
            return false;
        }
        if (g_task2_test_stop_requested != 0U) {
            task2_test_send_status("STOPPED", sequence);
            g_task2_test_active_sequence = 0U;
            g_task2_test_pending = 0U;
            g_task2_test_stop_requested = 0U;
            g_fault_code = FAULT_NONE;
            return false;
        }
        if (g_task2_test_pending != 0U &&
            g_task2_test_sequence == sequence &&
            g_task2_test_step >= 1U && g_task2_test_step <= 7U) {
            return true;
        }
        HAL_Delay(10U);
    }
}

static bool run_disc_visual_alignment(void)
{
    return run_disc_visual_alignment_at_speed(
        ROUTE_DISC_LINE_FORWARD_SPEED_M_S,
        ROUTE_DISC_LINE_REFERENCE_Y10,
        ROUTE_DISC_LINE_REFERENCE_TOLERANCE_Y10,
        ROUTE_DISC_LINE_ACCEL_M_S2,
        ROUTE_WHITE_LINE_PHASE_TASK1_AFTER_ARC);
}

void route_controller_set_field(uint8_t is_red)
{
    g_route_field_is_red = is_red != 0U ? 1U : 0U;
}

void route_controller_reset_run_context(void)
{
    g_fault_code = FAULT_NONE;
    g_rk_last_task_bypassed = 0U;
    g_rk_last_task_soft_timed_out = 0U;
    g_first_arm_station_reached = 0U;
    g_rk_arm_link_ready = 0U;
    g_rk_disc_prep_high_ack = 0U;
    g_rk_disc_prep_high_requested = 0U;
    g_rk_disc_prep_high_last_send_ms = 0U;
    g_rk_reset_pending = 0U;
    g_rk_async_task_sequence = 0U;
    g_rk_pretask_line_len = 0U;
    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_estimated_distance_m = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;
    run_log_reset();
    if (g_task2_test_active_sequence == 0U &&
        g_task3_test_active_sequence == 0U) {
        request_rk_arm_reset();
    }
}

void route_controller_begin_pretask_sync(void)
{
    g_rk_pretask_last_sync_ms = HAL_GetTick() - RK_ARM_PRETASK_SYNC_PERIOD_MS;
    board_uart1_write(g_route_field_is_red != 0U
                          ? "H7,ARM,PRETASK_SYNC_ACTIVE,FIELD=RED\r\n"
                          : "H7,ARM,PRETASK_SYNC_ACTIVE,FIELD=BLUE\r\n");
}

bool route_controller_wait_for_rk_reset_before_route(void)
{
    const uint32_t started_ms = HAL_GetTick();

    board_uart1_write("H7,ARM,WAIT_RESET_BEFORE_ROUTE\r\n");
    while (g_rk_reset_pending != 0U &&
           (uint32_t)(HAL_GetTick() - started_ms) <
               RK_ARM_RESET_BEFORE_ROUTE_TIMEOUT_MS) {
        service_rk_link_before_first_station();
        lcd_display_update();
        HAL_Delay(10U);
    }
    if (g_rk_reset_pending == 0U) {
        board_uart1_write("H7,ARM,RESET_CONFIRMED_BEFORE_ROUTE\r\n");
        return true;
    }
    board_uart1_write("H7,ARM,RESET_BYPASS_NO_RK_BEFORE_ROUTE\r\n");
    return false;
}

void route_controller_mark_first_arm_station(void)
{
    g_first_arm_station_reached = 1U;
    board_uart1_write("H7,ARM,PRETASK_SYNC_STOP_AT_DISC\r\n");
}

void route_controller_request_rk_reset(void)
{
    request_rk_arm_reset();
}

void route_controller_service_rk_link(void)
{
    service_rk_link_before_first_station();
}

void route_controller_start_disc_prep_high_async(void)
{
    (void)service_disc_prep_high_during_arc();
    board_uart1_write("H7,ARM,DISC_CATCH,PREP_HIGH_STARTED_BEFORE_ARC\r\n");
}

void route_controller_reset_pose(void)
{
    g_yaw_rad = 0.0f;
    g_route_heading_target_rad = 0.0f;
    g_estimated_distance_m = 0.0f;
}

void route_controller_set_heading_target(float heading_rad)
{
    g_route_heading_target_rad = heading_rad;
}

uint8_t route_controller_last_arm_task_bypassed(void)
{
    return g_rk_last_task_bypassed;
}

uint8_t route_controller_last_arm_task_soft_timed_out(void)
{
    return g_rk_last_task_soft_timed_out;
}

bool route_controller_wait_for_can_startup(void)
{
    return wait_for_can_startup();
}

bool route_controller_calibrate_gyro(void)
{
    return calibrate_gyro();
}

bool route_controller_enable_motors(void)
{
    return enable_motors();
}

void route_controller_hold_zero(uint32_t duration_ms)
{
    hold_zero(duration_ms);
}

void route_controller_enter_fault_wait_restart(uint32_t code)
{
    enter_fault_wait_restart(code);
}

void route_controller_log_event(uint32_t event)
{
    log_route_event(event);
}

bool route_controller_run_translation_profile(float vx_direction,
                                               float vy_direction,
                                               float target_distance_m,
                                               float maximum_speed_m_s,
                                               float acceleration_m_s2)
{
    return run_translation_profile(vx_direction, vy_direction,
                                   target_distance_m, maximum_speed_m_s,
                                   acceleration_m_s2);
}

bool route_controller_run_timed_forward(float speed_m_s, uint32_t duration_ms)
{
    return run_timed_forward(speed_m_s, duration_ms);
}

bool route_controller_run_translation(float vx_direction, float vy_direction,
                                      float target_distance_m)
{
    return run_translation(vx_direction, vy_direction, target_distance_m);
}

bool route_controller_run_translation_with_turn(float vx_direction,
                                                float vy_direction,
                                                float target_distance_m,
                                                float maximum_speed_m_s,
                                                float acceleration_m_s2,
                                                float heading_delta_rad)
{
    return run_translation_profile_with_turn(
        vx_direction, vy_direction, target_distance_m, maximum_speed_m_s,
        acceleration_m_s2, heading_delta_rad);
}

bool route_controller_run_task2_test(uint32_t sequence, const char *letter1,
                                     const char *letter2)
{
    const float lateral_sign = g_task2_test_is_red != 0U
                                   ? -ROUTE_RIGHT_STRAFE_SIGN
                                   : ROUTE_RIGHT_STRAFE_SIGN;
    bool moved;

    if (sequence == 0U || letter1 == NULL || letter2 == NULL ||
        g_task2_test_active_sequence != sequence || letter1[0] == '\0' ||
        letter2[0] == '\0') {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    task2_test_send_status("RUNNING", sequence);
    if (g_task2_test_step == 0U) {
        /* The standalone App starts in the task-two area. Move laterally to
         * the platform line first; the formal 1.5/1.7 m + 180-degree task-two
         * entry route remains in main.c. */
        g_run_state = g_task2_test_is_red != 0U
                          ? RUN_PLATFORM_SHIFT_LEFT
                          : RUN_PLATFORM_SHIFT_RIGHT;
        board_uart1_write(
            g_task2_test_is_red != 0U
                ? "H7,TEST,TASK2,INITIAL_SHIFT,LEFT=400mm\r\n"
                : "H7,TEST,TASK2,INITIAL_SHIFT,RIGHT=400mm\r\n");
        (void)run_log_save_event((uint32_t)g_run_state, g_fault_code,
                                 RUN_LOG_EVENT_TASK2_INITIAL_SHIFT_START);
        moved = run_translation_profile(
            0.0f, lateral_sign, ROUTE_TASK2_TEST_INITIAL_LATERAL_M,
            ROUTE_TASK2_TEST_INITIAL_TRANSLATION_SPEED_M_S,
            ROUTE_TRANSLATION_ACCEL_M_S2);
        if (moved) {
            (void)run_log_save_event((uint32_t)g_run_state, g_fault_code,
                                     RUN_LOG_EVENT_TASK2_INITIAL_SHIFT_DONE);

            /* The standalone app owns the main camera after START/RUNNING.
             * Keep the chassis moving while it answers H7 white-line queries,
             * then make the short calibrated approach before DONE. */
            g_run_state = RUN_DISC_VISUAL_ALIGN;
            board_uart1_write(
                "H7,TEST,TASK2,WHITE_LINE,START,REF_Y10=2000,TOL=100,ACCEL=0.10\r\n");
            if (!run_disc_visual_alignment_at_speed(
                    ROUTE_TASK2_TEST_WHITE_LINE_FORWARD_SPEED_M_S,
                    ROUTE_TASK2_TEST_WHITE_LINE_REFERENCE_Y10,
                    ROUTE_TASK2_TEST_WHITE_LINE_REFERENCE_TOLERANCE_Y10,
                    ROUTE_TASK2_TEST_WHITE_LINE_ACCEL_M_S2,
                    ROUTE_WHITE_LINE_PHASE_TASK2_AFTER_SHIFT)) {
                task2_test_send_error("WHITE_LINE", sequence);
                g_task2_test_active_sequence = 0U;
                return false;
            }
            g_run_state = RUN_DISC_FINAL_APPROACH;
            {
                char reference_log[96];

                (void)snprintf(
                    reference_log, sizeof(reference_log),
                    "H7,TEST,TASK2,WHITE_LINE,REFERENCE_REACHED,FORWARD=%umm\r\n",
                    (unsigned)(ROUTE_TASK2_TEST_WHITE_LINE_AFTER_CROSSED_FORWARD_M *
                               1000.0f + 0.5f));
                board_uart1_write(reference_log);
            }
            /* The reference position is followed by the configured short
             * calibrated approach before the station decision. */
            moved = true;
            if (ROUTE_TASK2_TEST_WHITE_LINE_AFTER_CROSSED_FORWARD_M > 0.0f) {
                moved = run_translation_profile(
                    ROUTE_FORWARD_SIGN, 0.0f,
                    ROUTE_TASK2_TEST_WHITE_LINE_AFTER_CROSSED_FORWARD_M,
                    ROUTE_TASK2_TEST_WHITE_LINE_FORWARD_SPEED_M_S,
                    ROUTE_TASK2_TEST_WHITE_LINE_ACCEL_M_S2);
            }
        }
    } else {
        float shift_distance_m;

        g_run_state = g_task2_test_is_red != 0U
                          ? RUN_PLATFORM_SHIFT_RIGHT
                          : RUN_PLATFORM_SHIFT_LEFT;

        switch (g_task2_test_step) {
        case 1U:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_1_M;
            break;
        case 2U:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_2_M;
            break;
        case 3U:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_3_M;
            break;
        case 4U:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_4_M;
            break;
        case 5U:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_5_M;
            break;
        case 6U:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_6_M;
            break;
        default:
            shift_distance_m = ROUTE_TASK2_PLATFORM_SHIFT_7_M;
            break;
        }
        moved = run_translation_profile(
            0.0f, -lateral_sign, shift_distance_m,
            ROUTE_TASK2_TEST_TRANSLATION_SPEED_M_S,
            ROUTE_TRANSLATION_ACCEL_M_S2);
    }
    (void)route_motor_send_zero_all();
    if (g_task2_test_stop_requested != 0U) {
        task2_test_send_status("STOPPED", sequence);
        g_task2_test_active_sequence = 0U;
        g_task2_test_stop_requested = 0U;
        g_fault_code = FAULT_NONE;
        return true;
    }
    if (!moved) {
        task2_test_send_error("MOTOR_ROUTE", sequence);
        g_task2_test_active_sequence = 0U;
        return false;
    }
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    task2_test_send_status("DONE", sequence);
    if (g_task2_test_step >= 7U) {
        g_task2_test_active_sequence = 0U;
    }
    return g_run_state != RUN_FAULT;
}

/* Formal task-two entry mirrors the standalone task-two commissioning path.
 * The preselect transaction has already completed when this function is
 * called, so this routine only performs the first lateral move, the main
 * camera white-line alignment, and the calibrated approach to station one. */
bool route_controller_run_task2_platform_entry(void)
{
    const float lateral_sign = g_route_field_is_red != 0U
                                   ? -ROUTE_RIGHT_STRAFE_SIGN
                                   : ROUTE_RIGHT_STRAFE_SIGN;
    bool moved;

    g_run_state = g_route_field_is_red != 0U
                      ? RUN_PLATFORM_SHIFT_LEFT
                      : RUN_PLATFORM_SHIFT_RIGHT;
    board_uart1_write(
        g_route_field_is_red != 0U
            ? "H7,ROUTE,TASK2_INITIAL_SHIFT,LEFT=400mm,MODE=TASK2_WHITE_LINE\r\n"
            : "H7,ROUTE,TASK2_INITIAL_SHIFT,RIGHT=400mm,MODE=TASK2_WHITE_LINE\r\n");
    moved = run_translation_profile(
        0.0f, lateral_sign, ROUTE_TASK2_TEST_INITIAL_LATERAL_M,
        ROUTE_TASK2_TEST_INITIAL_TRANSLATION_SPEED_M_S,
        ROUTE_TRANSLATION_ACCEL_M_S2);
    if (!moved) {
        return false;
    }

    g_run_state = RUN_DISC_VISUAL_ALIGN;
    board_uart1_write(
        "H7,ROUTE,TASK2_WHITE_LINE,START,REF_Y10=2000,TOL=100,"
        "SPEED=0.10,ACCEL=0.10\r\n");
    if (!run_disc_visual_alignment_at_speed(
            ROUTE_TASK2_TEST_WHITE_LINE_FORWARD_SPEED_M_S,
            ROUTE_TASK2_TEST_WHITE_LINE_REFERENCE_Y10,
            ROUTE_TASK2_TEST_WHITE_LINE_REFERENCE_TOLERANCE_Y10,
            ROUTE_TASK2_TEST_WHITE_LINE_ACCEL_M_S2,
            ROUTE_WHITE_LINE_PHASE_TASK2_AFTER_SHIFT)) {
        return false;
    }

    g_run_state = RUN_DISC_FINAL_APPROACH;
    board_uart1_write(
        "H7,ROUTE,TASK2_WHITE_LINE,REFERENCE_REACHED,FORWARD=150mm\r\n");
    moved = run_translation_profile(
        ROUTE_FORWARD_SIGN, 0.0f,
        ROUTE_TASK2_TEST_WHITE_LINE_AFTER_CROSSED_FORWARD_M,
        ROUTE_TASK2_TEST_WHITE_LINE_FORWARD_SPEED_M_S,
        ROUTE_TASK2_TEST_WHITE_LINE_ACCEL_M_S2);
    (void)route_motor_send_zero_all();
    return moved;
}

bool route_controller_take_task3_test(uint8_t *is_red, uint32_t *sequence)
{
    if (is_red == NULL || sequence == NULL ||
        g_task3_test_pending == 0U) {
        return false;
    }

    *is_red = g_task3_test_is_red;
    *sequence = g_task3_test_sequence;
    g_task3_test_pending = 0U;
    g_task3_test_active_sequence = *sequence;
    g_task3_test_pause_requested = 0U;
    g_task3_test_resume_requested = 0U;
    g_task3_test_stop_requested = 0U;
    g_task3_test_paused = 0U;
    g_task3_test_slow_requested = 0U;
    return true;
}

bool route_controller_run_task3_test(uint32_t sequence)
{
    bool moved;
    const float orbit_angle_rad =
        (g_task3_test_is_red != 0U ? ROUTE_LEFT_TURN_SIGN
                                   : ROUTE_RIGHT_TURN_SIGN) *
        ROUTE_TASK3_TEST_ORBIT_ANGLE_RAD * ROUTE_GYRO_TURN_SCALE;

    if (sequence == 0U || g_task3_test_active_sequence != sequence) {
        g_fault_code = FAULT_KINEMATICS;
        return false;
    }

    task3_test_send_status("RUNNING", sequence);
    board_uart1_write_only(g_task3_test_is_red != 0U
                               ? "H7,TEST,TASK3,ORBIT_START,DIR=LEFT,"
                                 "RADIUS=400mm,ANGLE=360deg\r\n"
                               : "H7,TEST,TASK3,ORBIT_START,DIR=RIGHT,"
                                 "RADIUS=400mm,ANGLE=360deg\r\n");
    moved = run_front_center_orbit(
        orbit_angle_rad,
        ROUTE_TASK3_TEST_ORBIT_RADIUS_M);
    if (g_task3_test_active_sequence == 0U) {
        /* STOP or RC takeover already emitted STOPPED and left the motor
         * safe. Do not turn a deliberate stop into a fault. */
        return false;
    }
    if (!moved) {
        task3_test_send_error("ORBIT", sequence);
        g_task3_test_active_sequence = 0U;
        return false;
    }
    (void)route_motor_send_zero_all();
    route_controller_hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    task3_test_send_status("DONE", sequence);
    g_task3_test_active_sequence = 0U;
    g_task3_test_pause_requested = 0U;
    g_task3_test_resume_requested = 0U;
    g_task3_test_stop_requested = 0U;
    g_task3_test_paused = 0U;
    g_task3_test_slow_requested = 0U;
    return g_run_state != RUN_FAULT;
}

bool route_controller_run_disc_arc_entry(float lateral_sign,
                                         float turn_sign)
{
    return run_disc_arc_entry(lateral_sign, turn_sign);
}

bool route_controller_run_disc_visual_alignment(void)
{
    return run_disc_visual_alignment();
}

bool route_controller_run_relative_turn(float angle_rad)
{
    return run_relative_turn(angle_rad);
}

bool route_controller_run_front_center_orbit(float angle_rad,
                                             float center_distance_m)
{
    return run_front_center_orbit(angle_rad, center_distance_m);
}

bool route_controller_wait_for_rk_arm_task(const char *task)
{
    return wait_for_rk_arm_task(task);
}

bool route_controller_wait_for_rk_platform_preselect(void)
{
    return wait_for_rk_platform_transaction(0U, true);
}

bool route_controller_wait_for_rk_platform_slot(uint32_t slot)
{
    return wait_for_rk_platform_transaction(slot, false);
}

bool route_controller_start_rk_arm_task(const char *task)
{
    return start_rk_arm_task(task);
}

bool route_controller_stop_rk_arm_task(const char *task)
{
    return stop_rk_arm_task(task);
}

bool route_controller_run_zp_aux(uint32_t channel, uint32_t pulse,
                                 uint32_t time_ms, uint8_t servo_id)
{
    return run_zp_aux(channel, pulse, time_ms, servo_id);
}

void route_controller_set_task3_orbit_test_mode(uint8_t enable)
{
    /* Reuses the standalone task-three gating inside run_front_center_orbit
     * (slow speed, ramp, 120 s timeout). The sentinel sequence never matches
     * a real test transaction, so USB test commands stay inert. */
    g_task3_test_active_sequence = enable != 0U ? 0xFFFFU : 0U;
}

#if ROUTE_AUTO_RUN_ON_BOOT == 0U
void route_controller_wait_for_usb_run_command(void)
{
    wait_for_usb_run_command();
}
#endif

#if ROUTE_WAIT_RK_READY_ON_BOOT
void route_controller_wait_for_rk_ready_on_boot(void)
{
    wait_for_rk_ready_on_boot();
}
#endif

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
    }
}
