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
static uint8_t g_rk_arm_tasks_disabled_for_route;
static uint8_t g_first_arm_station_reached;
static uint8_t g_start_confirmed_from_fault;
static uint8_t g_route_field_is_red;
static uint8_t g_rk_reset_pending;
static uint32_t g_rk_pretask_last_sync_ms;
static char g_rk_pretask_line[96];
static uint32_t g_rk_pretask_line_len;
static void service_rk_link_before_first_station(void);
static bool enable_motors(void);

static void request_rk_arm_reset(void)
{
    g_rk_reset_pending = 1U;
    g_rk_arm_link_ready = 0U;
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

    g_run_state = RUN_WAIT_USB_RUN;
    lcd_display_set_start_status("WAIT");
    board_uart1_write("H7,START,WAIT_FIELD,joystick=UP_RED_OR_DOWN_BLUE\r\n");
    wait_for_user_start_key_release("WAIT");
    for (;;) {
        uint32_t now_ms = HAL_GetTick();

        lcd_display_update();
        if ((uint32_t)(now_ms - last_status_ms) >= 1000U) {
            last_status_ms = now_ms;
            board_uart1_write("H7,START,WAIT_USER_KEY\r\n");
        }
        if (rc_override_service()) {
            lcd_display_set_start_status("WAIT");
            continue;
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

static bool route_motor_send_zero_all(void)
{
    service_rk_link_before_first_station();
    if (!rc_override_is_running() && rc_override_service()) {
        request_rk_arm_reset();
        g_fault_code = FAULT_RC_OVERRIDE;
        return false;
    }
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

static bool route_motor_feedback_update(uint32_t now_ms, float wheel_rad_s[4])
{
    if (motor_feedback_update(now_ms, wheel_rad_s)) {
        return true;
    }
#if ROUTE_REQUIRE_MOTOR_FEEDBACK
    return false;
#else
    if (wheel_rad_s != NULL) {
        memset(wheel_rad_s, 0, 4U * sizeof(wheel_rad_s[0]));
    }
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
                g_fault_code = FAULT_MOTOR_COMMAND;
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

    g_fault_code = code;
    g_run_state = RUN_FAULT;
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
    request_rk_arm_reset();
    lcd_display_set_start_status("FAULT");
    board_uart1_write("H7,FAULT,SAVED,WAIT_START_TO_RESET\r\n");
    board_clear_selected_field();
    start_release_seen = board_user_start_active() == 0U ? 1U : 0U;
    for (;;) {
        (void)motor_send_zero_all();
        (void)motor_disable_all();
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
    return motor_clear_errors_all() &&
           board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS) &&
           motor_enable_all() &&
           board_fdcan1_wait_tx_fifo_free(8U, MOTOR_TX_DRAIN_TIMEOUT_MS) &&
           motor_send_zero_all() &&
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

static void rk_arm_handle_line(const char *line)
{
    if (line_starts_with(line, "RK,ARM,RESET,DONE")) {
        g_rk_reset_pending = 0U;
        g_rk_arm_link_ready = 1U;
        board_uart1_write_only("H7,ARM,RK_RESET_DONE\r\n");
        return;
    }
    if (line_starts_with(line, "RK,ARM,RESET,ACK")) {
        g_rk_arm_link_ready = 1U;
        return;
    }
    if (line_starts_with(line, "RK,ARM,READY")) {
        g_rk_arm_link_ready = 1U;
        board_uart1_write_only("H7,ARM,RK_READY\r\n");
    }
}

static void service_rk_link_before_first_station(void)
{
    uint8_t rx[64];
    uint32_t now_ms;
    uint32_t read_len;
    uint32_t i;

    if (g_first_arm_station_reached != 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_rk_pretask_last_sync_ms) >=
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
                rk_arm_handle_line(g_rk_pretask_line);
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
                    rk_arm_handle_line(line);
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
    bool rk_acknowledged = false;
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t ack_timeout_ms = (g_rk_arm_link_ready != 0U) ?
                              RK_ARM_ACK_TIMEOUT_MS :
                              RK_ARM_PROBE_ACK_TIMEOUT_MS;
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_status_ms = started_ms;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";

    if (g_rk_arm_tasks_disabled_for_route != 0U) {
        (void)snprintf(log_line, sizeof(log_line),
                       "H7,ARM,%s,SKIP_ROUTE_NO_RK\r\n", task);
        board_uart1_write(log_line);
        return true;
    }

    (void)snprintf(start_command, sizeof(start_command),
                   "ARM,%s,START,FIELD,%s\r\n", task, field_name);
    (void)snprintf(status_command, sizeof(status_command),
                   "ARM,%s,STATUS\r\n", task);
    (void)snprintf(ack_prefix, sizeof(ack_prefix),
                   "RK,ARM,%s,ACK", task);
    (void)snprintf(done_prefix, sizeof(done_prefix),
                   "RK,ARM,%s,DONE", task);
    (void)snprintf(idle_prefix, sizeof(idle_prefix),
                   "RK,ARM,%s,IDLE", task);
    (void)snprintf(log_line, sizeof(log_line), "H7,ARM,%s,WAIT_RK\r\n", task);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

#if RK_ARM_WAIT_FOREVER_FOR_ACK == 0U
        if (!rk_acknowledged &&
            (uint32_t)(now_ms - started_ms) >= ack_timeout_ms) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,BYPASS_NO_RK\r\n", task);
            board_uart1_write(log_line);
            if (line_starts_with(task, "DISC_CATCH")) {
                g_rk_arm_tasks_disabled_for_route = 1U;
                board_uart1_write_only("H7,ARM,ROUTE_ARM_TASKS_DISABLED\r\n");
            }
#if RK_ARM_REQUIRED
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return true;
#endif
        }
#endif
#if RK_ARM_TASK_TIMEOUT_MS > 0U
        if (rk_acknowledged &&
            (uint32_t)(now_ms - started_ms) >= RK_ARM_TASK_TIMEOUT_MS) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,TIMEOUT\r\n", task);
            board_uart1_write(log_line);
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
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
                    if (line_starts_with(line, ack_prefix)) {
                        rk_acknowledged = true;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,ACK\r\n", task);
                        board_uart1_write(log_line);
                    }
                    if (line_starts_with(line, done_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,DONE\r\n", task);
                        board_uart1_write(log_line);
                        return true;
                    }
                    if (rk_acknowledged && line_starts_with(line, idle_prefix)) {
                        rk_acknowledged = false;
                        started_ms = HAL_GetTick();
                        ack_timeout_ms = RK_ARM_ACK_TIMEOUT_MS;
                        last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,RESTART_AFTER_RK_IDLE\r\n", task);
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
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t ack_timeout_ms = (g_rk_arm_link_ready != 0U) ?
                              RK_ARM_ACK_TIMEOUT_MS :
                              RK_ARM_PROBE_ACK_TIMEOUT_MS;
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;
    const char *field_name = g_route_field_is_red != 0U ? "RED" : "BLUE";

    if (g_rk_arm_tasks_disabled_for_route != 0U) {
        (void)snprintf(log_line, sizeof(log_line),
                       "H7,ARM,%s,SKIP_ROUTE_NO_RK\r\n", task);
        board_uart1_write(log_line);
        return false;
    }

    (void)snprintf(start_command, sizeof(start_command),
                   "ARM,%s,START,FIELD,%s\r\n", task, field_name);
    (void)snprintf(ack_prefix, sizeof(ack_prefix),
                   "RK,ARM,%s,ACK", task);
    (void)snprintf(log_line, sizeof(log_line), "H7,ARM,%s,START_ASYNC\r\n", task);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

#if RK_ARM_WAIT_FOREVER_FOR_ACK == 0U
        if ((uint32_t)(now_ms - started_ms) >= ack_timeout_ms) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,BYPASS_NO_RK_ASYNC\r\n", task);
            board_uart1_write(log_line);
#if RK_ARM_REQUIRED
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return false;
#endif
        }
#endif
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
                    if (line_starts_with(line, ack_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,ACK_ASYNC\r\n", task);
                        board_uart1_write(log_line);
                        return true;
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
    uint32_t line_len = 0U;
    uint32_t started_ms = HAL_GetTick();
    uint32_t last_send_ms = started_ms - RK_ARM_START_RETRY_MS;
    uint32_t last_zero_ms = started_ms - CONTROL_PERIOD_MS;

    (void)snprintf(stop_command, sizeof(stop_command),
                   "ARM,%s,STOP\r\n", task);
    (void)snprintf(done_prefix, sizeof(done_prefix),
                   "RK,ARM,%s,DONE", task);
    (void)snprintf(log_line, sizeof(log_line), "H7,ARM,%s,STOP_WAIT\r\n", task);
    board_uart1_write(log_line);

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint32_t read_len;
        uint32_t i;

        if ((uint32_t)(now_ms - started_ms) >= RK_ARM_STOP_TIMEOUT_MS) {
            (void)snprintf(log_line, sizeof(log_line),
                           "H7,ARM,%s,STOP_TIMEOUT\r\n", task);
            board_uart1_write(log_line);
#if RK_ARM_REQUIRED
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return true;
#endif
        }
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
                    if (line_starts_with(line, done_prefix)) {
                        (void)snprintf(log_line, sizeof(log_line),
                                       "H7,ARM,%s,DONE_STOP\r\n", task);
                        board_uart1_write(log_line);
                        return true;
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
        HAL_Delay(1U);
    }
}

static bool settle_translation_cross_track(float along_x, float along_y,
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
        if (!route_motor_feedback_update(now_ms, measured_wheel_speed) ||
            !mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s,
                             &actual_wz_rad_s)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        (void)actual_wz_rad_s;

        heading_error_for_transform = clampf(
            g_yaw_rad - heading_target_rad,
            -TRANSLATION_FIELD_ORIENT_MAX_ERROR_RAD,
            TRANSLATION_FIELD_ORIENT_MAX_ERROR_RAD);
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
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();

        if (fabsf(*cross_track_m) <= TRANSLATION_CROSS_TRACK_TOLERANCE_M &&
            fabsf(g_actual_cross_speed_m_s) <=
                TRANSLATION_CROSS_SPEED_TOLERANCE_M_S) {
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

static bool run_translation_profile(float vx_direction, float vy_direction,
                                    float target_distance_m,
                                    float maximum_speed_m_s,
                                    float acceleration_m_s2)
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
    const float heading_target_rad = g_route_heading_target_rad;
    const bool is_strafe = fabsf(vy_direction) > fabsf(vx_direction);
    const float heading_kp = is_strafe ? STRAFE_HEADING_KP : HEADING_KP;
    const float heading_kd = is_strafe ? STRAFE_HEADING_KD : HEADING_KD;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    const uint32_t started_ms = previous_ms;
    uint32_t settled_since_ms = 0U;

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

        if ((uint32_t)(now_ms - started_ms) >= ROUTE_TRANSLATION_TIMEOUT_MS) {
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
        if (!route_motor_feedback_update(now_ms, measured_wheel_speed) ||
            !mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s, &actual_wz_rad_s)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }

        /* Positive yaw is a right turn on this chassis.  Transform measured
         * body velocity into the fixed frame captured at segment start. */
        heading_error_for_transform = clampf(
            g_yaw_rad - heading_target_rad,
            -TRANSLATION_FIELD_ORIENT_MAX_ERROR_RAD,
            TRANSLATION_FIELD_ORIENT_MAX_ERROR_RAD);
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

        correction_limit_rad_s =
            fabsf(along_speed_command_m_s) * HEADING_CORRECTION_SPEED_RATIO /
            (CHASSIS_HALF_LENGTH_M + CHASSIS_HALF_WIDTH_M);
        if (correction_limit_rad_s > HEADING_MAX_CORRECTION_RAD_S) {
            correction_limit_rad_s = HEADING_MAX_CORRECTION_RAD_S;
        }
        g_heading_correction_rad_s = clampf(
            -heading_kp * (g_yaw_rad - heading_target_rad) -
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
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();

        if (fabsf(target_distance_m - segment_distance_m) <=
                ODOM_ALONG_POSITION_TOLERANCE_M &&
            fabsf(actual_translation_speed_m_s) <=
                ODOM_ALONG_SPEED_TOLERANCE_M_S) {
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
    return settle_translation_cross_track(along_x, along_y,
                                          heading_target_rad, heading_kp,
                                          heading_kd, &cross_track_m);
}

static bool run_translation(float vx_direction, float vy_direction,
                            float target_distance_m)
{
    return run_translation_profile(vx_direction, vy_direction,
                                   target_distance_m,
                                   ROUTE_TRANSLATION_SPEED_M_S,
                                   ROUTE_TRANSLATION_ACCEL_M_S2);
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
        if (!route_motor_feedback_update(now_ms, measured_wheel_speed)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
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
    float route_yaw_rad = g_yaw_rad;
    const float target_yaw_rad = g_route_heading_target_rad + angle_rad;
    uint32_t previous_ms = HAL_GetTick();
    uint32_t last_control_ms = previous_ms;
    uint32_t last_log_ms = previous_ms - RUN_LOG_SAMPLE_PERIOD_MS;
    uint32_t settled_since_ms = 0U;
    const uint32_t started_ms = previous_ms;

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
        float orbit_vy_m_s;

        if ((uint32_t)(now_ms - started_ms) >= ROUTE_FRONT_CENTER_ORBIT_TIMEOUT_MS) {
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
        if (!route_motor_feedback_update(now_ms, measured_wheel_speed) ||
            !mecanum_forward(&chassis, measured_wheel_speed,
                             &actual_vx_m_s, &actual_vy_m_s, &actual_wz_rad_s)) {
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }

        error_rad = target_yaw_rad - route_yaw_rad;
        desired_rad_s = clampf(ROUTE_ORBIT_KP * error_rad -
                                   ROUTE_ORBIT_KD * g_gyro_z_rad_s,
                               -ROUTE_ORBIT_MAX_SPEED_RAD_S,
                               ROUTE_ORBIT_MAX_SPEED_RAD_S);
        max_delta_rad_s = ROUTE_ORBIT_ACCEL_RAD_S2 * dt;
        turn_command_rad_s = clampf(desired_rad_s,
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
            g_fault_code = FAULT_MOTOR_COMMAND;
            return false;
        }
        route_yaw_rad = g_yaw_rad;
        g_estimated_distance_m +=
            fabsf(actual_wz_rad_s) * center_distance_m * dt * DRIVE_DISTANCE_SCALE;
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

void route_controller_set_field(uint8_t is_red)
{
    g_route_field_is_red = is_red != 0U ? 1U : 0U;
}

void route_controller_reset_run_context(void)
{
    g_fault_code = FAULT_NONE;
    g_rk_arm_tasks_disabled_for_route = 0U;
    g_first_arm_station_reached = 0U;
    g_rk_arm_link_ready = 0U;
    g_rk_reset_pending = 0U;
    g_rk_pretask_line_len = 0U;
    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_estimated_distance_m = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;
    run_log_reset();
    request_rk_arm_reset();
}

void route_controller_begin_pretask_sync(void)
{
    g_rk_pretask_last_sync_ms = HAL_GetTick() - RK_ARM_PRETASK_SYNC_PERIOD_MS;
    board_uart1_write(g_route_field_is_red != 0U
                          ? "H7,ARM,PRETASK_SYNC_ACTIVE,FIELD=RED\r\n"
                          : "H7,ARM,PRETASK_SYNC_ACTIVE,FIELD=BLUE\r\n");
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

void route_controller_reset_pose(void)
{
    g_yaw_rad = 0.0f;
    g_route_heading_target_rad = 0.0f;
    g_estimated_distance_m = 0.0f;
}

uint8_t route_controller_arm_tasks_disabled(void)
{
    return g_rk_arm_tasks_disabled_for_route;
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

bool route_controller_run_translation(float vx_direction, float vy_direction,
                                      float target_distance_m)
{
    return run_translation(vx_direction, vy_direction, target_distance_m);
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

bool route_controller_start_rk_arm_task(const char *task)
{
    return start_rk_arm_task(task);
}

bool route_controller_stop_rk_arm_task(const char *task)
{
    return stop_rk_arm_task(task);
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
