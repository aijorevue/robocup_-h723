#include "app_config.h"
#include "board.h"
#include "BMI088driver.h"
#include "lcd_display.h"
#include "route_controller.h"
#include "mecanum.h"
#include "motor_output.h"
#include "run_log.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    RUN_BOOT = 0,
    RUN_IMU_INIT,
    RUN_GYRO_CALIBRATION,
    RUN_MOTOR_ENABLE,

    RUN_STRAFE_RIGHT,
    RUN_STOPPING = 5,
    RUN_DONE = 6,
    RUN_FAULT = 7,
    
    //圆盘机任务
    RUN_FORWARD = 8,
    RUN_TURN_RIGHT = 9,

    RUN_REVERSE_AFTER_DISC = 10,
    RUN_SECOND_TURN_RIGHT = 11,
    RUN_PLATFORM_APPROACH = 12,
    RUN_ROUTE_RESERVED_13 = 13,
    
    //台柱任务
    RUN_THIRD_TURN_RIGHT = 14,
    RUN_WAIT_USB_RUN = 15,
    RUN_PLATFORM_SHIFT_LEFT = 16,
    RUN_DIAGONAL_AFTER_PLATFORM = 17,

    //圆台任务
    RUN_LAST_TURN_RIGHT = 18,
    RUN_FRONT_CENTER_ORBIT = 19,
    RUN_FINAL_REVERSE = 20,
    RUN_SERVO_90 = 21,
    RUN_ARM_DISC_CATCH = 22,
    RUN_ARM_PLATFORM_PICK = 23,
    RUN_ARM_COLUMN_CATCH = 24
} run_state_t;

enum {
    FAULT_NONE = 0,
    FAULT_IMU_INIT = 1,
    FAULT_IMU_MOVING = 2,
    FAULT_CAN_STARTUP = 3,
    FAULT_MOTOR_ENABLE = 4,
    FAULT_MOTOR_COMMAND = 5,
    FAULT_KINEMATICS = 6,
    FAULT_TURN_TIMEOUT = 7,
    FAULT_ARM_TIMEOUT = 8
};

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
static volatile uint8_t g_rk_arm_link_ready;
static uint8_t g_rk_arm_tasks_disabled_for_route;
static uint8_t g_first_arm_station_reached;
static uint8_t g_start_confirmed_from_fault;
static uint32_t g_rk_pretask_last_sync_ms;
static char g_rk_pretask_line[96];
static uint32_t g_rk_pretask_line_len;

static void service_rk_link_before_first_station(void);

uint8_t route_controller_rk_link_ready(void)
{
    return g_rk_arm_link_ready;
}

#if ROUTE_WAIT_USER_KEY_ON_BOOT
static void wait_for_user_start_key(void)
{
    uint32_t last_status_ms = HAL_GetTick() - 1000U;

    g_run_state = RUN_WAIT_USB_RUN;
    lcd_display_set_start_status("WAIT");
    board_uart1_write("H7,START,WAIT_USER_KEY,code=1,PA15_OR_LCD_JOYSTICK\r\n");
    for (;;) {
        uint32_t now_ms = HAL_GetTick();

        lcd_display_update();
        if ((uint32_t)(now_ms - last_status_ms) >= 1000U) {
            last_status_ms = now_ms;
            board_uart1_write("H7,START,WAIT_USER_KEY\r\n");
        }
        if (board_user_start_pressed() != 0U) {
            lcd_display_set_start_status("RUN");
            board_uart1_write("H7,START,USER_KEY,code=1\r\n");
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

static bool route_motor_send_zero_all(void)
{
    service_rk_link_before_first_station();
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
#if ROUTE_TASK_LINK_SIMULATION_ONLY
    return true;
#else
    return route_motor_send_zero_all();
#endif
}

static bool route_motor_send_wheel_speeds(const float wheel_rad_s[4])
{
    service_rk_link_before_first_station();
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
    }
}

static void enter_fault_wait_restart(uint32_t code)
{
    static const float stopped[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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
    lcd_display_set_start_status("FAULT");
    board_uart1_write("H7,FAULT,SAVED,WAIT_START_TO_RESET\r\n");
    for (;;) {
        (void)motor_send_zero_all();
        (void)motor_disable_all();
        lcd_display_update();
        if (board_user_start_pressed() != 0U) {
            g_start_confirmed_from_fault = 1U;
            lcd_display_set_start_status("RUN");
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
    uint32_t sample;

    for (sample = 0U; sample < GYRO_CALIBRATION_SAMPLES; ++sample) {
        float z;
        BMI088_read(gyro, accel, &temperature);
        z = gyro[2] * GYRO_Z_SIGN;
        sum += z;
        sum_square += z * z;
        g_imu_temperature_c = temperature;
        service_rk_link_before_first_station();
        HAL_Delay(GYRO_CALIBRATION_PERIOD_MS);
    }

    g_gyro_z_bias_rad_s = sum / (float)GYRO_CALIBRATION_SAMPLES;
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
        HAL_Delay(20U);
    }
    started_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started_ms) < CAN_STARTUP_RETRY_TIMEOUT_MS) {
        service_rk_link_before_first_station();
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
    if (g_rk_arm_link_ready == 0U &&
        (uint32_t)(now_ms - g_rk_pretask_last_sync_ms) >=
            RK_ARM_PRETASK_SYNC_PERIOD_MS) {
        g_rk_pretask_last_sync_ms = now_ms;
        board_usb_write("ARM,SYNC\r\n");
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
            board_usb_write("ARM,SYNC\r\n");
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

    if (g_rk_arm_tasks_disabled_for_route != 0U) {
        (void)snprintf(log_line, sizeof(log_line),
                       "H7,ARM,%s,SKIP_ROUTE_NO_RK\r\n", task);
        board_uart1_write(log_line);
        return true;
    }

    (void)snprintf(start_command, sizeof(start_command),
                   "ARM,%s,START\r\n", task);
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
#if RK_ARM_REQUIRED || ROUTE_TASK_LINK_SIMULATION_ONLY
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
                g_fault_code = FAULT_MOTOR_COMMAND;
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

    if (g_rk_arm_tasks_disabled_for_route != 0U) {
        (void)snprintf(log_line, sizeof(log_line),
                       "H7,ARM,%s,SKIP_ROUTE_NO_RK\r\n", task);
        board_uart1_write(log_line);
        return false;
    }

    (void)snprintf(start_command, sizeof(start_command),
                   "ARM,%s,START\r\n", task);
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
#if RK_ARM_REQUIRED || ROUTE_TASK_LINK_SIMULATION_ONLY
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
                g_fault_code = FAULT_MOTOR_COMMAND;
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
#if RK_ARM_REQUIRED || ROUTE_TASK_LINK_SIMULATION_ONLY
            g_fault_code = FAULT_ARM_TIMEOUT;
            return false;
#else
            return true;
#endif
        }
        if ((uint32_t)(now_ms - last_zero_ms) >= CONTROL_PERIOD_MS) {
            last_zero_ms = now_ms;
            if (!keep_chassis_stopped_for_arm_task()) {
                g_fault_code = FAULT_MOTOR_COMMAND;
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
    }
}

static void arm_set_pair(float sweep_angle_deg, float gripper_angle_deg,
                         uint32_t settle_ms)
{
    board_servo_set_angle_deg_index(ARM_SERVO_SWEEP_INDEX, sweep_angle_deg);
    board_servo_set_angle_deg_index(ARM_SERVO_GRIPPER_INDEX, gripper_angle_deg);
    HAL_Delay(settle_ms);
    board_servo_disable_index(ARM_SERVO_SWEEP_INDEX);
    board_servo_disable_index(ARM_SERVO_GRIPPER_INDEX);
}

static void __attribute__((unused)) arm_disc_catch(void)
{
    arm_set_pair(ARM_DISC_SWEEP_ANGLE_DEG, ARM_DISC_GRIP_ANGLE_DEG,
                 ARM_ACTION_SETTLE_MS);
}

static void __attribute__((unused)) arm_platform_pick(void)
{
    static const float platform_angles[3] = {
        ARM_PLATFORM_LEFT_ANGLE_DEG,
        ARM_PLATFORM_CENTER_ANGLE_DEG,
        ARM_PLATFORM_RIGHT_ANGLE_DEG
    };
    uint32_t index;

    for (index = 0U; index < 3U; ++index) {
        arm_set_pair(platform_angles[index], ARM_GRIP_OPEN_ANGLE_DEG,
                     ARM_ACTION_SETTLE_MS);
        arm_set_pair(platform_angles[index], ARM_GRIP_CLOSE_ANGLE_DEG,
                     ARM_GRIP_SETTLE_MS);
    }
}

static void __attribute__((unused)) arm_column_catch(void)
{
    arm_set_pair(ARM_COLUMN_SWEEP_ANGLE_DEG, ARM_COLUMN_GRIP_ANGLE_DEG,
                 ARM_ACTION_SETTLE_MS);
}

static bool settle_translation_cross_track(float along_x, float along_y,
                                             float heading_target_rad,
                                             float heading_kp,
                                             float heading_kd,
                                             float *cross_track_m)
{
    float wheel_speed[4] = {0.0f};
    float measured_wheel_speed[4] = {0.0f};
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
        g_actual_cross_speed_m_s =
            actual_route_vx_m_s * cross_x + actual_route_vy_m_s * cross_y;
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
    float actual_vx_m_s = 0.0f;
    float actual_vy_m_s = 0.0f;
    float actual_wz_rad_s = 0.0f;
    float segment_distance_m = 0.0f;
    float commanded_distance_m = 0.0f;
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

    while (segment_distance_m < target_distance_m - DRIVE_STOP_TOLERANCE_M &&
           commanded_distance_m < target_distance_m - DRIVE_STOP_TOLERANCE_M) {
        uint32_t now_ms = HAL_GetTick();
        float dt;
        float feedback_remaining;
        float command_remaining;
        float remaining;
        float stopping_speed;
        float desired_speed;
        float max_delta;
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
        actual_translation_speed_m_s =
            actual_route_vx_m_s * along_x + actual_route_vy_m_s * along_y;
        actual_cross_speed_m_s =
            actual_route_vx_m_s * cross_x + actual_route_vy_m_s * cross_y;
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
        if (g_command_speed_m_s < desired_speed) {
            g_command_speed_m_s += max_delta;
            if (g_command_speed_m_s > desired_speed) {
                g_command_speed_m_s = desired_speed;
            }
        } else {
            g_command_speed_m_s -= max_delta;
            if (g_command_speed_m_s < desired_speed) {
                g_command_speed_m_s = desired_speed;
            }
        }

        correction_limit_rad_s =
            g_command_speed_m_s * HEADING_CORRECTION_SPEED_RATIO /
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
            g_command_speed_m_s * along_x + cross_track_command_m_s * cross_x;
        command_route_vy_m_s =
            g_command_speed_m_s * along_y + cross_track_command_m_s * cross_y;

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
        if (actual_translation_speed_m_s < 0.0f) {
            actual_translation_speed_m_s = 0.0f;
        }
        segment_distance_m += actual_translation_speed_m_s * dt * DRIVE_DISTANCE_SCALE;
        g_estimated_distance_m += actual_translation_speed_m_s * dt * DRIVE_DISTANCE_SCALE;
        commanded_distance_m += g_command_speed_m_s * dt;
        if ((uint32_t)(now_ms - last_log_ms) >= RUN_LOG_SAMPLE_PERIOD_MS) {
            last_log_ms = now_ms;
            log_route_sample(now_ms, measured_wheel_speed);
        }
        /* Self-throttled to 5 Hz, repaints a single value field per tick. */
        lcd_display_update();
    }
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
#if ROUTE_AIRBORNE_SIMULATE_YAW
        route_yaw_rad += turn_command_rad_s * dt;
        g_yaw_rad = route_yaw_rad;
#else
        route_yaw_rad = g_yaw_rad;
#endif
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
#if ROUTE_AIRBORNE_SIMULATE_YAW
        route_yaw_rad += turn_command_rad_s * dt;
        g_yaw_rad = route_yaw_rad;
        actual_wz_rad_s = turn_command_rad_s;
#else
        route_yaw_rad = g_yaw_rad;
#endif
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

#if ROUTE_TASK_LINK_SIMULATION_ONLY
static void task_link_simulation_halt(uint32_t fault_code)
{
    g_fault_code = fault_code;
    g_run_state = fault_code == FAULT_NONE ? RUN_DONE : RUN_FAULT;
    log_route_event(fault_code == FAULT_NONE ? RUN_LOG_EVENT_ROUTE_DONE
                                             : RUN_LOG_EVENT_FAULT);
    (void)run_log_save((uint32_t)g_run_state, g_fault_code);
    run_log_dump_stored();
    board_uart1_write(fault_code == FAULT_NONE
                          ? "H7,SIM,COMPLETE\r\n"
                          : "H7,SIM,HALTED_ON_ERROR\r\n");

    for (;;) {
        HAL_Delay(1000U);
    }
}

static void run_task_link_simulation(void)
{
    uint32_t platform_index;

    board_uart1_write("H7,SIM,TASK_LINK_ONLY\r\n");
    HAL_Delay(ROUTE_TASK_SIM_BOOT_DELAY_MS);

    while (g_rk_arm_link_ready == 0U) {
        wait_for_rk_ready_on_boot();
    }
    board_uart1_write("H7,SIM,BEGIN_TASK_SEQUENCE\r\n");

    g_run_state = RUN_ARM_DISC_CATCH;
    log_route_event(RUN_LOG_EVENT_ARM_START);
    if (!wait_for_rk_arm_task(ROUTE_TASK1_RK_ARM_TASK)) {
        task_link_simulation_halt(FAULT_ARM_TIMEOUT);
    }
    log_route_event(RUN_LOG_EVENT_ARM_DONE);
    HAL_Delay(ROUTE_TASK_SIM_BETWEEN_STATIONS_MS);

    for (platform_index = 0U;
         platform_index < ROUTE_TASK2_PLATFORM_PICK_COUNT;
         ++platform_index) {
        g_run_state = RUN_ARM_PLATFORM_PICK;
        log_route_event(RUN_LOG_EVENT_ARM_START);
        if (!wait_for_rk_arm_task(ROUTE_TASK2_RK_ARM_TASK)) {
            task_link_simulation_halt(FAULT_ARM_TIMEOUT);
        }
        log_route_event(RUN_LOG_EVENT_ARM_DONE);
        HAL_Delay(ROUTE_TASK_SIM_BETWEEN_STATIONS_MS);
    }

    g_run_state = RUN_ARM_COLUMN_CATCH;
    log_route_event(RUN_LOG_EVENT_ARM_START);
    if (!start_rk_arm_task(ROUTE_TASK3_RK_ARM_TASK)) {
        task_link_simulation_halt(FAULT_ARM_TIMEOUT);
    }
    log_route_event(RUN_LOG_EVENT_ARM_DONE);
    HAL_Delay(ROUTE_TASK_SIM_COLUMN_ACTIVE_MS);

    log_route_event(RUN_LOG_EVENT_ARM_STOP);
    if (!stop_rk_arm_task(ROUTE_TASK3_RK_ARM_TASK)) {
        task_link_simulation_halt(FAULT_ARM_TIMEOUT);
    }
    log_route_event(RUN_LOG_EVENT_ARM_STOP_DONE);
    task_link_simulation_halt(FAULT_NONE);
}
#endif

#define enter_fault(code)       \
    do {                        \
        enter_fault_wait_restart(code); \
        goto route_start;       \
    } while (0)

void route_controller_run(void)
{
#if ROUTE_TASK_LINK_SIMULATION_ONLY
    board_init_task_link_only();
#else
    board_init();
    /* Panel + static labels are painted once here, outside the control loop. */
    lcd_display_init();
#endif

route_start:
#if ROUTE_WAIT_USER_KEY_ON_BOOT
    if (g_start_confirmed_from_fault != 0U) {
        g_start_confirmed_from_fault = 0U;
        lcd_display_set_start_status("RUN");
    } else {
        wait_for_user_start_key();
    }
#endif
    g_fault_code = FAULT_NONE;
    g_rk_arm_tasks_disabled_for_route = 0U;
    g_first_arm_station_reached = 0U;
    g_rk_pretask_line_len = 0U;
    g_command_speed_m_s = 0.0f;
    g_heading_correction_rad_s = 0.0f;
    g_estimated_distance_m = 0.0f;
    g_cross_track_m = 0.0f;
    g_cross_track_command_m_s = 0.0f;
    g_actual_cross_speed_m_s = 0.0f;
    run_log_reset();

#if ROUTE_TASK_LINK_SIMULATION_ONLY
    run_task_link_simulation();
#endif

#if ROUTE_AUTO_RUN_ON_BOOT == 0U
    wait_for_usb_run_command();
#endif

#if ROUTE_WAIT_RK_READY_ON_BOOT
    while (g_rk_arm_link_ready == 0U) {
        wait_for_rk_ready_on_boot();
    }
#endif

    g_rk_pretask_last_sync_ms = HAL_GetTick() - RK_ARM_PRETASK_SYNC_PERIOD_MS;
    board_uart1_write("H7,ARM,PRETASK_SYNC_ACTIVE\r\n");

    g_run_state = RUN_BOOT;
    if (!wait_for_can_startup()) {
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
    if (!calibrate_gyro()) {
        enter_fault(FAULT_IMU_MOVING);
    }

    g_run_state = RUN_MOTOR_ENABLE;
    if (!enable_motors()) {
#if ROUTE_REQUIRE_MOTOR_ENABLE
        enter_fault(FAULT_MOTOR_ENABLE);
#else
        board_uart1_write("H7,WARN,MOTOR_ENABLE_BYPASS\r\n");
        g_fault_code = FAULT_NONE;
#endif
    }
    hold_zero(250U);
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
            HAL_Delay(10U);
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
            HAL_Delay(10U);
        }
    }
#endif

    g_yaw_rad = 0.0f;
    g_route_heading_target_rad = 0.0f;
    g_estimated_distance_m = 0.0f;

    g_run_state = RUN_STRAFE_RIGHT;
    if (!run_translation(0.0f, ROUTE_RIGHT_STRAFE_SIGN,
                         ROUTE_STRAFE_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    if (!run_relative_turn(0.0f)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_FORWARD;
    if (!run_translation_profile(ROUTE_FORWARD_SIGN, 0.0f,
                                 ROUTE_FORWARD_DISTANCE_M,
                                 ROUTE_LONG_FORWARD_SPEED_M_S,
                                 ROUTE_LONG_FORWARD_ACCEL_M_S2)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    if (!run_relative_turn(0.0f)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_TURN_TIMEOUT : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_TURN_RIGHT;
    if (!run_relative_turn(ROUTE_RIGHT_TURN_SIGN * ROUTE_TURN_ANGLE_RAD *
                           ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }

    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

#if ROUTE_TASK1_DISC_CATCH_ENABLED
    g_first_arm_station_reached = 1U;
    board_uart1_write("H7,ARM,PRETASK_SYNC_STOP_AT_DISC\r\n");
    g_run_state = RUN_ARM_DISC_CATCH;
    log_route_event(RUN_LOG_EVENT_ARM_START);
    if (!wait_for_rk_arm_task(ROUTE_TASK1_RK_ARM_TASK)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
    }
    log_route_event(g_rk_arm_tasks_disabled_for_route != 0U
                        ? RUN_LOG_EVENT_ARM_BYPASS
                        : RUN_LOG_EVENT_ARM_DONE);
#endif

    g_run_state = RUN_REVERSE_AFTER_DISC;
    if (!run_translation(-ROUTE_FORWARD_SIGN, 0.0f,
                         ROUTE_AFTER_DISC_REVERSE_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_SECOND_TURN_RIGHT;
    if (!run_relative_turn(ROUTE_RIGHT_TURN_SIGN * ROUTE_TURN_ANGLE_RAD *
                           ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }

    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_PLATFORM_APPROACH;
    if (!run_translation(ROUTE_FORWARD_SIGN, 0.0f,
                         ROUTE_PLATFORM_APPROACH_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_THIRD_TURN_RIGHT;
    if (!run_relative_turn(ROUTE_RIGHT_TURN_SIGN * ROUTE_TURN_ANGLE_RAD *
                           ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }

    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
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
            log_route_event(RUN_LOG_EVENT_ARM_START);
            if (!wait_for_rk_arm_task(ROUTE_TASK2_RK_ARM_TASK)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
            }
            log_route_event(g_rk_arm_tasks_disabled_for_route != 0U
                                ? RUN_LOG_EVENT_ARM_BYPASS
                                : RUN_LOG_EVENT_ARM_DONE);
            if (platform_index + 1U < ROUTE_TASK2_PLATFORM_PICK_COUNT) {
                const float shift_distance_m =
                    platform_index == 0U
                        ? ROUTE_TASK2_PLATFORM_FIRST_SHIFT_M
                        : ROUTE_TASK2_PLATFORM_SECOND_SHIFT_M;

                hold_zero(ROUTE_SEGMENT_SETTLE_MS);
                if (g_run_state == RUN_FAULT) {
                    enter_fault(g_fault_code);
                }
                g_run_state = RUN_PLATFORM_SHIFT_LEFT;
                if (!run_translation(0.0f, -ROUTE_RIGHT_STRAFE_SIGN,
                                     shift_distance_m)) {
                    enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
                }
                hold_zero(ROUTE_SEGMENT_SETTLE_MS);
                if (g_run_state == RUN_FAULT) {
                    enter_fault(g_fault_code);
                }
            }
        }
    }
#endif

    g_run_state = RUN_DIAGONAL_AFTER_PLATFORM;
    if (!run_translation(
            -ROUTE_FORWARD_SIGN * ROUTE_AFTER_PLATFORM_REVERSE_COMPONENT_M,
            -ROUTE_RIGHT_STRAFE_SIGN * ROUTE_AFTER_PLATFORM_LEFT_COMPONENT_M,
            ROUTE_AFTER_PLATFORM_DIAGONAL_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_LAST_TURN_RIGHT;
    if (!run_relative_turn(ROUTE_RIGHT_TURN_SIGN * ROUTE_TURN_ANGLE_RAD *
                           ROUTE_GYRO_TURN_SCALE)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    {
        bool orbit_arm_started = false;

#if ROUTE_TASK3_COLUMN_CATCH_ENABLED
        g_run_state = RUN_ARM_COLUMN_CATCH;
        log_route_event(RUN_LOG_EVENT_ARM_START);
        orbit_arm_started = start_rk_arm_task(ROUTE_TASK3_RK_ARM_TASK);
        log_route_event(orbit_arm_started ? RUN_LOG_EVENT_ARM_DONE
                                          : RUN_LOG_EVENT_ARM_BYPASS);
#endif

    g_run_state = RUN_FRONT_CENTER_ORBIT;
    if (!run_front_center_orbit(ROUTE_RIGHT_TURN_SIGN *
                                    ROUTE_FRONT_CENTER_ORBIT_ANGLE_RAD *
                                    ROUTE_GYRO_TURN_SCALE,
                                ROUTE_FRONT_CENTER_ORBIT_RADIUS_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

    g_run_state = RUN_FINAL_REVERSE;
    if (!run_translation(-ROUTE_FORWARD_SIGN, 0.0f,
                         ROUTE_FINAL_REVERSE_DISTANCE_M)) {
        enter_fault(g_fault_code == FAULT_NONE ? FAULT_MOTOR_COMMAND : g_fault_code);
    }
    hold_zero(ROUTE_SEGMENT_SETTLE_MS);
    if (g_run_state == RUN_FAULT) {
        enter_fault(g_fault_code);
    }

#if ROUTE_TASK3_COLUMN_CATCH_ENABLED
        if (orbit_arm_started) {
            g_run_state = RUN_ARM_COLUMN_CATCH;
            log_route_event(RUN_LOG_EVENT_ARM_STOP);
            if (!stop_rk_arm_task(ROUTE_TASK3_RK_ARM_TASK)) {
                enter_fault(g_fault_code == FAULT_NONE ? FAULT_ARM_TIMEOUT : g_fault_code);
            }
            log_route_event(RUN_LOG_EVENT_ARM_STOP_DONE);
        }
#else
        (void)orbit_arm_started;
#endif
    }

    g_run_state = RUN_SERVO_90;
    board_servo_set_angle_deg_index(0U, SERVO_MG90S_ROUTE_ANGLE_DEG);
    board_servo_set_angle_deg_index(1U, SERVO_MG90S_ROUTE_ANGLE_DEG);
    HAL_Delay(ROUTE_SERVO_SETTLE_MS);
    board_servo_set_angle_deg_index(0U, ROUTE_SERVO_INITIAL_ANGLE_DEG);
    board_servo_set_angle_deg_index(1U, ROUTE_SERVO_INITIAL_ANGLE_DEG);
    HAL_Delay(ROUTE_SERVO_SETTLE_MS);
    board_servo_disable_index(0U);
    board_servo_disable_index(1U);

    g_run_state = RUN_STOPPING;
    hold_zero(1000U);
    (void)motor_disable_all();
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

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {
    }
}
