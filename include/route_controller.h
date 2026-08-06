#ifndef TEST1_ROUTE_CONTROLLER_H
#define TEST1_ROUTE_CONTROLLER_H

#include "app_config.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RUN_BOOT = 0,
    RUN_IMU_INIT,
    RUN_GYRO_CALIBRATION,
    RUN_MOTOR_ENABLE,
    RUN_STRAFE_RIGHT,
    RUN_STOPPING = 5,
    RUN_DONE = 6,
    RUN_FAULT = 7,
    RUN_FORWARD = 8,
    RUN_TURN_RIGHT = 9,
    RUN_REVERSE_AFTER_DISC = 10,
    RUN_SECOND_TURN_RIGHT = 11,
    RUN_PLATFORM_APPROACH = 12,
    RUN_ROUTE_RESERVED_13 = 13,
    RUN_THIRD_TURN_RIGHT = 14,
    RUN_WAIT_USB_RUN = 15,
    RUN_PLATFORM_SHIFT_LEFT = 16,
    RUN_DIAGONAL_AFTER_PLATFORM = 17,
    RUN_LAST_TURN_RIGHT = 18,
    RUN_FRONT_CENTER_ORBIT = 19,
    RUN_FINAL_REVERSE = 20,
    RUN_SERVO_90 = 21,
    RUN_ARM_DISC_CATCH = 22,
    RUN_ARM_PLATFORM_PICK = 23,
    RUN_ARM_COLUMN_CATCH = 24,
    RUN_RC_OVERRIDE = 25,
    RUN_STRAFE_LEFT = 26,
    RUN_TURN_LEFT = 27,
    RUN_SECOND_TURN_LEFT = 28,
    RUN_THIRD_TURN_LEFT = 29,
    RUN_PLATFORM_SHIFT_RIGHT = 30,
    RUN_LAST_TURN_LEFT = 31
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
    FAULT_ARM_TIMEOUT = 8,
    FAULT_RC_OVERRIDE = 9
};

extern volatile run_state_t g_run_state;
extern volatile uint32_t g_fault_code;
extern volatile uint8_t g_bmi088_init_error;
extern volatile float g_gyro_z_rad_s;
extern volatile float g_yaw_rad;
extern volatile float g_command_speed_m_s;
extern volatile float g_heading_correction_rad_s;
extern volatile float g_estimated_distance_m;
extern volatile float g_imu_temperature_c;
extern volatile float g_cross_track_m;
extern volatile float g_cross_track_command_m_s;
extern volatile float g_actual_cross_speed_m_s;

void route_controller_init(void);
void route_controller_wait_for_start(void);
void route_controller_set_field(uint8_t is_red);
void route_controller_reset_run_context(void);
void route_controller_begin_pretask_sync(void);
void route_controller_mark_first_arm_station(void);
void route_controller_request_rk_reset(void);
void route_controller_service_rk_link(void);
void route_controller_reset_pose(void);
uint8_t route_controller_arm_tasks_disabled(void);
uint8_t route_controller_rk_link_ready(void);

bool route_controller_wait_for_can_startup(void);
bool route_controller_calibrate_gyro(void);
bool route_controller_enable_motors(void);
void route_controller_hold_zero(uint32_t duration_ms);
void route_controller_enter_fault_wait_restart(uint32_t code);
void route_controller_log_event(uint32_t event);

bool route_controller_run_translation_profile(float vx_direction,
                                               float vy_direction,
                                               float target_distance_m,
                                               float maximum_speed_m_s,
                                               float acceleration_m_s2);
bool route_controller_run_translation(float vx_direction,
                                      float vy_direction,
                                      float target_distance_m);
bool route_controller_run_relative_turn(float angle_rad);
bool route_controller_run_front_center_orbit(float angle_rad,
                                             float center_distance_m);
bool route_controller_wait_for_rk_arm_task(const char *task);
bool route_controller_start_rk_arm_task(const char *task);
bool route_controller_stop_rk_arm_task(const char *task);

#if ROUTE_AUTO_RUN_ON_BOOT == 0U
void route_controller_wait_for_usb_run_command(void);
#endif

#if ROUTE_WAIT_RK_READY_ON_BOOT
void route_controller_wait_for_rk_ready_on_boot(void);
#endif

#endif
