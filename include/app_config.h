#ifndef TEST1_APP_CONFIG_H
#define TEST1_APP_CONFIG_H

/* Physical wheel mapping: ID1=FL, ID2=FR, ID3=RL, ID4=RR. */
#define USER_VERIFY_MOTOR_FL_BASE_ID 1U
#define USER_VERIFY_MOTOR_FR_BASE_ID 2U
#define USER_VERIFY_MOTOR_RL_BASE_ID 3U
#define USER_VERIFY_MOTOR_RR_BASE_ID 4U

#define DM_VELOCITY_COMMAND_ID_OFFSET 0x200U
#define FDCAN_KERNEL_CLOCK_HZ 80000000UL
#define FDCAN_NOMINAL_BITRATE_HZ 1000000UL
#define FDCAN_NOM_PRESCALER 1U
#define FDCAN_NOM_SJW 20U
#define FDCAN_NOM_SEG1 59U
#define FDCAN_NOM_SEG2 20U

/* Chassis parameters copied from the previous project; verify on the real car. */
#define WHEEL_RADIUS_M 0.050f
#define CHASSIS_HALF_LENGTH_M 0.180f
#define CHASSIS_HALF_WIDTH_M 0.160f
#define MOTOR_MAX_WHEEL_RAD_S 58.6431f
/* Scales decoded DM feedback velocity to wheel-end rad/s; calibrated from route logs. */
#define MOTOR_FEEDBACK_WHEEL_SPEED_SCALE 3.55f
#define WHEEL_FL_SIGN 1.0f
#define WHEEL_FR_SIGN -1.0f
#define WHEEL_RL_SIGN 1.0f
#define WHEEL_RR_SIGN -1.0f

/* Autonomous route: strafe right 0.8 m, drive forward 4.1 m, turn right 90 deg,
 * run the disc task, reverse 1.6 m, turn right 90 deg, drive forward 1.6 m,
 * turn right 90 deg, run three platform picks with two 0.35 m left shifts,
 * move diagonally with 0.9 m reverse and 0.1 m left components, turn right
 * 90 deg, orbit 270 deg around a point 0.5 m ahead, and reverse 0.3 m. */
#define CONTROL_PERIOD_MS 10U
#define ROUTE_AUTO_RUN_ON_BOOT 1U
/* PA5/ADC1_INP19 is the LCD joystick ladder.  Only UP/DOWN may start a
 * route: UP selects RED and DOWN selects BLUE. */
#define ROUTE_WAIT_USER_KEY_ON_BOOT 1U
#define ROUTE_USER_KEY_DEBOUNCE_MS 100U
#define ROUTE_POWER_ON_SETTLE_MS 500U
/* Start the chassis route immediately. RK synchronization runs in the
 * background until the first arm station is reached. */
#define ROUTE_WAIT_RK_READY_ON_BOOT 0U
#define RK_ARM_BOOT_READY_TIMEOUT_MS 2500U
#define RK_ARM_PRETASK_SYNC_PERIOD_MS 250U
#define CAN_STARTUP_RETRY_TIMEOUT_MS 5000U
#define CAN_STARTUP_RETRY_GAP_MS 50U
/* Airborne integration mode: keep the route state machine running even if
 * motor CAN startup/feedback is late. Restore these to 1 before ground runs. */

#define ROUTE_REQUIRE_CAN_STARTUP 1U

#define ROUTE_REQUIRE_MOTOR_ENABLE 1U

#define ROUTE_REQUIRE_MOTOR_FEEDBACK 1U

#define ROUTE_REQUIRE_MOTOR_TX_SUCCESS 1U

#define ROUTE_STRAFE_DISTANCE_M 0.800f
#define ROUTE_FORWARD_DISTANCE_M 4.100f
#define ROUTE_AFTER_DISC_REVERSE_DISTANCE_M 1.600f
#define ROUTE_PLATFORM_APPROACH_DISTANCE_M 1.600f
#define ROUTE_AFTER_PLATFORM_REVERSE_COMPONENT_M 0.900f
#define ROUTE_AFTER_PLATFORM_LEFT_COMPONENT_M 0.100f
#define ROUTE_AFTER_PLATFORM_DIAGONAL_DISTANCE_M 0.9055385f
#define ROUTE_FRONT_CENTER_ORBIT_RADIUS_M 0.500f
#define ROUTE_FRONT_CENTER_ORBIT_ANGLE_RAD 4.7123890f
#define ROUTE_FRONT_CENTER_ORBIT_TIMEOUT_MS 12000U
#define ROUTE_ORBIT_MAX_SPEED_RAD_S 1.000f
#define ROUTE_ORBIT_ACCEL_RAD_S2 1.800f
#define ROUTE_ORBIT_KP 1.80f
#define ROUTE_ORBIT_KD 0.25f
#define ROUTE_FINAL_REVERSE_DISTANCE_M 0.300f
#define ROUTE_SERVO_SETTLE_MS 700U
#define ROUTE_SERVO_INITIAL_ANGLE_DEG 0.0f


#define ROUTE_TRANSLATION_SPEED_M_S 1.800f
#define ROUTE_TRANSLATION_ACCEL_M_S2 2.000f
#define ROUTE_LONG_FORWARD_SPEED_M_S 1.800f
#define ROUTE_LONG_FORWARD_ACCEL_M_S2 2.000f
#define ROUTE_TRANSLATION_TIMEOUT_MS 15000U
#define DRIVE_STOP_TOLERANCE_M 0.004f
#define DRIVE_DISTANCE_SCALE 1.000f
/* FS-i6S forward stick produces negative vx on the verified chassis. */
#define ROUTE_FORWARD_SIGN -1.0f
/* Standard mecanum coordinates use negative vy and wz for rightward motion. */
#define ROUTE_RIGHT_STRAFE_SIGN 1.0f
#define ROUTE_RIGHT_TURN_SIGN 1.0f
#define ROUTE_LEFT_TURN_SIGN -1.0f
#define ROUTE_BACK_TURN_SIGN 1.0f

#define ROUTE_TURN_ANGLE_RAD 1.5707963f
#define ROUTE_HALF_TURN_ANGLE_RAD 3.1415927f
#define ROUTE_GYRO_TURN_SCALE 1.0056f
#define ROUTE_TURN_MAX_SPEED_RAD_S 2.200f
#define ROUTE_TURN_ACCEL_RAD_S2 3.800f
#define ROUTE_TURN_KP 2.20f
#define ROUTE_TURN_KD 0.20f
#define ROUTE_TURN_TOLERANCE_RAD 0.020f
#define ROUTE_TURN_RATE_TOLERANCE_RAD_S 0.040f
#define ROUTE_TURN_SETTLE_MS 80U
#define ROUTE_TURN_TIMEOUT_MS 6000U
#define ROUTE_SEGMENT_SETTLE_MS 80U

/* RK arm task points on the chassis route.
 * Task 1: after the first right 90 deg turn, run disc red/yellow ball catch.
 * Task 2: after the second right 90 deg turn, run three one-shot platform picks.
 *         The chassis shifts left 350 mm between each pick.
 * Task 3: before the orbit, expand the arm and keep detecting red balls while
 *         the chassis is moving; stop/retract after the final reverse. */
#define ROUTE_TASK1_DISC_CATCH_ENABLED 1U
#define ROUTE_TASK1_RK_ARM_TASK "DISC_CATCH"
#define ROUTE_TASK2_PLATFORM_PICK_ENABLED 1U
#define ROUTE_TASK2_RK_ARM_TASK "PLATFORM_PICK"
#define ROUTE_TASK2_PLATFORM_PICK_COUNT 3U
#define ROUTE_TASK2_PLATFORM_FIRST_SHIFT_M 0.350f
#define ROUTE_TASK2_PLATFORM_SECOND_SHIFT_M 0.350f
#define ROUTE_TASK3_COLUMN_CATCH_ENABLED 1U
#define ROUTE_TASK3_RK_ARM_TASK "COLUMN_CATCH"

#define RK_ARM_START_RETRY_MS 200U
#define RK_ARM_PROBE_ACK_TIMEOUT_MS 1800U
#define RK_ARM_ACK_TIMEOUT_MS 3000U
#define RK_ARM_WAIT_FOREVER_FOR_ACK 0U
#define RK_ARM_TASK_TIMEOUT_MS 20000U
#define RK_ARM_STATUS_PERIOD_MS 1000U
#define RK_ARM_STOP_TIMEOUT_MS 15000U
#define RK_ARM_REQUIRED 0U

/* BMI088 yaw feedback parameters. */
#define GYRO_CALIBRATION_SAMPLES 300U
#define GYRO_CALIBRATION_PERIOD_MS 3U
#define GYRO_STATIONARY_STDDEV_MAX_RAD_S 0.050f
#define GYRO_Z_SIGN -1.0f
/* BMI088-to-chassis axis map.  Verify these four values from a ground-run log
 * if the controller board is mounted in a different orientation. */
#define IMU_ACCEL_BODY_X_INDEX 0U
#define IMU_ACCEL_BODY_Y_INDEX 1U
#define IMU_ACCEL_BODY_X_SIGN -1.0f
#define IMU_ACCEL_BODY_Y_SIGN -1.0f
#define IMU_ACCEL_FILTER_ALPHA 0.20f
#define IMU_ACCEL_DEADBAND_M_S2 0.08f
#define IMU_ACCEL_MAX_M_S2 4.00f
#define IMU_VELOCITY_PREDICTION_WEIGHT 0.08f
#define IMU_VELOCITY_MAX_ENCODER_DELTA_M_S 0.15f
#define IMU_ZERO_VELOCITY_THRESHOLD_M_S 0.020f
#define HEADING_KP 7.50f
#define HEADING_KD 0.22f
#define STRAFE_HEADING_KP 5.00f
#define STRAFE_HEADING_KD 0.25f
#define HEADING_MAX_CORRECTION_RAD_S 0.70f
#define HEADING_CORRECTION_SPEED_RATIO 0.50f
/* Keep each translation on its original world-frame line.  Heading control
 * alone can straighten the chassis after it has already drifted sideways. */
#define TRANSLATION_CROSS_TRACK_KP 1.80f
#define TRANSLATION_CROSS_TRACK_KD 0.15f
#define TRANSLATION_CROSS_TRACK_MAX_M_S 0.30f
#define TRANSLATION_FIELD_ORIENT_MAX_ERROR_RAD 0.12f
#define TRANSLATION_CROSS_TRACK_SETTLE_MAX_M_S 0.18f
#define TRANSLATION_CROSS_TRACK_TOLERANCE_M 0.008f
#define TRANSLATION_CROSS_SPEED_TOLERANCE_M_S 0.030f
#define TRANSLATION_CROSS_SETTLE_MS 60U
#define TRANSLATION_CROSS_SETTLE_TIMEOUT_MS 500U
#define TRANSLATION_SETTLE_HEADING_MAX_CORRECTION_RAD_S 0.35f
/* Encoder odometry closes the along-track position loop; BMI088 gyro yaw
 * supplies the heading used to rotate wheel odometry into the route frame. */
#define ODOM_ALONG_POSITION_KP 1.20f
#define ODOM_ALONG_CORRECTION_MAX_M_S 0.25f
#define ODOM_ALONG_SPEED_KP 0.35f
#define ODOM_ALONG_SPEED_KI 0.80f
#define ODOM_ALONG_SPEED_INTEGRAL_MAX_M_S 0.25f
#define ODOM_ALONG_POSITION_TOLERANCE_M 0.008f
#define ODOM_ALONG_SPEED_TOLERANCE_M_S 0.030f
#define ODOM_ALONG_SETTLE_MS 60U

#define MOTOR_TX_DRAIN_TIMEOUT_MS 200U

/* FS-i6S receiver on P6 / UART5.  CH5 high takes over the chassis with RC;
 * when CH5 goes low or the receiver is lost, control returns to the start gate. */
#define ROUTE_RC_OVERRIDE_ENABLED 1U
#define IBUS_OVER_P6 1
#define SBUS_OVER_P6 2
#ifndef RC_PROTOCOL_MODE
#define RC_PROTOCOL_MODE SBUS_OVER_P6
#endif
#define RC_CH_VX_INDEX 1U
#define RC_CH_VY_INDEX 3U
#define RC_CH_WZ_INDEX 0U
#define RC_CH_UNLOCK_INDEX 4U
#define RC_CH_SPEED_INDEX 5U
#define RC_VX_DIRECTION 1.0f
#define RC_VY_DIRECTION 1.0f
#define RC_WZ_DIRECTION 1.0f
#define RC_CHANNEL_MIN_US 1000U
#define RC_CHANNEL_CENTER_US 1500U
#define RC_CHANNEL_MAX_US 2000U
#define RC_STICK_DEADZONE_US 80U
#define RC_ARM_CENTER_WINDOW_US 80U
#define RC_UNLOCK_LOW_MAX_US 1200U
#define RC_UNLOCK_HIGH_MIN_US 1800U
#define RC_SPEED_LOW_MAX_US 1200U
#define RC_SPEED_HIGH_MIN_US 1800U
#define RC_SPEED_LOW_SCALE 0.35f
#define RC_SPEED_MID_SCALE 0.65f
#define RC_SPEED_HIGH_SCALE 1.00f
#define RC_AXIS_EXPO 0.20f
#define RC_TIMEOUT_MS 300U
#define RC_OVERRIDE_ALLOW_HIGH_ON_BOOT 1U
#define RC_OVERRIDE_RELEASE_CONFIRM_MS 600U
#define RC_OVERRIDE_MAX_LINEAR_M_S 1.000f
#define RC_OVERRIDE_MAX_ANGULAR_RAD_S 2.000f
#define RC_OVERRIDE_WHEEL_RADIUS_M 0.050f
#define RC_OVERRIDE_HALF_LENGTH_M 0.180f
#define RC_OVERRIDE_HALF_WIDTH_M 0.160f
#define RC_OVERRIDE_WHEEL_FL_SIGN 1.0f
#define RC_OVERRIDE_WHEEL_FR_SIGN -1.0f
#define RC_OVERRIDE_WHEEL_RL_SIGN 1.0f
#define RC_OVERRIDE_WHEEL_RR_SIGN -1.0f

/*
 * MG90S PWM outputs on the DM-MC-Board02 expansion header:
 * servo 0: PA0 / TIM2_CH1
 * servo 1: PA2 / TIM2_CH3
 * servo 2: PE9 / TIM1_CH1
 * servo 3: PE13 / TIM1_CH3
 */
#define SERVO_PWM_CHANNEL_COUNT 4U
#define SERVO_PWM_TIMER_HZ 1000000UL
#define SERVO_PWM_PERIOD_US 20000U
#define SERVO_MG90S_MIN_PULSE_US 500U
#define SERVO_MG90S_MAX_PULSE_US 2500U
#define SERVO_MG90S_MAX_ANGLE_DEG 180.0f
#define SERVO_MG90S_ROUTE_ANGLE_DEG 95.0f

#endif
