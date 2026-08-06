# H7/RK Production Workflow

## Code ownership

- H7 route parameters and production switches: `include/app_config.h`
- H7 route, motion sequencing, and RK handshake: `src/route_controller.c`
- H7 persistent run log: `src/run_log.c` and `include/run_log.h`
- RK station protocol and arm state machines:
  `/home/cat/ros2_ws/ros2_test1/ros2_test1/target_vision.py`
- RK production launch profile:
  `/home/cat/ros2_ws/ros2_test1/ros2_test1/launch_common.py`
- RK boot service: `robocup-chassis-arm.service`

The production route has one H7 entry point, `route_controller_run()`. The old
`extend.c`/`extend.h` route names are no longer part of the build.

## Boot contract

1. H7 initializes CAN and BMI088, calibrates yaw, enables the motors, and
   starts the production route without waiting for RK to finish booting.
2. Before the first arm station, H7 sends `ARM,SYNC` every 250 ms while its
   normal motion control continues. RK may answer `RK,ARM,READY` at any time.
3. RK systemd waits until `/dev/video20` can deliver a real frame and until
   `/dev/ttyS9` and `/dev/ttyS0` exist. It then homes the arm (`ID1=480`,
   `ID2=10`, `ID6=500`, `ID7=1120`, `ID4=800`, `ID5=800`) and opens the camera.
4. At the first arm station H7 stops the chassis and sends
   `ARM,DISC_CATCH,START` until RK acknowledges it. With the current
   fail-open test configuration, a missing ACK bypasses the station after the
   configured timeout and the chassis continues. If RK acknowledges the task,
   H7 waits for DONE, with a 20 second task timeout.

`ROUTE_TASK_LINK_SIMULATION_ONLY` in `include/app_config.h` must be `0` for the
production route. Set it to `1` only for an elevated communication test.

## Station protocol

Commands are newline-terminated ASCII over H7 USB CDC.
RK uses the udev symlink `/dev/h7_chassis`; the nonblocking serial setup keeps
camera processing running while H7 is disconnected or halted.

- Start: `ARM,<TASK>,START`
- Start acknowledgement: `RK,ARM,<TASK>,ACK`
- Liveness query: `ARM,<TASK>,STATUS`
- Completion: `RK,ARM,<TASK>,DONE,<REASON>`
- Asynchronous stop: `ARM,COLUMN_CATCH,STOP`

H7 retries START for a bounded period. After ACK it keeps the chassis stopped
and sends a STATUS query every second until DONE or the 20 second task timeout.
If RK restarts and reports the task IDLE, H7 restarts that station from START.

## Route tasks

- `DISC_CATCH`: after the first 90 degree right turn. RK expands to
  `ID1=600, ID2=350`, opens catcher ID5 to `1100`, descends to `ID1=520`,
  pulses ID7 for red or yellow balls, and sets splitter ID4 to `1600` for a
  yellow ball or `800` for a red ball before the ID7 pulse. After two seconds
  without a valid ball, RK returns ID5 to `800`, homes the arm, and only then
  returns DONE so H7 can leave the station.
- `PLATFORM_PICK`: H7 sends three separate tasks and waits for each DONE before
  the next left shift. Each task performs at most one red square, ring, or QR
  grasp. A two-second no-target timeout applies only while searching/centering.
- `COLUMN_CATCH`: RK expands and descends as above, but only red balls trigger
  ID7. Splitter ID4 is held at `800` throughout this task. It runs during the
  orbit and final reverse, then homes and returns DONE when H7 sends STOP.

## Production route

1. Both controllers power on. RK homes the arm, opens `/dev/video20`, and
   announces READY. H7 remains stationary until READY is received.
2. H7 initializes CAN and BMI088, calibrates the gyro, and enables all four
   chassis motors.
3. Strafe right 0.8 m while holding absolute heading 0 degrees, then correct
   the heading to 0 degrees in place.
4. Drive forward 4.1 m while holding 0 degrees. This long segment uses a
   2.2 m/s speed limit and 1.6 m/s2 acceleration limit. Correct to 0 degrees
   again at the end.
5. Turn right to approximately 90 degrees, stop, run `DISC_CATCH`, and wait
   for RK DONE.
6. Reverse 1.6 m while holding approximately 90 degrees, turn right to
   approximately 180 degrees, then drive forward 1.6 m.
7. Turn right to approximately 270 degrees and run the first `PLATFORM_PICK`.
8. Strafe left 0.35 m and run the second `PLATFORM_PICK`; strafe left another
   0.35 m and run the third `PLATFORM_PICK`.
9. Move diagonally by combining 0.9 m reverse and 0.1 m left, then turn right
   to a cumulative heading of approximately 360 degrees.
10. Start `COLUMN_CATCH` asynchronously. Orbit right through 270 degrees around
    a point 0.5 m in front of the chassis, then reverse 0.3 m while RK keeps
    detecting red balls.
11. Send `COLUMN_CATCH,STOP`; RK homes the arm and returns DONE.
12. Move both H7 MG90S outputs to 95 degrees, return them to 0 degrees, and
    disable PWM.
13. Stop and disable the chassis motors, then save and print the run log.

General translation uses a 2.6 m/s speed limit and 2.0 m/s2 acceleration
limit. Turns use a 2.2 rad/s speed limit and 3.8 rad/s2 acceleration limit.

## Expected logs

The RK log must show, in order, successful startup-home servo writes, camera
open, and stable vision FPS. When H7 is connected it must then show `CHASSIS
LINK open /dev/h7_chassis`, `CHASSIS RX`, station START, and matching station
DONE lines. `CHASSIS LINK waiting for /dev/h7_chassis` is expected while H7 is
powered off or halted.

The H7 UART log records every USB command and response. The persistent flash
log records route state, fault code, yaw, distance, wheel feedback, cross-track
correction, ARM start/done/stop events, and the final route-done event. A clean
run ends with fault code 0 and `RUN_LOG_EVENT_ROUTE_DONE`.

## Current diagnostic result

The latest persisted H7 run log reached the first platform station:

- `DISC_CATCH` was entered and recorded `ARM_BYPASS` because RK did not answer.
- `PLATFORM_PICK` was also bypassed because the route had already disabled
  arm tasks after the missing RK link.
- The first `PLATFORM_SHIFT_LEFT` began, then the route ended with
  `FAULT_MOTOR_COMMAND` at an estimated total distance of about 8.37 m.

Therefore the latest stop is not an RK wait. It is in the first 0.35 m
platform shift or its motor-command/feedback path.

## Failure behavior

- No RK during the opening route: H7 continues to the first station while
  sending SYNC.
- No task ACK: the current fail-open test configuration logs a bypass and
  continues the route.
- Lost task DONE: H7 sends STATUS queries until the task timeout, then logs a
  bypass and continues.
- RK servo startup failure: the RK process exits instead of announcing READY;
  systemd restarts it and H7 continues waiting.
- Ctrl+C, process exit, task timeout, and COLUMN STOP use the same home contract.

RK writes the combined vision/protocol log to
`/home/cat/ros2_ws/chassis_arm_link.log`. Logrotate keeps five compressed
history files and rotates each log at 5 MiB.
