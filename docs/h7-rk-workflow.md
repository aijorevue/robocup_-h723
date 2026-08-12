# H7/RK Production Workflow

## Ownership and entry points

- H7 field selection and route sequence: `src/main.c`
- H7 motion control and RK protocol: `src/route_controller.c`
- H7 route parameters: `include/app_config.h`
- H7 persistent run log: `src/run_log.c` and `include/run_log.h`
- RK serial protocol: `/home/cat/ros2_ws/ros2_test1/ros2_test1/chassis_link.py`
- RK station and vision state machines:
  `/home/cat/ros2_ws/ros2_test1/ros2_test1/target_vision.py`
- RK launch profiles:
  `/home/cat/ros2_ws/ros2_test1/ros2_test1/launch_common.py`
- Unified RK launcher: `/home/cat/ros2_ws/start_target_vision.sh`
- RK boot service: `robocup-chassis-arm.service`

The normal RK command starts the linked profile:

```bash
RED_SQUARE_EXECUTE=true /home/cat/ros2_ws/start_target_vision.sh
```

Use `VISION_PROFILE=standalone` only for manual vision/arm testing without H7.

## Boot and start contract

1. RK systemd waits for the camera to provide a real frame. The launch script
   also waits for `/dev/ttyS9` (85 kg bus) and `/dev/ttyS0` (ZP bus).
2. RK homes the arm to `ID1=480`, `ID2=10`, `ID6=500`, `ID7=1120`,
   splitter `ID4=800`, and catcher `ID5=800` before announcing READY.
3. H7 waits at the start gate. LCD joystick UP selects RED; DOWN selects BLUE.
4. H7 starts CAN/BMI088 initialization and the route without blocking on RK.
   Before the first arm station it sends `ARM,SYNC,RESET,FIELD,<FIELD>` every
   250 ms. Motion control continues while RK homes or finishes booting.
5. At an arm station H7 commands zero wheel speed before sending START.

The H7 standalone policy is fail-open only before RK accepts a task. If no RK
ACK arrives, that station is bypassed and the next station probes RK again.
Once RK has acknowledged a task, a task timeout, remote ERR, or failed COLUMN
STOP is a fault because the arm may no longer be safe for chassis motion.

## Protocol version 2

The link is newline-terminated ASCII over H7 USB CDC. RK opens the stable udev
name `/dev/h7_chassis`. Every task transaction has a non-zero 32-bit sequence
number so delayed replies from one `PLATFORM_PICK` cannot complete the next.

```text
H7 -> RK  ARM,<TASK>,START,SEQ,<N>,FIELD,<RED|BLUE>
RK -> H7  RK,ARM,<TASK>,ACK,SEQ,<N>,FIELD,<RED|BLUE>
H7 -> RK  ARM,<TASK>,STATUS,SEQ,<N>
RK -> H7  RK,ARM,<TASK>,DONE,SEQ,<N>,REASON,<REASON>,FIELD,<FIELD>
RK -> H7  RK,ARM,<TASK>,ERR,SEQ,<N>,REASON,<REASON>,FIELD,<FIELD>
H7 -> RK  ARM,COLUMN_CATCH,STOP,SEQ,<N>
```

RK returns `BUSY,SEQ,<N>,REASON,STARTUP` while startup is incomplete and
`BUSY,SEQ,<N>,REASON,RESET` while homing. H7 keeps the chassis stopped and
allows up to `RK_ARM_BUSY_TIMEOUT_MS` for that state.

Duplicate handling is idempotent:

- Repeated START for the active sequence returns the same ACK.
- Repeated START or STATUS for a completed sequence replays its DONE or ERR.
- A response with another sequence is ignored by H7.
- Reconnecting RK clears a partial serial line but preserves the active or
  last-completed transaction so H7 STATUS can recover it.

## Field behavior

The distance magnitudes are shared; direction and target color are mirrored.

| Behavior | BLUE field | RED field |
| --- | --- | --- |
| Initial strafe | Right 0.8 m | Left 0.8 m |
| Route turns | Right | Left |
| Platform shifts | Left 0.35 m twice | Right 0.35 m twice |
| DISC target | Blue or yellow ball | Red or yellow ball |
| PLATFORM target | Blue block/ring/QR | Red block/ring/QR |
| COLUMN target | Blue ball | Red ball |
| Final orbit | Right 270 degrees | Left 270 degrees |

Both red and blue detectors run every detection cycle. Field selection filters
which detections may trigger a task; it does not merely recolor the display.

## Route sequence

1. Select field with the LCD joystick and release it.
2. Strafe 0.8 m toward the selected field side and correct yaw to 0 degrees.
3. Drive forward 4.1 m and correct yaw again.
4. Turn 90 degrees toward the selected field side.
5. Run `DISC_CATCH` and wait for DONE or an offline bypass.
6. Reverse 1.6 m, turn another 90 degrees, and drive forward 1.6 m.
7. Turn another 90 degrees and run the first `PLATFORM_PICK`.
8. Shift 0.35 m toward the platform direction and run the second pick.
9. Shift another 0.35 m and run the third pick.
10. Move diagonally using 0.9 m reverse plus 0.1 m mirrored lateral motion.
11. Turn another 90 degrees and start `COLUMN_CATCH` asynchronously.
12. Orbit 270 degrees around a point 0.5 m ahead, then reverse 0.3 m.
13. Send COLUMN STOP and wait until RK has homed the arm.
14. Move both H7 MG90S outputs to 95 degrees, return to 0 degrees, disable
    PWM, stop the motors, and save the run log.

Current limits are 1.8 m/s and 2.0 m/s^2 for translation, 2.2 rad/s and
3.8 rad/s^2 for turns, and 1.0 rad/s for the orbit.

## RC takeover continuity

- CH5 high owns the chassis from any route, arm-wait, fault-wait, or final
  servo-settle state.
- A parser, UART, lost-frame, or failsafe event does not by itself inject a
  zero-speed command. During a short interruption H7 replays the last valid
  chassis command and reports `source=HOLD` in the RC status log.
- A fresh CH5-low frame stops immediately and releases ownership after the
  configured confirmation interval. Sustained signal loss also stops and
  releases ownership after its bounded timeout.
- H7 continuously drains DM motor feedback while holding zero, enabling,
  disabling, and driving under RC. This prevents the FDCAN RX FIFO from
  filling while the autonomous odometry loop is paused.
- If a CAN zero, enable, drive, or disable transaction fails, H7 keeps motor
  ownership and retries the safe transition instead of reporting a false
  release.

## Task behavior

- `DISC_CATCH`: move to `ID1=600`, `ID2=350`, catcher `ID5=1100`, then
  descend to `ID1=520`. A yellow ball sets splitter ID4 to 1600; a field-color
  ball sets it to 800. ID7 pulses once per newly observed ball. Two seconds
  without a valid ball homes the arm and completes the task.
- `PLATFORM_PICK`: each transaction performs at most one field-color block,
  ring, or QR grasp. Two seconds without a fresh target homes the arm and
  completes that transaction with `NO_TARGET_TIMEOUT`.
- `COLUMN_CATCH`: splitter ID4 stays at 800. ID7 pulses once per newly observed
  field-color ball while the chassis orbits. The task ends only after H7 STOP
  and a successful home command.

## Log interpretation

H7 UART logs include the task and `SEQ=<N>` on START, ACK, DONE, bypass, ERR,
and timeout records. The persistent run log uses:

- `RUN_LOG_EVENT_ARM_START`: station entered
- `RUN_LOG_EVENT_ARM_ACK`: asynchronous COLUMN task accepted
- `RUN_LOG_EVENT_ARM_DONE`: synchronous station completed
- `RUN_LOG_EVENT_ARM_BYPASS`: RK did not accept that station
- `RUN_LOG_EVENT_ARM_STOP_DONE`: COLUMN task homed successfully

RK writes protocol and vision output to
`/home/cat/ros2_ws/chassis_arm_link.log`. A normal transaction contains the
same task name and sequence in RX, ACK, and DONE. `FAULT_ARM_TIMEOUT` means an
accepted task or STOP did not finish in time. `FAULT_ARM_REMOTE` means RK
explicitly reported a servo, homing, startup-command, or control failure.

## Verification before a ground run

1. Keep the chassis lifted and leave servo power available with an emergency
   stop reachable.
2. Run RK protocol unit tests and build the ROS package.
3. Build H7 and confirm there are no compiler errors.
4. Start RK with `RED_SQUARE_EXECUTE=false` for a protocol-only test.
5. Select RED, then BLUE, and confirm H7 logs opposite strafe/turn states and
   RK logs matching field target policies.
6. Repeat three `PLATFORM_PICK` transactions and confirm their sequence numbers
   are different and each DONE matches only its own sequence.
7. Only then enable servo writes and perform the complete ground route.
