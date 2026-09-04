# H7/RK Joint RoboCup Workflow

## Ownership

- H7: LCD joystick field selection, CAN motor control, IMU/odometry, route
  motion, USB CDC requests, RC override, and persistent run log.
- RK: camera capture, white-line measurement, field-aware target policy, arm
  poses, direct servo/C8T6 commands, and sequence-aware replies.
- Main camera: arm-mounted camera for white-line, ball, ring, and platform
  target recognition.
- Secondary camera: task-two entry only; records two distinct letters from
  A/B/C/D. It is closed after `PRESELECT_DONE`.
- `ABCD_detector`: A/B/C/D letters. `balls_detector`: red, blue, and yellow
  balls. Do not restore QR, old red/blue blocks, or white-ball primary logic.

## Start and reset

1. Release the LCD joystick, then select RED by moving RIGHT or BLUE by moving
   DOWN. H7 logs both the decoded direction and raw ADC value.
2. H7 sends `FIELD,RED` or `FIELD,BLUE` over USB CDC.
3. H7 sends `ARM,SYNC,RESET,FIELD,<FIELD>` until RK completes the home/reset
   transaction. H7 does not start the route before reset confirmation.
4. RK reset homes the arm and replies:

```text
RK,ARM,RESET,DONE,FIELD,<RED|BLUE>
```

Camera availability does not gate RESET or `PREP_HIGH`. A camera failure may
prevent visual alignment or target pickup, but must not prevent the arm from
being raised during the disc arc.

## Task-one route

1. H7 starts `DISC_CATCH,PREP_HIGH` before the first arc command and resends it
   non-blockingly while the arc is running.
2. The mirrored cubic arc ends at the configured forward endpoint
   `ROUTE_FORWARD_DISTANCE_M=3.930 m` and lateral endpoint `0.650 m`.
   Arc maximum speed is `1.40 m/s`; acceleration is `0.50 m/s^2`.
3. RK applies `PREP_HIGH`: 85KG ID1/ID2/ID6 are `600/500/670`, ZP splitter
   ID4 is `1200`, ZP ID5 is `800`, and ZP ID7 is closed at `1300`.
4. After the arc, H7 queries the main camera for the white line. The reference
   is `Y10=1500 +/- 100` in an `800x600` frame and `A100=0`.
5. H7 continues at `0.03 m/s` while the line is below the reference. If the
   line is not found, H7 uses the configured fallback before continuing. After
   reaching the reference, the formal route advances a further `250 mm` and
   stops line tracking.
6. H7 starts `DISC_CATCH` after the line station is reached. RK filters balls
   by field: RED accepts red/yellow; BLUE accepts blue/yellow. The opponent
   color must not trigger the splitter or gripper.
7. When DISC_CATCH completes, RK closes ID7 and retracts ZP ID5 plus 85KG
   ID1/ID2/ID6 to home. H7 keeps that safe retracted state during the formal
   task-two transfer: one continuous `1.30 m` backward / `2.25 m` lateral /
   `174 deg` diagonal motion. Only after the transfer completes does H7
   request PREP_HIGH for task two. RK then raises only 85KG `ID1=600,
   ID2=500, ID6=650`; ZP ID5 remains at its home value `900` and is not
   expanded again.

## Task-two route

1. H7 enters task two with one continuous diagonal segment. In the fixed route
   frame it moves `1.30 m` backward and `2.25 m` toward the selected field side,
   for a commanded diagonal magnitude of approximately `2.599 m`.
2. During that diagonal segment, H7 smoothly rotates the chassis through `174
   degrees`: RED turns left and BLUE turns right. There are no intermediate
   stops for two separate 90-degree turns.
3. After the diagonal stops, H7 reuses the task-one BMI088 heading correction
   and closes any residual yaw error to the saved final `174 deg` heading.
   Task-two arm expansion and camera work begin only after this correction.
3. H7 sends:

```text
ARM,PLATFORM_PICK,PRESELECT,SEQ,<N>,FIELD,<FIELD>,COUNT,2
```

4. RK holds the high platform pose and uses only the secondary camera to
   collect two distinct letters from A/B/C/D. Duplicate observations do not
   count. The preselection has a bounded timeout; a missing secondary camera
   returns a controlled `ERR` instead of blocking H7 indefinitely.
5. RK replies with:

```text
RK,ARM,PLATFORM_PICK,PRESELECT_ACK,SEQ,<N>
RK,ARM,PLATFORM_PICK,PRESELECT_DONE,SEQ,<N>,COUNT,2,LETTER1,<A-D>,LETTER2,<A-D>
```

6. H7 first performs the task-two station entry used by the standalone
   commissioning path: field-mirrored lateral shift `400 mm`, then main-camera
   white-line alignment at `Y10=2000 +/- 100` with `0.10 m/s` approach speed
   and `0.10 m/s^2` acceleration, followed by one fixed `210 mm` forward
   approach. White-line tracking is then disabled. H7 runs eight slots.
   Between slots the right/left shifts are `100, 130, 100, 100, 100, 130,
   100 mm`, mirrored by field, at `0.50 m/s`.
7. For each slot H7 sends:

```text
ARM,PLATFORM_PICK,START,SEQ,<N>,FIELD,<FIELD>,SLOT,<1-8>
```

8. After preselection, only the main camera recognizes each slot. RK accepts
   either one of the two locked letters or the own-field ring. Unselected
   letters and the opponent-field ring are skipped while the arm remains in
   the expanded pose.
9. A successful letter/ring pickup uses the current platform arm sequence,
   including the configured descent, ID7 pulse, ID6 recovery, and return to
   the expanded pose before the next slot. Every slot returns `DONE` or a
   bounded skip outcome with the same `SEQ`.

## Task-three handoff

After slot eight, H7 moves straight backward `0.900 m`, turns `90 deg` in
the field-mirrored direction, and starts `COLUMN_CATCH` asynchronously before
the orbit. H7 sends `STOP` after the orbit and final reverse, then waits for
RK to retract the arm and return `DONE`. The H7 route log and RK log must
contain the same task and sequence.

After the formal task-three tail, H7 requests RK to command ZL channels `S12`
and `S23` to `1500` for `1000 ms`, moves laterally `500 mm` in the mirrored
direction between them, turns in place `180 deg`, and requests physical ZL
`ID3` to move from `900` to `1300` in `400 ms`. These requests use
`ARM,AUX_ZP,SET,SEQ,...`; the standalone tests are excluded from this tail.

## Protocol contract

All line messages are newline-terminated ASCII over H7 USB CDC:

```text
H7 -> RK  ARM,<TASK>,START,SEQ,<N>,FIELD,<FIELD>
RK -> H7  RK,ARM,<TASK>,ACK,SEQ,<N>,FIELD,<FIELD>
H7 -> RK  ARM,<TASK>,STATUS,SEQ,<N>
RK -> H7  RK,ARM,<TASK>,DONE,SEQ,<N>,REASON,<REASON>,FIELD,<FIELD>
RK -> H7  RK,ARM,<TASK>,ERR,SEQ,<N>,REASON,<REASON>,FIELD,<FIELD>
```

`PREP_HIGH` is an asynchronous pose request and may omit `SEQ` for backward
compatibility. H7 accepts replies only for the active task/sequence. RK
replays the last completed result for duplicate `START`, `STATUS`, or
`PRESELECT` requests. `PRESELECT` is sequence-aware and opens the fixed
secondary-camera path only for its bounded preselection transaction; it does
not reuse the main-camera task loop.

## Verification

Before a ground run:

1. Build RK with `colcon build --symlink-install --packages-select ros2_test1`
   and run the protocol/station tests.
2. Build H7 and inspect `app_config.h` and the generated ELF/BIN.
3. Confirm exactly one process owns the H7 USB CDC device and that the 85KG
   bus is `/dev/ttyS9` through C8T6 while the ZP bus is `/dev/ttyS0`.
4. Confirm logs contain matching `FIELD`, `SEQ`, `ACK`, `DONE`/`ERR`, and slot
   numbers. Do not treat stale log lines as a new run.
5. Keep the chassis lifted for the first protocol and arm-pose test. Flash H7
   only after the build is verified, then require a real flash readback
   comparison before reporting success.
