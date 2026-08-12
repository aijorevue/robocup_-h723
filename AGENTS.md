# Project Instructions

- For RoboCup automatic sorting robot work, including H7 chassis route control,
  RK3588 vision/arm integration, red/blue field mirroring, target recognition,
  disc/platform/column task sequencing, storage-zone scoring, and rule
  compliance, use `$auto-sorting-robocup-2026` before planning or editing.
- Treat the 2026 automatic sorting rule book summarized by that skill as the
  rule source for this project. Do not reintroduce old 2025 QR/color-block task
  assumptions unless the user explicitly asks for a legacy test mode.
- Keep H7 responsible for chassis state machine, CAN/motor control, BMI088
  yaw/odometry loops, LCD/RC input, and USB task commands.
- Keep RK responsible for camera detection, target policy, arm poses,
  gripper/splitter/catcher commands, and H7 task acknowledgements.
