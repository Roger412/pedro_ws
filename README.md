# PEDRO Workspace

This workspace contains the PEDRO omnibase firmware target and the ROS 2 host
tools used to command, monitor, and validate it.

## What Happened

The original `firmware/STM32H7_PEDRO` project was kept untouched. A separate
target, `firmware/STM32H7_PEDRO_OMNIBASE`, was used as the integration target
for a mecanum omnibase migration.

The migration kept PEDRO's local motor-control architecture:

- 4 DC motors driven through L298N H-bridges
- local per-wheel PID on the STM32
- quadrature encoder feedback
- USART3 serial link to ROS 2

It brought over the useful OMNIBASE pieces:

- mecanum inverse/forward kinematics and wheel sign convention
- BNO085 IMU support over I2C1 using the SH2 driver stack
- watchdog / STOP / ESTOP state handling
- extended telemetry for IMU, odometry, PWM, encoders, and robot state

The ROS 2 `serial_comm` package was updated to match the merged firmware. It
now supports `/cmd_vel`, publishes standard `imu/data` and `odom` messages,
keeps the older plotting/debug topics, and includes a PEDRO-focused web
dashboard.

## How It Was Done

The work was split into six prompt sessions:

1. `session1_analysis.md` - analyzed PEDRO, OMNIBASE, pin usage, protocols,
   conflicts, and reusable code.
2. `session2_firmware.md` - merged the firmware architecture into
   `STM32H7_PEDRO_OMNIBASE`.
3. `session3_ros2_serial.md` - updated the ROS 2 serial bridge.
4. `session4_dashboard.md` - added the PEDRO dashboard node and web UI.
5. `session5_validation.md` - reviewed firmware/ROS consistency and build
   status.
6. `session6_final_docs.md` - finalized documentation and wrote the summary.

The detailed running record is in:

- `firmware/STM32H7_PEDRO_OMNIBASE/status.md`

The concise final summary is in:

- `firmware/STM32H7_PEDRO_OMNIBASE/status.txt`

## Current Status

Software status:

- ROS 2 package build passed for `serial_comm`.
- Python syntax checks passed for the serial bridge and dashboard.
- Firmware and ROS command/telemetry formats were cross-checked.
- Firmware compile validation is still blocked by stale generated CubeIDE
  makefiles that point to old `home-custom-base` paths.
- ROS lint tests still need style/docstring cleanup.

Hardware-dependent TODOs:

- confirm wheel index to physical wheel location;
- verify motor and encoder signs with the robot lifted;
- confirm BNO085 wiring on PD14, PD15, and PA4;
- measure and update `x_off`, `y_off`, wheel radius, and encoder counts/rev;
- bench-test `DEBUG_LEDS=1` before powering motors.

## Brief Guide To `status.md`

`status.md` is the main engineering document for this migration.

Important sections:

- Sections 1-16: original analysis of PEDRO, OMNIBASE, pinouts, tasks,
  protocols, conflicts, and merge plan.
- Section 17: executed firmware merge, files added/modified, final UART
  protocol, state machine, TODOs, and compile notes.
- Section 18: ROS 2 serial bridge update.
- Section 19: PEDRO dashboard update.
- Section 20: validation pass, build/lint results, and prioritized TODOs.
- Section 21: final documentation review with the bench-test references:
  mecanum mapping, LED/protoboard procedure, ROS 2 test commands, UART example
  frames, and the complete modified/added file list.

For day-to-day use, start with section 21 and `status.txt`. Use earlier
sections when you need the reasoning, source comparisons, or migration history.
