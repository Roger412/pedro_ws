# PEDRO Workspace

This workspace contains the PEDRO omnibase firmware target and the ROS 2 host
tools used to command, monitor, and validate it.

## STM32 Project Quick Reference

Target firmware:

- `firmware/STM32H7_PEDRO_OMNIBASE`
- MCU: STM32H755ZITx, CM7 firmware
- Original baseline kept untouched: `firmware/STM32H7_PEDRO`

### Pinout

Motor direction pins, L298N inputs:

| Motor index | H-bridge pins | Firmware call |
|---|---|---|
| M0 / `u[0]` | PD4, PD5 | `setMotorDirection(GPIOD, 4, 5, ...)` |
| M1 / `u[1]` | PD6, PD7 | `setMotorDirection(GPIOD, 6, 7, ...)` |
| M2 / `u[2]` | PE2, PE4 | `setMotorDirection(GPIOE, 2, 4, ...)` |
| M3 / `u[3]` | PE3, PE6 | `setMotorDirection(GPIOE, 3, 6, ...)` |

Motor PWM / L298N enable pins:

| Motor index | PWM pin | Timer channel |
|---|---|---|
| M0 / `u[0]` | PA0 | TIM5_CH1 |
| M1 / `u[1]` | PB14 | TIM12_CH1 |
| M2 / `u[2]` | PF9 | TIM14_CH1 |
| M3 / `u[3]` | PE5 | TIM15_CH1 |

Encoder pins:

| Motor index | Encoder timer | Pins |
|---|---|---|
| M0 / `u[0]` | TIM1 CH1/CH2 | PE9, PE11 |
| M1 / `u[1]` | TIM2 CH1/CH2 | PA15, PB3 |
| M2 / `u[2]` | TIM4 CH1/CH2 | PD12, PD13 |
| M3 / `u[3]` | TIM8 CH1/CH2 | PC6, PC7 |

Other relevant pins:

| Function | Pins | Notes |
|---|---|---|
| ROS 2 serial link | USART3 PD8 TX, PD9 RX | 115200 8-N-1, ST-Link VCP |
| BNO085 I2C | PB6 SCL, PB7 SDA | I2C1 |
| BNO085 INT | PD14 | Reuses original MCP2515 CS pin |
| BNO085 RST | PD15 | Active low, idle high |
| BNO085 WAKE/PS0 | PA4 | Idle high |
| Debug LED mode | PB0 LD1, PE1 LD2 | `DEBUG_LEDS=1`; LD3/PB14 skipped because PB14 is M1 PWM |

Mecanum sign convention:

- `g_wheel_sign = {-1, +1, -1, +1}`
- physical FL/FR/RL/RR assignment is still hardware TODO
- verify on the lifted robot before powered ground driving

### Bench-Test Pinout

Use this pinout when testing with LEDs, a logic probe, or an oscilloscope before
connecting motor power. For LED tests, put a 330-1k resistor in series with each
LED and connect the LED cathode to GND.

Safe LED test mode:

- set `DEBUG_LEDS` to `1` in `firmware/STM32H7_PEDRO_OMNIBASE/CM7/Core/Inc/main.h`
- rebuild and flash
- do not connect motor power for the first test
- in this mode, all real H-bridge direction pins are held LOW/LOW and PWM
  compare registers are forced to 0

Debug LED outputs in `DEBUG_LEDS=1`:

| Test signal | MCU pin | Connect | Expected result |
|---|---|---|---|
| LD1 / M0-or-M2 activity mirror | PB0 | PB0 -> resistor -> LED -> GND | ON when M0 or M2 is commanded above threshold |
| LD2 / M1-or-M3 activity mirror | PE1 | PE1 -> resistor -> LED -> GND | ON when M1 or M3 is commanded above threshold |
| LD3 | PB14 | Do not use for LED test | PB14 is M1 PWM in this project |

H-bridge direction pins to probe:

| Motor | Pin A | Pin B | `M_dirs=0` | `M_dirs=1` | brake/off |
|---|---|---|---|---|---|
| M0 | PD4 | PD5 | PD4 LOW, PD5 HIGH | PD4 HIGH, PD5 LOW | both LOW |
| M1 | PD6 | PD7 | PD6 LOW, PD7 HIGH | PD6 HIGH, PD7 LOW | both LOW |
| M2 | PE2 | PE4 | PE2 LOW, PE4 HIGH | PE2 HIGH, PE4 LOW | both LOW |
| M3 | PE3 | PE6 | PE3 LOW, PE6 HIGH | PE3 HIGH, PE6 LOW | both LOW |

PWM pins to probe with an oscilloscope or logic analyzer:

| Motor | PWM pin | Timer | Expected in production mode |
|---|---|---|---|
| M0 | PA0 | TIM5_CH1 | 50 Hz PWM, duty 0-19999 timer counts |
| M1 | PB14 | TIM12_CH1 | 50 Hz PWM, duty 0-19999 timer counts |
| M2 | PF9 | TIM14_CH1 | 50 Hz PWM, duty 0-19999 timer counts |
| M3 | PE5 | TIM15_CH1 | 50 Hz PWM, duty 0-19999 timer counts |

Encoder pins to verify by hand-spinning each wheel:

| Motor | Timer | Channel A | Channel B | Expected check |
|---|---|---|---|---|
| M0 | TIM1 | PE9 | PE11 | `TIM1` changes in `stm32/raw` and `stm32/encoders` |
| M1 | TIM2 | PA15 | PB3 | `TIM2` changes in `stm32/raw` and `stm32/encoders` |
| M2 | TIM4 | PD12 | PD13 | `TIM4` changes in `stm32/raw` and `stm32/encoders` |
| M3 | TIM8 | PC6 | PC7 | `TIM8` changes in `stm32/raw` and `stm32/encoders` |

BNO085 pins to verify:

| Signal | MCU pin | Expected check |
|---|---|---|
| I2C1 SCL | PB6 | I2C clock during `StartIMUTask` init/service |
| I2C1 SDA | PB7 | I2C data during `StartIMUTask` init/service |
| INT | PD14 | Sensor pulls low when data is ready |
| RST | PD15 | Idle HIGH, pulsed LOW during reset sequence |
| WAKE/PS0 | PA4 | Idle HIGH |

Recommended test order:

1. Flash `DEBUG_LEDS=1`; send `/cmd_vel`; confirm PB0/PE1 LEDs respond and
   real H-bridge pins stay safe.
2. With `DEBUG_LEDS=0` and robot lifted, probe PWM and direction pins while
   sending small `/cmd_vel` commands.
3. Hand-spin each wheel and confirm the matching encoder topic changes.
4. Power motors with wheels lifted and verify +x, +y, and +yaw sign behavior.
5. Only then test on the ground at low speed.

### FreeRTOS Tasks

| Task | Priority | Stack | Function |
|---|---|---|---|
| `start_UART_RX_Task` | High | 3072 bytes | Receives USART3 bytes, assembles newline-terminated frames, parses the 30-float host command, and pushes command/PID data to queues. |
| `StartControlTask` | AboveNormal | 5120 bytes | 100 Hz control loop: reads encoders and IMU globals, handles IDLE/RUNNING/STOP/ESTOP, applies mecanum IK/FK, runs wheel PID, drives PWM/DIR pins, and publishes telemetry data to the TX queue. |
| `Start_UART_TX_Task` | Normal | 2048 bytes | Emits comma-separated `key=value` telemetry over USART3 at roughly 100 Hz. |
| `StartIMUTask` | AboveNormal | 4096 bytes | Services BNO085 SH2 reports over I2C1 and updates shared IMU globals. |
| `StartDefaultTask` | Normal | 512 bytes | Idle stub; legacy MCP2515/ODrive test loop was removed from the active path. |

### Queues

| Queue | Element | Producer | Consumer | Purpose |
|---|---|---|---|---|
| `UART2CtrlTsk_QueueHandle` | `InputData` | `start_UART_RX_Task` | `StartControlTask` | Current command frame: pose/twist fields, geometry, and wheel setpoints. |
| `UART2KPIDs_QueueHandle` | `PIDConfig` | `start_UART_RX_Task` | `StartControlTask` | PID gains from the 30-float command frame. |
| `CtrlTsk_QueueHandle` | `CtrlTsk_Data` | `StartControlTask` | `Start_UART_TX_Task` | Encoder, IMU, odometry, PWM, error, and robot-state telemetry. |
| `UART_QueueHandle` | `InputData` | `start_UART_RX_Task` | `Start_UART_TX_Task` | Last command echo for telemetry. |
| `kpids_UART_TX_QueueHandle` | `PIDConfig` | `start_UART_RX_Task` | `Start_UART_TX_Task` | PID gain echo for telemetry. |

`MutexUART_DataHandle` exists from CubeMX-generated code but is not used.

## ROS 2 Workspace Quick Reference

Workspace:

- `omnibase_ws`

Relevant package:

- `omnibase_ws/src/serial_comm`

Relevant nodes / console scripts:

| Node | Command | Purpose |
|---|---|---|
| `serial_communication` | `ros2 run serial_comm serial_communication` | Opens `/dev/ttyACM0` by default, sends the STM32 30-float command frame, reads telemetry, publishes ROS topics, and subscribes to `/cmd_vel`. |
| `pedro_dashboard` | `ros2 run serial_comm pedro_dashboard` | Serves the PEDRO web dashboard at `http://localhost:5000` and subscribes to topics from `serial_communication`. |
| `simple_rx` | `ros2 run serial_comm simple_rx` | Minimal raw serial receiver; kept for simple debugging. |

Main `serial_communication` behavior:

- serial defaults: `/dev/ttyACM0`, 115200 baud
- subscribes to `/cmd_vel` (`geometry_msgs/Twist`) when `use_cmd_vel=true`
- sends STM32 command frames at `tx_rate_hz=10.0`
- publishes legacy debug/plotting topics under `stm32/*`
- publishes standard `sensor_msgs/Imu` on `imu/data`
- publishes standard `nav_msgs/Odometry` on `odom`
- publishes robot state on `stm32/robot_state` and `stm32/robot_state_name`

Useful commands:

```bash
cd /home/roger/Github/pedro_ws/omnibase_ws
colcon build --packages-select serial_comm --symlink-install
source install/setup.bash
ros2 run serial_comm serial_communication
```

Send a small forward command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

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
