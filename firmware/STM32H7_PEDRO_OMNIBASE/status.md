# STM32H7_PEDRO_OMNIBASE — Project Merge Analysis

Analysis-only document. No code modified.

Source projects:
- `/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO` (PEDRO) — L298N + local PID + MCP2515 CAN testing
- `/home/roger/Github/home-custom-base/firmware/STM32H7_OMNIBASE_CAN_BNO085` (OMNIBASE) — ODrive over FDCAN + BNO085 over I2C1 + ESP32 BT on USART2 + ROS protocol on USART3
- `/home/roger/Github/pedro_ws/omnibase_ws/src/serial_comm` — ROS 2 serial bridge for PEDRO
- `/home/roger/Github/home-custom-base/omnibase_ws/src/odrive_comm/odrive_comm/odrive_dashboard.py` — ROS 2 dashboard / serial bridge for OMNIBASE

Target: `/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE` — currently a baseline copy of PEDRO (no ODrive, no sh2, no FDCAN). MCU is STM32H755ZITx (dual-core; firmware here runs on CM7).

---

## 1. PEDRO — Full Pinout

Extracted from `CM7/Core/Inc/main.h`, `CM7/Core/Src/main.c`, and `CM7/Core/Src/stm32h7xx_hal_msp.c`.

### 1.1 H-Bridge direction (GPIO outputs)

Defined in `main.h` lines 58–66; clock/init in `main.c:1229-1282`. Mode: push-pull, no pull, high speed.

| Bridge | Signal | Pin |
|---|---|---|
| H-Bridge 1 IN1 | PD4 | M1 direction A |
| H-Bridge 1 IN2 | PD5 | M1 direction B |
| H-Bridge 1 IN3 | PD6 | M2 direction A |
| H-Bridge 1 IN4 | PD7 | M2 direction B |
| H-Bridge 2 IN1 | PE2 | M3 direction A |
| H-Bridge 2 IN2 | PE4 | M3 direction B |
| H-Bridge 2 IN3 | PE3 | M4 direction A |
| H-Bridge 2 IN4 | PE6 | M4 direction B |

Note: the PE pin pairs are wired in (2,4) and (3,6) order — i.e. M3 uses pins 2 & 4 and M4 uses pins 3 & 6 (see `main.c:1988-1991`).

### 1.2 PWM (motor enable, L298N ENA/ENB)

PWM pins are configured by `HAL_TIM_MspPostInit()` in `stm32h7xx_hal_msp.c`:

| Motor | Timer/Channel | Pin | AF | Comment line in PEDRO source |
|---|---|---|---|---|
| M1 (ENA bridge 1) | TIM5_CH1 | PA0 | AF2_TIM5 | "H-Bridge_1 ENA" |
| M2 (ENB bridge 1) | TIM12_CH1 | PB14 | AF2_TIM12 | "H-Bridge_1 ENB" — but the comment in main.c:84 says PB14 is "H-Bridge_2 ENA" |
| M3 (ENA bridge 2) | TIM14_CH1 | PF9 | AF9_TIM14 | (comment in main.c says PB14) |
| M4 (ENB bridge 2) | TIM15_CH1 | PE5 | AF4_TIM15 | "H-Bridge_2 ENB" but comment in main.c says PF9 |

Comments inside `main.c:71-87` and the post-init code disagree. The ground truth (what gpio init writes) is:

- TIM5_CH1 → PA0
- TIM12_CH1 → PB14
- TIM13_CH1 → PF8 (initialised but TIM13 PWM is **not** started — TIM13_Init runs, but `HAL_TIM_PWM_Start` is never called and the channel is not used)
- TIM14_CH1 → PF9
- TIM15_CH1 → PE5

Mapping used by ControlTask (`main.c:1993-1996`): duty[0]→TIM5, duty[1]→TIM12, duty[2]→TIM14, duty[3]→TIM15.

So the canonical wire-up is:

```
Motor index → PWM pin
  M0 (u[0])  → PA0   (TIM5_CH1)
  M1 (u[1])  → PB14  (TIM12_CH1)
  M2 (u[2])  → PF9   (TIM14_CH1)
  M3 (u[3])  → PE5   (TIM15_CH1)
```

### 1.3 Encoder pins

From `stm32h7xx_hal_msp.c` (`HAL_TIM_Encoder_MspInit`):

| Encoder | Timer/Channels | Pin A | Pin B | AF |
|---|---|---|---|---|
| ENC0 (M0) | TIM1 CH1/CH2 | PE9 | PE11 | AF1_TIM1 |
| ENC1 (M1) | TIM2 CH1/CH2 | PA15 | PB3 | AF1_TIM2 |
| ENC2 (M2) | TIM4 CH1/CH2 | PD12 | PD13 | AF2_TIM4 |
| ENC3 (M3) | TIM8 CH1/CH2 | PC6 | PC7 | AF3_TIM8 |

Mapping used by ControlTask (`main.c:1823`): `cnt_vals[0]=TIM1->CNT`, `cnt_vals[1]=TIM2->CNT`, `cnt_vals[2]=TIM4->CNT`, `cnt_vals[3]=TIM8->CNT`.

### 1.4 Other peripherals

| Function | Pin(s) | Peripheral | Init |
|---|---|---|---|
| USART3 TX (ROS link) | PD8 | USART3, AF7 | `MX_USART3_UART_Init` (main.c:1181) |
| USART3 RX (ROS link) | PD9 | USART3, AF7 | `MX_USART3_UART_Init` |
| I2C1 SCL (BNO055) | PB6 | I2C1, AF4_OD | `MX_I2C1_Init` — note **call to `MX_I2C1_Init()` in main is commented out** (main.c:411) |
| I2C1 SDA (BNO055) | PB7 | I2C1, AF4_OD | as above |
| SPI1 SCK (MCP2515) | PA5 | SPI1, AF5 | `MX_SPI1_Init` (main.c:667) |
| SPI1 MISO | PA6 | SPI1, AF5 | |
| SPI1 MOSI | PB5 | SPI1, AF5 | |
| MCP2515 CS | PD14 | GPIO out | `main.h:160-161`, `main.c:1258` |
| ST-Link VCP (board USB-CDC) | PD8/PD9 | USART3 | Same lines as ROS link (the Nucleo wires USART3 to ST-Link VCOM) |

PEDRO uses PE9, PE11, PA15, PB3, PD12, PD13, PC6, PC7 for encoders; PA5/PA6/PB5 for SPI1 → MCP2515 (CAN). It does **not** use FDCAN1 (PD0/PD1) and does **not** use USART2.

### 1.5 BNO055 IMU code present

- Header: `CM7/Core/Inc/bno055.h`, `CM7/Core/Inc/bno055_stm32.h`
- Source: `CM7/Core/Src/bno055.c` (the STM32-specific wrapper `bno055_stm32.c` is not in Src — only the generic `bno055.c` is)
- Inside `ControlTask` (`main.c:1709-1715`, 1825-1827): the IMU code is gated by a **local `usingIMU` flag that is initialised to `0`**, so it never runs. `MX_I2C1_Init()` is also commented out in `main()`. Effectively, IMU support exists in source form but is disabled.

---

## 2. PEDRO — L298N H-Bridge Signal Mapping

From `setMotorDirection()` (main.c:213-221) and the four direction calls in `ControlTask` (main.c:1988-1991):

```
setMotorDirection(GPIOD, 4, 5, M_dirs[0]);   // M0 → PD4/PD5
setMotorDirection(GPIOD, 6, 7, M_dirs[1]);   // M1 → PD6/PD7
setMotorDirection(GPIOE, 2, 4, M_dirs[2]);   // M2 → PE2/PE4
setMotorDirection(GPIOE, 3, 6, M_dirs[3]);   // M3 → PE3/PE6
```

Direction convention in `setMotorDirection`: `dir=0` sets pin1 LOW + pin2 HIGH; `dir=1` sets pin1 HIGH + pin2 LOW; `dir≥2` brakes (both LOW). Per `CtrlOutData.M_dirs[i]` semantics in `main.h:138` and ControlTask (main.c:1974-1985): `M_dirs[i]=0` → forward, `M_dirs[i]=1` → reverse, sign of `PWM_vals[i]` selects direction.

Resulting **per-motor wiring**:

| Motor (index) | PWM pin | DIR pin A | DIR pin B |
|---|---|---|---|
| M0 (`u[0]`) | PA0  (TIM5_CH1)  | PD4 | PD5 |
| M1 (`u[1]`) | PB14 (TIM12_CH1) | PD6 | PD7 |
| M2 (`u[2]`) | PF9  (TIM14_CH1) | PE2 | PE4 |
| M3 (`u[3]`) | PE5  (TIM15_CH1) | PE3 | PE6 |

---

## 3. PEDRO — Timer / PWM Configuration

Source: `MX_TIMn_Init()` functions in `main.c`.

| Timer | Mode | Prescaler | Period | Effective | Used as |
|---|---|---|---|---|---|
| TIM1 | Encoder TI12 | 0 | 65535 | 16-bit wrap | Encoder 0 (M0) |
| TIM2 | Encoder TI12 | 0 | 65535 | 32-bit cap, read 16-bit | Encoder 1 (M1) |
| TIM4 | Encoder TI12 | 0 | 65535 | 16-bit wrap | Encoder 2 (M2) |
| TIM8 | Encoder TI12 | 0 | 65535 | 16-bit wrap | Encoder 3 (M3) |
| TIM5 | PWM1 | 239 | 19999 | ≈50 Hz (period 20 ms, 20 000 cnt) | M0 PWM (PA0) |
| TIM12 | PWM1 | 239 | 19999 | ≈50 Hz | M1 PWM (PB14) |
| TIM13 | PWM1 | 239 | 65535 | ≈3 Hz | Initialised but unused; PWM_Start never called |
| TIM14 | PWM1 | 239 | 19999 | ≈50 Hz | M2 PWM (PF9) |
| TIM15 | PWM1 | 239 | 19999 | ≈50 Hz | M3 PWM (PE5) |

SYSCLK = 480 MHz; APB1 / APB2 timer clocks land at 240 MHz, so prescaler 239 produces a 1 MHz timer tick; with `Period=19999`, PWM frequency = 1 MHz / 20000 = **50 Hz**, `max_duty_cycle = 19999`. ControlTask uses `deadzone_duty_lim = 1000` (main.c:1757) so the bottom 5 % of the duty cycle is clamped.

Encoder counts are read as raw `TIMx->CNT` (16-bit `uint16_t`); wrap-around handled by `computeDeltaCNT` (main.c:192-195) using `(int16_t)(curr - prev)`.

Encoder resolution constant: `R_counts = 424.0` (main.c:1723) for the small motors (`17380.0` for big motors is commented out). With TI12 mode this is counts/rev including ×4 quadrature.

---

## 4. PEDRO — UART Configuration & Protocol

- **USART3** (`MX_USART3_UART_Init`, main.c:1181):
  - Baud **115200**, 8-N-1, no flow control, FIFO disabled, oversampling 16.
  - Pins PD8 (TX) / PD9 (RX), AF7.
  - Routed to the Nucleo-H755 ST-Link VCOM (so it appears as `/dev/ttyACM0` on the host).

### 4.1 Command format (PC → STM32)

Parsed inside `start_UART_RX_Task` (main.c:1473-1550) by a single `sscanf` expecting **30 space-separated floats** terminated by `\n` or `\r`:

```
x_desired  y_desired  phi_end  d  r
u1_desired  u2_desired  u3_desired  u4_desired
x_Kp  x_Ki  x_Kd
y_Kp  y_Ki  y_Kd
phi_Kp phi_Ki phi_Kd
u0_Kp u0_Ki u0_Kd
u1_Kp u1_Ki u1_Kd
u2_Kp u2_Ki u2_Kd
u3_Kp u3_Ki u3_Kd
```

`x_desired`–`r` are `double`; `u1_desired`–`u4_desired` and all 21 PID gains are `float`. After parsing, the data is fanned out to four queues (`UART2KPIDs_QueueHandle`, `kpids_UART_TX_QueueHandle`, `UART_QueueHandle`, `UART2CtrlTsk_QueueHandle`). Failed parses are dropped with a debug print.

### 4.2 Telemetry format (STM32 → PC)

Built in `Start_UART_TX_Task` (main.c:1572-1669) — one `printf` per cycle at ~100 Hz (10 ms loop). Output is a single line of **comma-separated `key=value` pairs**, ~60 fields. Keys (in order):

```
x_desired, y_desired, phi_desired, d, r,
roll, pitch, yaw,
TIM1, TIM2, TIM4, TIM8,
Enc_Wheel_Omega1..4,
Inertial_ang_vel_calc, Inertial_x_vel_calc, Inertial_y_vel_calc,
ODOM_phi, ODOM_x_pos, ODOM_y_pos,
ODOM_Err_x, ODOM_Err_y, ODOM_Err_phi,
U_Err_1..4,
Ctrl_Inertial_x_dot, Ctrl_Inertial_y_dot, Ctrl_Inertial_phi_dot,
Ctrl_necc_u1..4,
ts_current, ts_previous, ts_delta,
Ctrl_duty_u1..4,
xKp, xKi, xKd,  yKp, yKi, yKd,  phiKp, phiKi, phiKd,
u0Kp, u0Ki, u0Kd, u1Kp, u1Ki, u1Kd, u2Kp, u2Ki, u2Kd, u3Kp, u3Ki, u3Kd
```

`printf` is retargeted to USART3 (via `myprintf.c` / `syscalls.c`).

---

## 5. PEDRO — FreeRTOS Task Structure

Tasks are declared in `main.c:92-150` and created in `main.c:498-510`.

| Task handle | Function | Priority | Stack (bytes) | Role |
|---|---|---|---|---|
| `defaultTaskHandle` | `StartDefaultTask` | Normal | 512 | **Not created** — `osThreadNew` line is commented out. Body is the legacy MCP2515 / ODrive test loop (main.c:1297-1409). |
| `UART_RX_TaskHandle` | `start_UART_RX_Task` | High | 3072 (256×12) | USART3 byte ring buffer + line assembler + 30-field `sscanf`; fans out to queues. |
| `UART_TX_TaskHandle` | `Start_UART_TX_Task` | Normal | 2048 (256×8) | Reads from queues, emits the comma-separated telemetry line. |
| `ControlTaskHandle` | `StartControlTask` | AboveNormal | 5120 (256×20) | Main 10 ms control loop: encoders → IK → wheel PID → PWM+DIR → odometry → publish to TX queue. |

Mutexes & queues:
- `MutexUART_DataHandle` — declared, never locked anywhere in source.
- `UART_QueueHandle` — created twice (lines 487 and 491); the second `osMessageQueueNew` overwrites the first. Final element type `InputData`, depth 3.
- `UART2CtrlTsk_QueueHandle` — `InputData`, depth 3.
- `CtrlTsk_QueueHandle` — `CtrlTsk_Data`, depth 3.
- `UART2KPIDs_QueueHandle` / `kpids_UART_TX_QueueHandle` — `PIDConfig`, depth 3 each.

Tick rate: 1 kHz (`FreeRTOSConfig.h` standard); `osDelay(10)` in ControlTask = 100 Hz, `osDelay(9)` after the TX printf = ~100 Hz telemetry.

---

## 6. PEDRO — IMU Code

Files present:
- `CM7/Core/Inc/bno055.h`, `CM7/Core/Inc/bno055_stm32.h`
- `CM7/Core/Src/bno055.c`

Usage in `ControlTask`:
- Local `uint8_t usingIMU = 0;` (main.c:1709) — IMU branch is dead code as shipped.
- When enabled, `bno055_assignI2C(&hi2c1)`, `bno055_setup()`, `bno055_setOperationModeNDOF()`, then `bno055_getVectorEuler()` is polled each cycle. Result goes into `imu->yaw/roll/pitch`.
- `MX_I2C1_Init()` call is commented out in `main()` (main.c:411), so even flipping `usingIMU=1` would hang/fault without re-enabling I2C1.

---

## 7. OMNIBASE — Mecanum Wheel Mapping

Source: `StartODriveTask` (main.c:2071-2343, key lines 2077-2090).

```c
const uint8_t num_odrives   = 4;
const double  x_offset      = 0.195;   // metres, half wheelbase X
const double  y_offset      = 0.195;   // metres, half wheelbase Y
const double  radius        = 0.0762;  // metres, wheel radius
const double  wheel_sign[4] = { -1.0, 1.0, -1.0, 1.0 };

odrives[0].NODE_ID = 36;   // CAN node id, index 0
odrives[1].NODE_ID = 34;
odrives[2].NODE_ID = 33;
odrives[3].NODE_ID = 40;
```

### 7.1 Inverse kinematics (`computeNecessaryWheelSpeedsMecanum`)

From `main.c:311-317`:

```c
u[0] = ( x_dot*(c+s) - y_dot*(c-s) - phi_dot*(x_off+y_off)) / r;
u[1] = ( x_dot*(c-s) + y_dot*(c+s) + phi_dot*(x_off+y_off)) / r;
u[2] = ( x_dot*(c-s) + y_dot*(c+s) - phi_dot*(x_off+y_off)) / r;
u[3] = ( x_dot*(c+s) - y_dot*(c-s) + phi_dot*(x_off+y_off)) / r;
```

with `c = cos(phi)`, `s = sin(phi)`. Used with `phi = 0` inside `ODrive_ProcessCommand` (main.c:1677), so body-frame twist is decomposed assuming heading-aligned drive — odometry handles the heading rotation separately.

Final command to each ODrive (`main.c:1681-1683`):

```c
Set_Input_Vel(&odrives[i], tx,
              wheel_sign[i] * (float)((u[i] * gear_ratio) / (2*PI)), 0.0f);
```

→ scaled by `gear_ratio = 9` and divided by `2π` (so the ODrive sees **rev/s** at the motor shaft). `wheel_sign[i]` flips polarity for wheels physically mounted mirror-image.

### 7.2 Position assignment (inferred from wheel_sign pattern)

`computeNecessaryWheelSpeedsMecanum` and the `wheel_sign` pattern `{-1, +1, -1, +1}` correspond to a standard 4-wheel mecanum where indices 0 and 2 are on one diagonal, 1 and 3 on the other. The asymmetric `wheel_sign` means motors 0 and 2 spin opposite their kinematic direction (the ODrive shafts on those axes are mounted facing the other way). The exact wheel-position label (FL/FR/RL/RR) is not written in the firmware; what is invariant is:

- For pure +x (forward): all four `u[i]` get sign `(+,+,+,+)`; with `wheel_sign` they become `(-, +, -, +)` at the ODrive, i.e. left-side wheels reversed — consistent with motors mounted facing each other across the body.
- For pure +y (sideways): `u` sign pattern `(-, +, +, -)` (after FK confirmation in `globalSpeedsFromUMecanum`, lines 339-341).
- For pure +φ (yaw CCW): `u` sign pattern `(-, +, -, +)` (matches FK `phi_dot` row).

`globalSpeedsFromUMecanum` (main.c:319-347) inverts those exact patterns, so IK and FK round-trip.

### 7.3 Forward kinematics / odometry (`globalSpeedsFromUMecanum`)

```c
L       = x_off + y_off;
vx_body = (r/4) * ( u0 + u1 + u2 + u3);
vy_body = (r/4) * (-u0 + u1 + u2 - u3);
phi_dot = (r/(4*L)) * (-u0 + u1 - u2 + u3);

q_dot[0] = phi_dot;
q_dot[1] = cos(phi)*vx_body - sin(phi)*vy_body;
q_dot[2] = sin(phi)*vx_body + cos(phi)*vy_body;
```

Comment in main.c:319-337 notes that the earlier version had rows swapped (vy and phi_dot patterns mixed up) and this is the fixed version. Wheel signs are re-applied in `ODrive_UpdateTelemetryAndOdometry` (main.c:1913-1914):

```c
u[i] = wheel_sign[i] * (odrives[i].AXIS_Encoder_Vel / gear_ratio) * 2.0 * PI;  // rad/s
```

---

## 8. OMNIBASE — State Machine

`enum ODriveSMState` in `main.h:198-203`:

```
SM_BOOT    = 0
SM_STARTUP = 1
SM_RUNNING = 2
SM_IDLE    = 3
```

Implemented inside `StartODriveTask` (main.c:2178-2326).

| From | Event | To | Action |
|---|---|---|---|
| BOOT | `ODRIVE_CFG_STARTUP` received | RUNNING | Run `ODrive_ProcessCommand(STARTUP)` then jump straight to RUNNING (the `SM_STARTUP` step in between is a no-op transition). |
| BOOT | `boot_delay_ms` (3000 ms) elapsed | STARTUP→RUNNING (auto) | Call `ODrive_Startup(VELOCITY_CONTROL, PASSTHROUGH, CLOSED_LOOP_CONTROL)` on all 4 axes. On failure: stay in BOOT, reset `boot_tick`. |
| STARTUP | unconditional | RUNNING | (vestigial, executed in the same tick) |
| RUNNING | `cmd.buttons & BT_ESTOP_BUTTON` (bit 2) | IDLE | Send `ODRIVE_CMD_STOP_ODRIVES`. |
| RUNNING | `cmd.type == ODRIVE_CFG_STOP` | IDLE | Process stop. |
| RUNNING | `cmd.type == ODRIVE_CFG_REBOOT` | BOOT | Reboot ODrives, restart `boot_tick`. |
| RUNNING | otherwise (cmd present) | RUNNING | `ODrive_ProcessCommand(cmd)` — subject to BT-override gating. |
| IDLE | `ODRIVE_CFG_STARTUP` | RUNNING | Process startup. |

Two watchdogs run in `SM_RUNNING`:
- **BT override watchdog** (`BT_OVERRIDE_TIMEOUT_MS = 500`): if `bt_override_active` and no BT packet for 500 ms, send a zero-vel SET_VEL and clear override so ROS can resume.
- **Command watchdog** (`CMD_WATCHDOG_TIMEOUT_MS = 500`): if no `ODRIVE_CMD_SET_VEL` from any source for 500 ms, send a single zero-vel SET_VEL to all axes and latch (`cmd_watchdog_fired = 1`) to avoid spamming CAN. Latch clears on next SET_VEL.

Source priority (main.c:2268-2285): a BT message marks `bt_override_active=1` for 500 ms; while active, ROS SET_VEL commands are silently dropped. Bit 2 of the `buttons` field (`BT_ESTOP_BUTTON 0x04` in main.h:69) is an emergency stop that bypasses priority.

---

## 9. OMNIBASE — BNO085 I2C Task Structure

### 9.1 Files

| File | Purpose |
|---|---|
| `CM7/Core/Inc/sh2_hal_impl.h` | Public `BNO085_GetHal()` |
| `CM7/Core/Src/sh2_hal_impl.c` | I2C/INT/RST glue — STM32-specific HAL adapter |
| `CM7/Core/Inc/sh2/sh2.h` | Hillcrest SH2 high-level API |
| `CM7/Core/Inc/sh2/sh2_err.h` | SH2 error codes |
| `CM7/Core/Inc/sh2/shtp.h` | SHTP transport (used by sh2.c) |
| `CM7/Core/Inc/sh2/sh2_SensorValue.h` | Sensor record union |
| `CM7/Core/Inc/sh2/sh2_util.h` | Helpers |
| `CM7/Core/Inc/sh2/sh2_hal.h` | Generic HAL interface to be implemented |
| `CM7/Core/Inc/sh2/euler.h` | Quaternion → YPR helper |
| `CM7/Core/Src/sh2/sh2.c` | SH2 protocol implementation |
| `CM7/Core/Src/sh2/shtp.c` | SHTP packet layer |
| `CM7/Core/Src/sh2/sh2_SensorValue.c` | Sensor decoders |
| `CM7/Core/Src/sh2/sh2_util.c` | |
| `CM7/Core/Src/sh2/euler.c` | `q_to_ypr()` |

### 9.2 Hardware wiring (sh2_hal_impl.c lines 7-12, 33-41)

```
I2C1 SCL = PB6   (AF4_I2C1, open-drain)
I2C1 SDA = PB7   (AF4_I2C1, open-drain)
BNO085 7-bit addr = 0x4A  (PS0=PS1=GND, Adafruit default)
RST     = PD15   (active-LOW output, idle HIGH)
INT     = PD14   (active-LOW input,  pull-up)
WAKE/PS0 = PA4   (push-pull, idle HIGH, configured but optional)
CS      = PB4    (configured but unused on I2C — leftover from SPI path)
```

I2C1 init: `MX_I2C1_Init` in main.c:682, `Timing = 0x00B03FDB` (≈400 kHz at D2PCLK1).

### 9.3 FreeRTOS task

`StartIMUTask` (main.c:1581-1630), priority AboveNormal, stack 4096:

1. `osDelay(500)` to let ODriveTask finish boot prints (no UART TX mutex).
2. `sh2_open(BNO085_GetHal(), imu_async_event_cb, NULL)` — runs the RST sequence inside `sh2_hal_impl.c`.
3. `imu_service_ms(200)` (calls `sh2_service()` for 200 ms while yielding 1 ms per loop).
4. `sh2_setSensorCallback(imu_sensor_data_cb, NULL)`.
5. `imu_enable_all_reports()` — enables `SH2_ROTATION_VECTOR`, `SH2_GYROSCOPE_CALIBRATED`, `SH2_LINEAR_ACCELERATION` at 50 Hz (`BNO085_REPORT_INTERVAL_US = 20000`).
6. Main loop: `sh2_service()` + 2 ms delay. On `SH2_RESET` async event (latched in `g_bno085_sensor_ready`), re-enable reports.

Callbacks (`imu_sensor_data_cb` main.c:1497-1550) write atomic 32-bit floats into shared globals consumed by `StartODriveTask`:

```
g_bno085_qx, g_bno085_qy, g_bno085_qz, g_bno085_qw   (rotation vector, ROS x,y,z,w order)
g_bno085_wx, g_bno085_wy, g_bno085_wz                 (gyro rad/s, body)
g_bno085_ax, g_bno085_ay, g_bno085_az                 (linear accel m/s^2, gravity removed)
g_bno085_yaw, g_bno085_pitch, g_bno085_roll           (radians, derived in callback via q_to_ypr)
```

No mutex — each 32-bit float read/write is atomic on Cortex-M7 and ODriveTask reads at 100 Hz.

---

## 10. OMNIBASE — UART Telemetry & Command Parsing

### 10.1 UART configuration

| UART | Pins | Baud | Role |
|---|---|---|---|
| USART2 | PD5 TX / PD6 RX (AF7) | 115200 | ESP32 Bluetooth bridge |
| USART3 | PD8 TX / PD9 RX (AF7) | **230400** | ROS link to PC |

Both use 8-N-1, no flow control, FIFO disabled.

### 10.2 Command format on USART3 (`UART_RX_ParseLine`, main.c:1952-2067)

First token is `msg_type`:

- **Type 1** — control (set vel): `"1 <vx> <vy> <wz>"` (doubles). Builds `ODriveCmdMsg{type=ODRIVE_CMD_SET_VEL, target_mask=0x0F, source=CMD_SOURCE_ROS, robot_twist=[vx,vy,wz]}`. Pushed to `UART_QueueHandle` (echo to UART_TX_Task) and `URX_2_CAN_QueueHandle` (consumer = ODriveTask).
- **Type 2** — config: `"2 <sub_type> <mask_hex> [params...]"`. `sub_type` matches `ODriveCmdType` enum (`main.h:174-196`):

| sub_type (hex) | Format | Notes |
|---|---|---|
| `0x20` clear_errors | `2 20 <mask>` | |
| `0x21` set_state | `2 21 <mask> <state>` | `state` is the ODrive `AXIS_STATE_*` value |
| `0x22` set_ctrl_mode | `2 22 <mask> <ctrl_mode> <input_mode>` | |
| `0x23` set_limits | `2 23 <mask> <vel_lim> <curr_lim>` | floats |
| `0x24` set_pos_gain | `2 24 <mask> <pg>` | |
| `0x25` set_vel_gains | `2 25 <mask> <vg> <vi>` | |
| `0x26` startup | `2 26 <mask> <cm> <im> <state>` | defaults 2,1,8 if omitted |
| `0x27` reboot | `2 27 <mask>` | |
| `0x28` set_torque | `2 28 <mask> <tq>` | applied to all four `torque_ff[]` |
| `0x29` stop | `2 29 <mask>` | |
| `0x30` set_input_pos | `2 30 <mask> <pos> <vel_ff> <torque_ff>` | |

Each parsed config produces an `ODriveCmdMsg` pushed only to `URX_2_CAN_QueueHandle`.

### 10.3 USART2 Bluetooth format (`start_BT_RX_Task`, main.c:1253-1392)

Type-3 message from ESP32: `"3 <vx> <vy> <wz> <buttons_hex> <controll_state>\r\n"` where `controll_state ∈ {0=Inactive, 1=Paired/TX-disabled, 2=Active}`. Deadzone: 0.05 m/s linear, 0.05 rad/s angular. Only Active (`controll_state == 2`) packets queue a SET_VEL.

### 10.4 Telemetry on USART3 (`Start_UART_TX_Task`, main.c:1396-1474)

Single `printf` per cycle, ~100 Hz, format = comma-separated `key=value`. Fields (in order):

```
CMD_vx, CMD_vy, CMD_wz
IMU_yaw, IMU_roll, IMU_pitch
IMU_qx, IMU_qy, IMU_qz, IMU_qw
IMU_wx, IMU_wy, IMU_wz
IMU_ax, IMU_ay, IMU_az
IK_u0..3
ODOM_phi, ODOM_x, ODOM_y, ODOM_w, ODOM_vx, ODOM_vy
For each axis 0..3:
  N<i>, E<i>, S<i>, C<i>, P<i>, V<i>, Sh<i>, CPR<i>, Vbus<i>, Ibus<i>, IqSet<i>, IqMeas<i>, U<i>
BT_active, BT_vx, BT_vy, BT_wz
ESP32_age_ms
```

`ESP32_age_ms = -1` means "no ESP32 packet ever received". `BT_active`: 0 inactive, 1 paired, 2 active.

---

## 11. OMNIBASE — Odometry Calculation

Performed by `ODrive_UpdateTelemetryAndOdometry` (main.c:1889-1932) every `telemetry_period = 10 ms`:

1. For each axis, read `AXIS_Encoder_Vel` (rev/s at motor shaft), convert back to wheel angular velocity in rad/s:
   ```c
   u[i] = wheel_sign[i] * (vel_est_i / gear_ratio) * 2*PI;
   ```
2. `globalSpeedsFromUMecanum(theta, x_offset, y_offset, radius, u, q_dot)` (see §7.3).
3. Integrate:
   ```c
   dt_s   = delta_t * 0.001;
   x     += q_dot[1] * dt_s;
   y     += q_dot[2] * dt_s;
   theta += q_dot[0] * dt_s;
   ```
4. Copy into `telemetryMsg.odom.{phi, x_pos, y_pos, q_dot[3]}`.

No θ wrapping is performed; the value grows unbounded. `delta_t` comes from `now - last_telem_tick`, not from a fixed dt.

---

## 12. OMNIBASE — Watchdog / Timeout / Emergency Stop

| Mechanism | Where | Timeout | Action |
|---|---|---|---|
| `FDCAN_WAIT_TX_FREE` macro | main.c:87-95 | 50 ms | Spin-wait for free FDCAN TX FIFO slot, print "CAN TX timeout" on expiry, then return so a dead ODrive doesn't freeze the task. |
| Command watchdog | `StartODriveTask` main.c:2109-2238 | 500 ms (`CMD_WATCHDOG_TIMEOUT_MS`) | If no `ODRIVE_CMD_SET_VEL` arrives from any source, send one zero-velocity SET_VEL and latch. |
| BT override watchdog | main.c:2097-2099, 2240-2251 | 500 ms (`BT_OVERRIDE_TIMEOUT_MS`) | If `bt_override_active` and no BT packet in window, zero-vel and release override. |
| BT_ESTOP_BUTTON | main.c:69 (`0x04`), 2254-2266 | immediate | RUNNING → IDLE via `ODRIVE_CMD_STOP_ODRIVES`. |
| Boot delay | main.c:2113 | 3000 ms | After boot, auto-trigger `ODrive_Startup`. |
| BT_RX line buffer watchdog | main.c:1263, 1383-1387 | 1000 ms | Heartbeat: log if no USART2 bytes (currently the printf is commented out). |

There is no hardware IWDG/WWDG enabled.

---

## 13. OMNIBASE — FreeRTOS Tasks & Queues

Tasks (created in `main()` main.c:511-529; defaults at top of file):

| Task | Priority | Stack (bytes) | Role |
|---|---|---|---|
| `start_UART_RX_Task` | High | 4096 | USART3 ring buffer + line assembler + `UART_RX_ParseLine` |
| `start_BT_RX_Task` | High | 4096 | USART2 ring buffer + Type-3 parser (ESP32 BT) |
| `Start_UART_TX_Task` | Normal | 4096 | Telemetry printf at ~100 Hz |
| `StartODriveTask` | AboveNormal | 4096 | State machine + IK + FDCAN1 TX + odometry + telemetry assembly |
| `StartIMUTask` | AboveNormal | 4096 | BNO085 SH2 service loop, 50 Hz reports |
| `StartControlTask` | AboveNormal | 1024 | **Empty body** (`for(;;) osDelay(10);`) — vestigial |
| `StartDefaultTask` | Normal | 512 | Defined, **never instantiated** (no `osThreadNew` for it) |

Queues:

| Queue | Element | Depth | Producer | Consumer |
|---|---|---|---|---|
| `UART_QueueHandle` | `ODriveCmdMsg` | 3 | `UART_RX_ParseLine` (type 1) | `Start_UART_TX_Task` (`last_cmd` echo) |
| `URX_2_CAN_QueueHandle` | `ODriveCmdMsg` | 3 | `UART_RX_ParseLine`, `start_BT_RX_Task` | `StartODriveTask` |
| `CAN_2_UTX_QueueHandle` | `ODriveTelemetryMsg` | 3 | `StartODriveTask` (via `ODrive_PushLatestTelemetry`) | `Start_UART_TX_Task` |
| `CtrlTsk_QueueHandle`, `UART2CtrlTsk_QueueHandle`, `UART2KPIDs_QueueHandle`, `kpids_UART_TX_QueueHandle` | various | 3 | declared at top of file but **never created** in `main()` and **never used** in any active task | unused |

Other shared state (no mutex):
- `BT_active` (volatile u8)
- `g_bt_last_valid_msg_tick` (volatile u32)
- `g_bno085_*` (volatile float)

Mutex `MutexUART_DataHandle` is created but never acquired anywhere.

ISR callback fan-out: `HAL_UART_RxCpltCallback` (main.c:1193-1204) demultiplexes USART2 vs USART3 to the right ring buffer.

---

## 14. OMNIBASE — ODrive-specific vs Reusable Files

### 14.1 ODrive / FDCAN-specific (will not be carried over)

| File | Why | Disposition |
|---|---|---|
| `CM7/Core/Inc/ODrive.h` | Whole API is ODrive CAN frames | Reject |
| `CM7/Core/Src/ODrive.c` | All `Set_Input_Vel` / `Clear_Errors` / `Set_Axis_State` helpers and `ODrive_RX_CallBack` | Reject |
| FDCAN1 init in `main.c:599-677` (`MX_FDCAN1_Init`) | FDCAN peripheral, GPIO PD0/PD1, filters, ISR | Reject (pins also reused by PEDRO so they're free, but the peripheral is unused) |
| `HAL_FDCAN_MspInit` / `HAL_FDCAN_MspDeInit` in `stm32h7xx_hal_msp.c` | FDCAN MSP | Reject |
| `HAL_FDCAN_RxFifo0Callback` in `main.c:1097-1112` | ODrive RX dispatcher | Reject |
| `Axis odrives[ODRIVE_COUNT]` global, `Find_ODrive_By_NodeID`, all `ODrive_*` helpers (`ODrive_Startup`, `ODrive_ProcessCommand`, `ODrive_UpdateTelemetryAndOdometry`, `ODrive_PushLatestTelemetry`) | ODrive-only | Reject as written; reuse `ODrive_UpdateTelemetryAndOdometry`'s odometry math (the call to `globalSpeedsFromUMecanum` + integration) as inspiration only |
| `ODriveCmdMsg`, `ODriveTelemetryMsg`, `ODriveCmdType`, `ODriveSMState`, `ODriveCmdSource` typedefs | Wrap ODrive concepts | Reject — PEDRO needs simpler `InputData`/`CtrlTsk_Data` (already present) |
| `CAN_2_UTX_QueueHandle`, `URX_2_CAN_QueueHandle` | Carry ODrive messages | Reject; replace with PEDRO's `CtrlTsk_Queue` / `UART2CtrlTsk_Queue` already in PEDRO |
| Type-2 config sub-commands `ODRIVE_CFG_*` in `UART_RX_ParseLine` | Tied to ODrive runtime config (limits, gains, control mode) | Reject — PEDRO PID gains are sent inside the Type-1 line directly |

### 14.2 Reusable (no ODrive coupling)

| File / block | Why it's portable | Disposition |
|---|---|---|
| `CM7/Core/Inc/sh2/*.h`, `CM7/Core/Src/sh2/*.c` | Hillcrest SH2 library — generic | Carry over verbatim |
| `CM7/Core/Inc/sh2_hal_impl.h`, `CM7/Core/Src/sh2_hal_impl.c` | STM32 I2C glue, only depends on `hi2c1`, GPIO PD14/PD15/PB4/PA4 | Carry over; PEDRO already has I2C1 init but the call is commented out |
| `StartIMUTask` + `imu_*` callbacks + `g_bno085_*` globals | Self-contained 50 Hz BNO085 task | Carry over; replace PEDRO's BNO055 path in `ControlTask` (the dead `usingIMU` branch) |
| `BT_active`, `g_bt_last_valid_msg_tick`, `start_BT_RX_Task` | USART2 ring-buffer + Type-3 parser; no ODrive references | Reusable but **conflicts with PEDRO H-Bridge pins** — see §16 |
| `globalSpeedsFromUMecanum` (main.c:319-347) | Pure math — corrected mecanum FK | Carry over to replace PEDRO's older `globalSpeedsFromUMecanum` (which has rows swapped) and use it for odometry |
| `computeNecessaryWheelSpeedsMecanum` (main.c:311-317) | Pure math | Carry over to replace PEDRO's older version; PEDRO already has the same function but with a different `u[2]/u[3]` arrangement — needs reconciliation |
| Type-1 / Type-2 protocol shape on USART3 | Wire format only | Reusable as the **command** wire format if we want to move from PEDRO's 30-float line to OMNIBASE's compact `1 vx vy wz`. Optional. |
| `myprintf.c`, `syscalls.c`, `stm32h7xx_hal_msp.c` (timer/encoder/usart3 blocks) | Already match PEDRO | Already present in PEDRO_OMNIBASE |
| Watchdog idea (`CMD_WATCHDOG_TIMEOUT_MS`, zero-vel latch) | Pure logic | Carry concept across (apply to PEDRO's per-wheel PID inputs) |
| `BT_ESTOP_BUTTON` (`0x04`) emergency stop check | Pure logic | Carry concept |
| `q_to_ypr` (`euler.c`) | Pure math | Carry over |

---

## 15. ROS 2 — `serial_comm` Package

### 15.1 Files

- `serial_comm/serial_communication.py` — full bridge (TX 10 Hz, RX threaded).
- `serial_comm/simple_rx.py` — minimal raw-line republisher to `/uart_rx_raw`.

### 15.2 Wire format used by `serial_communication.py`

- Serial: `/dev/ttyACM0` @ **115200** baud, timeout 0.2 s (matches PEDRO USART3).
- TX (`send_mesage`, 10 Hz): 30 space-separated floats, `\r\n`-terminated, in order:
  ```
  x  y  phi  d  r
  u1_desired  u2_desired  u3_desired  u4_desired
  x_kp  x_ki  x_kd
  y_kp  y_ki  y_kd
  phi_kp phi_ki phi_kd
  u0_kp u0_ki u0_kd  u1_kp u1_ki u1_kd  u2_kp u2_ki u2_kd  u3_kp u3_ki u3_kd
  ```
  Exactly matches the PEDRO firmware `sscanf` ordering.
- RX (`receiver`, threaded): regex `(\w+)=([-+]?\d*\.\d+|\d+)` on every line; parses key=value into a dict, then publishes per-field topics. Matches the PEDRO telemetry keys (`x_desired`, `roll`, `pitch`, `yaw`, `TIM1..TIM8`, `Enc_Wheel_OmegaN`, `Inertial_*`, `ODOM_*`, `Ctrl_*`, `ts_*`, `Ctrl_duty_*`, `xKp..u3Kd`).

### 15.3 Topics

Publishers (all `serial_comm` topics):
- `stm32_debug` (`String`) — multi-line human-readable summary
- `stm32/raw` (`String`) — raw line
- `stm32/pose` (`Float32MultiArray`) — `[x, y, phi, d, r]`
- `stm32/imu` (`Float32MultiArray`) — `[roll, pitch, yaw]`
- `stm32/omegas` (`Float32MultiArray`) — `[ω1..ω4]`
- `stm32/real_speeds` (`Float32MultiArray`) — `[φ_dot, x_dot, y_dot]`
- `stm32/odom` (`Float32MultiArray`) — `[phi, x, y]`
- `stm32/errors` (`Float32MultiArray`) — `[dx, dy, dphi]`
- `stm32/u_errors` (`Float32MultiArray`) — per-wheel errors
- `stm32/ctrl_speeds` (`Float32MultiArray`) — `[x_dot, y_dot, phi_dot]`
- `stm32/ctrl_u` (`Float32MultiArray`) — `[u1..u4]`
- `stm32/pwm` (`Int32MultiArray`) — `[duty1..duty4]`
- `stm32/timing` (`Int32MultiArray`)
- `stm32/encoders` (`Int32MultiArray`) — `[TIM1, TIM2, TIM4, TIM8]`
- `stm32/pid_gains/{x,y,phi,u0,u1,u2,u3}` (`Float32MultiArray`) — `[Kp, Ki, Kd]`

Subscribers: **none**. There is no `cmd_vel` topic; setpoints are driven only by ROS parameters (`x`, `y`, `phi`, `u1_desired`..`u4_desired`, all PID gains).

`simple_rx.py` only publishes `/uart_rx_raw` (raw `String` at 1 Hz QoS BEST_EFFORT).

---

## 16. Conflicts when merging OMNIBASE into PEDRO_OMNIBASE

| Resource | PEDRO uses | OMNIBASE uses | Outcome |
|---|---|---|---|
| PD5, PD6 | H-Bridge1 IN2/IN3 (motor M1 dir + motor M2 dir) | USART2 TX/RX for ESP32 BT | **Direct collision.** Bringing BT to PEDRO requires remapping either USART2 to alternate pins (e.g. PA2/PA3 if free) or remapping the H-Bridge pins. |
| PD0, PD1 | unused | FDCAN1 RX/TX | Free in PEDRO — no conflict if FDCAN is ever needed. We are **not** carrying FDCAN over. |
| PA5 | SPI1_SCK (MCP2515 CAN) | SPI1_SCK (but `MX_SPI1_Init` is commented out in OMNIBASE; SPI bus is unused) | PEDRO can keep MCP2515 if it needs CAN. Not needed for the merge target. |
| PB4 | unused | BNO085 CS (idle high, unused on I2C) | Free; harmless if init code is carried over. |
| PD14, PD15 | MCP2515 CS (PD14) | BNO085 RST (PD15), BNO085 INT (PD14 in OMNIBASE) | **PD14 collision** if PEDRO keeps the MCP2515 path; if we drop MCP2515 (we are dropping CAN entirely) the pin frees up for BNO085 INT. |
| PA4 | unused | BNO085 WAKE/PS0 | Free; carry over. |
| PB6, PB7 | I2C1 SCL/SDA (BNO055, init commented out) | I2C1 SCL/SDA (BNO085, init enabled) | Same pins, same peripheral. PEDRO_OMNIBASE will re-enable `MX_I2C1_Init()` and swap BNO055 driver for the SH2 stack. |
| USART3 baud | 115200 | 230400 | PEDRO's `serial_comm` Python node is hardcoded to 115200. We can keep 115200 on PEDRO_OMNIBASE since the existing host node already works at that rate (telemetry payload is smaller than OMNIBASE's). |

`stm32h7xx_hal_msp.c` and `main.c` GPIO init in the target need cross-checking against the above before any code change.

---

## MERGE PLAN

Goal: extend PEDRO_OMNIBASE with the BNO085 IMU pipeline and (optionally) BT command path, while keeping PEDRO's L298N + local PID architecture. Everything below is **proposal only** — no code modified.

1. **Replace BNO055 with BNO085 (mecanum-base IMU path).**
   - Carry over from OMNIBASE: `CM7/Core/Inc/sh2/`, `CM7/Core/Inc/sh2_hal_impl.h`, `CM7/Core/Src/sh2/`, `CM7/Core/Src/sh2_hal_impl.c`, and the `euler.c`/`euler.h` helper.
   - Remove `bno055.h`, `bno055_stm32.h`, `bno055.c` from PEDRO_OMNIBASE.
   - In PEDRO_OMNIBASE `main.c`, replace the `usingIMU` block in `StartControlTask` with read of `g_bno085_*` globals (yaw used as `phi` for odometry rotation, qx..qw / wx..wz / ax..az added to a new `IMUData`-shaped struct).
   - Update `IMUData` typedef in `main.h` to add `qx, qy, qz, qw, wx, wy, wz, ax, ay, az` (copy from OMNIBASE `main.h:101-124`).
   - Add a new `StartIMUTask` thread (copy from OMNIBASE main.c:1581-1630). Priority AboveNormal, stack 4096.
   - Add the static callbacks `imu_async_event_cb`, `imu_sensor_data_cb`, `imu_service_ms`, `imu_enable_report`, `imu_enable_all_reports` (OMNIBASE main.c:1489-1579) verbatim.
   - Re-enable `MX_I2C1_Init()` in `main()` (uncomment the line that is currently commented out).
   - Configure PB4 (CS unused), PA4 (WAKE), PD14 (INT), PD15 (RST) in `MX_GPIO_Init()` (copy from OMNIBASE main.c:1050-1083).

2. **Reconcile mecanum kinematics with PEDRO's existing IK/FK.**
   - PEDRO already has `computeNecessaryWheelSpeedsMecanum` and `globalSpeedsFromUMecanum`, but the FK rows are swapped relative to OMNIBASE's corrected version. Replace PEDRO's `globalSpeedsFromUMecanum` body with OMNIBASE's (main.c:319-347).
   - PEDRO's `computeNecessaryWheelSpeedsMecanum` (main.c:266-273) differs from OMNIBASE's at `u[2]`: PEDRO writes `u[2] = (... + phi_dot*(x_off+y_off))/r`, OMNIBASE writes `u[2] = (... - phi_dot*(x_off+y_off))/r`. Use OMNIBASE's signs so IK round-trips against the new FK.
   - In `StartControlTask`, switch the IK call from `computeNecessaryWheelSpeedsOmni` (main.c:1945) to `computeNecessaryWheelSpeedsMecanum` and the FK call from `globalSpeedsFromUOmni` (main.c:2005) to `globalSpeedsFromUMecanum`. Pass `x_off`/`y_off` instead of `d`.
   - Add `x_off`, `y_off` to the `InputData.sscanf` line (or make them compile-time constants `0.195`/`0.195` matching the physical PEDRO frame — confirm with user first).
   - Apply per-wheel sign correction if mechanical mirror-mount is the case (`wheel_sign[4]` constant similar to OMNIBASE).

3. **Optional: ESP32 BT command path.**
   - Only if the user wants joystick override on PEDRO. Requires re-routing USART2 away from PD5/PD6 (they are H-Bridge1 IN2/IN3). Alternate pins for USART2: PA2 (TX) / PA3 (RX), AF7 — free in PEDRO.
   - If pursued: copy `start_BT_RX_Task`, the secondary ring buffer (`rx_buf2`, `rx_head2`, `rx_tail2`, `rx_char2`), the `HAL_UART_RxCpltCallback` USART2 branch, and `BT_active` / `g_bt_last_valid_msg_tick` globals.
   - Define `BT_ESTOP_BUTTON 0x04` (or whatever the PEDRO joystick maps to) in `main.h`.
   - Skipping this initially is recommended — it adds a watchdog and a parallel source path before the local PID is even known to behave.

4. **Telemetry/command compatibility decision.**
   - The host node `pedro_ws/omnibase_ws/src/serial_comm/serial_communication.py` is built around PEDRO's 30-float command + 60-field telemetry. Keep that format unchanged. Do **not** import OMNIBASE's Type-1/Type-2/Type-3 wire format — the local PID architecture needs the 21 gain fields the OMNIBASE format does not carry.
   - Add new telemetry keys for the BNO085 extras (`IMU_qx..qw`, `IMU_wx..wz`, `IMU_ax..az`) to the existing `Start_UART_TX_Task` printf format string. Update `serial_communication.py` regex consumers accordingly (or accept missing keys via `data.get(..., 0)`).

5. **Watchdog (concept import).**
   - Adopt the OMNIBASE 500 ms zero-velocity watchdog inside PEDRO's `StartControlTask`: if `UART2CtrlTsk_QueueHandle` has not produced an `InputData` for > 500 ms, zero out `data.u{1..4}_desired` (and `data.x_desired`/`y_desired`/`phi_end` for pose mode) and set PWM duties to 0. Latch like OMNIBASE.

6. **Cleanup of PEDRO leftovers.**
   - Delete `mcp2515.c`/`mcp2515.h`/`mcp2515_consts.h`/`can.h` if MCP2515 is not used in the merged target (saves stack + frees PD14). Confirm with user.
   - Delete `StartDefaultTask` body (the MCP2515 + ODrive test). It is not instantiated anyway.
   - Delete the duplicate `osMessageQueueNew(UART_QueueHandle, ...)` call at main.c:487 (the line at 491 overrides it; the first one allocates with the wrong element type).

7. **FreeRTOS task table (proposed for PEDRO_OMNIBASE):**

   | Task | Priority | Stack | Source |
   |---|---|---|---|
   | `start_UART_RX_Task` | High | 3072 | PEDRO (unchanged) |
   | `Start_UART_TX_Task` | Normal | 2048 | PEDRO (printf format extended with BNO085 fields) |
   | `StartControlTask` | AboveNormal | 5120 | PEDRO (IK/FK swapped to mecanum, BNO085 reads from globals) |
   | `StartIMUTask` | AboveNormal | 4096 | OMNIBASE (carry over) |
   | (optional) `start_BT_RX_Task` | High | 4096 | OMNIBASE (carry over only if USART2 is remapped) |

8. **Verification steps once code lands:**
   - Spin all four wheels open-loop with hardcoded duties, confirm DIR/PWM mapping matches table in §2.
   - Read encoder counters with the four motors driven, confirm sign per wheel matches the `wheel_sign[]` constant from §7.
   - Bring up BNO085 standalone (just `StartIMUTask`); check `IMU_qw` ≈ 1 at rest, `IMU_wz` swings under rotation.
   - End-to-end: with `serial_comm` node running, send a unit `vx` and check `ODOM_x_pos` integrates over time.

---

## REJECT LIST

Will **not** be copied into PEDRO_OMNIBASE. Everything below is OMNIBASE-only, ODrive/FDCAN-coupled, or otherwise inappropriate for the L298N target:

1. `CM7/Core/Inc/ODrive.h`
2. `CM7/Core/Src/ODrive.c`
3. `MX_FDCAN1_Init()` and all FDCAN1 setup in `main.c`
4. `HAL_FDCAN_MspInit` / `HAL_FDCAN_MspDeInit` in `stm32h7xx_hal_msp.c`
5. `HAL_FDCAN_RxFifo0Callback` and `Find_ODrive_By_NodeID` in `main.c`
6. `Axis odrives[ODRIVE_COUNT]` global, the `NODE_ID` table, `gear_ratio` field uses
7. `ODrive_Startup`, `ODrive_ProcessCommand`, `ODrive_UpdateTelemetryAndOdometry`, `ODrive_PushLatestTelemetry` (use the OMNIBASE odometry math as reference inside PEDRO's existing `StartControlTask`, but do not import these symbols)
8. `ODriveCmdMsg`, `ODriveTelemetryMsg`, `ODriveCmdType`, `ODriveSMState`, `ODriveCmdSource` typedefs (`main.h:174-263`)
9. `CAN_2_UTX_QueueHandle`, `URX_2_CAN_QueueHandle`, `UART_QueueHandle` reshaped to `ODriveCmdMsg`
10. `MX_SPI1_Init` body (commented out in OMNIBASE anyway; we are also dropping SPI1/MCP2515)
11. `StartDefaultTask` body in OMNIBASE (the ODrive test loop)
12. `StartControlTask` body in OMNIBASE (empty / vestigial — PEDRO already has the real control loop)
13. USART3 baud 230400 — keep PEDRO's 115200 to remain compatible with `serial_communication.py`.
14. Type-1 / Type-2 wire protocol (`UART_RX_ParseLine` in OMNIBASE main.c:1952-2067) — PEDRO needs the 30-float format because per-wheel PID gains are tuned over the link.
15. `BT_OVERRIDE_TIMEOUT_MS` source-priority logic — only meaningful when both BT and ROS race; skip unless BT path is added.
16. `Axis`/CAN_STUB conditional compilation block
17. `ODriveTelemetryMsg`-shaped CAN_2_UTX queue path
18. `MutexUART_DataHandle` — created but never used in either project; no need to carry the unused mutex over.
19. MCP2515 (`mcp2515.c/h`, `mcp2515_consts.h`, `can.h`) — present in both PEDRO and PEDRO_OMNIBASE source tree but not initialised in OMNIBASE's main; drop if user confirms no MCP CAN need.
20. Telemetry fields specific to ODrive axes: `N<i>, E<i>, S<i>, C<i>, P<i>, V<i>, Sh<i>, CPR<i>, Vbus<i>, Ibus<i>, IqSet<i>, IqMeas<i>, U<i>` — PEDRO will emit `TIMx`, `Enc_Wheel_OmegaN`, `Ctrl_duty_uN` instead (already in PEDRO format).
21. `BT_active`, `BT_vx`, `BT_vy`, `BT_wz`, `ESP32_age_ms` telemetry — only if §3 of the merge plan is executed.

---

## EXECUTED MERGE — files changed / added / removed

All changes are inside `STM32H7_PEDRO_OMNIBASE/`; the original `STM32H7_PEDRO/`
tree is untouched.

### Files added (copied verbatim from `STM32H7_OMNIBASE_CAN_BNO085`)

| Path | Source |
|---|---|
| `CM7/Core/Inc/sh2_hal_impl.h` | OMNIBASE `CM7/Core/Inc/sh2_hal_impl.h` |
| `CM7/Core/Src/sh2_hal_impl.c` | OMNIBASE `CM7/Core/Src/sh2_hal_impl.c` |
| `CM7/Core/Inc/sh2/euler.h`            | OMNIBASE `…/Inc/sh2/euler.h` |
| `CM7/Core/Inc/sh2/sh2.h`              | OMNIBASE `…/Inc/sh2/sh2.h` |
| `CM7/Core/Inc/sh2/sh2_err.h`          | OMNIBASE `…/Inc/sh2/sh2_err.h` |
| `CM7/Core/Inc/sh2/sh2_hal.h`          | OMNIBASE `…/Inc/sh2/sh2_hal.h` |
| `CM7/Core/Inc/sh2/sh2_SensorValue.h`  | OMNIBASE `…/Inc/sh2/sh2_SensorValue.h` |
| `CM7/Core/Inc/sh2/sh2_util.h`         | OMNIBASE `…/Inc/sh2/sh2_util.h` |
| `CM7/Core/Inc/sh2/shtp.h`             | OMNIBASE `…/Inc/sh2/shtp.h` |
| `CM7/Core/Src/sh2/euler.c`            | OMNIBASE `…/Src/sh2/euler.c` |
| `CM7/Core/Src/sh2/sh2.c`              | OMNIBASE `…/Src/sh2/sh2.c` |
| `CM7/Core/Src/sh2/sh2_SensorValue.c`  | OMNIBASE `…/Src/sh2/sh2_SensorValue.c` |
| `CM7/Core/Src/sh2/sh2_util.c`         | OMNIBASE `…/Src/sh2/sh2_util.c` |
| `CM7/Core/Src/sh2/shtp.c`             | OMNIBASE `…/Src/sh2/shtp.c` |

### Files modified

- **`CM7/Core/Inc/main.h`**
  - `IMUData` extended with `qx,qy,qz,qw,wx,wy,wz,ax,ay,az` (rotation
    vector, body-frame gyro rad/s, linear accel m/s² gravity-removed).
  - Added `RobotState` enum (`ROBOT_STATE_IDLE/RUNNING/STOP/ESTOP`).
  - Added `BNO085_RST_Pin (PD15)`, `BNO085_INT_Pin (PD14)`,
    `BNO085_WAKE_Pin (PA4)` private defines.
  - Added Nucleo `LD1_Pin (PB0)`, `LD2_Pin (PE1)`, `LD3_Pin (PB14)` and
    `DEBUG_LEDS` compile-time switch (default 0).
  - Added `CMD_WATCHDOG_TIMEOUT_MS = 500` and `ESTOP_CMD_KEY = -9999.0`.
  - `CtrlTsk_Data` gained a `robot_state` byte for telemetry.

- **`CM7/Core/Src/main.c`** — substantial restructure:
  - Removed direct calls to `bno055_*` and `mcp2515_*`. `bno055_stm32.h`
    is still included (so `bno055.c` continues to find its glue at link
    time) but no BNO055 function is called.
  - Added `#include` for `sh2.h`, `sh2_hal_impl.h`, `sh2_SensorValue.h`,
    `sh2_err.h`, `sh2_hal.h`, `euler.h`.
  - `MX_I2C1_Init()` is now called from `main()`; `MX_SPI1_Init()` is
    not (MCP2515 is dropped — PD14 reused for BNO085 INT).
  - `MX_GPIO_Init()` now also configures `PD14` (BNO085 INT, pull-up
    input), `PD15` (BNO085 RST, output idle HIGH), `PA4` (BNO085 WAKE,
    output idle HIGH), and conditionally configures `PB0/PE1/PB14` as
    LED outputs when `DEBUG_LEDS` is set.
  - File-scope additions: `g_bno085_yaw/pitch/roll/qx/.../az` shared
    floats, `g_last_cmd_tick`, `g_wheel_sign[4] = {-1, +1, -1, +1}`
    (matches OMNIBASE main.c:2081 exactly), `IMU_TaskHandle`.
  - `StartIMUTask` ported verbatim from OMNIBASE (50 Hz rotation vector,
    gyro, linear-accel reports). Created in `main()` at priority
    AboveNormal / stack 4096.
  - `computeNecessaryWheelSpeedsMecanum` now matches OMNIBASE main.c:
    311-317 signs (`u[2]` phi_dot sign flipped from PEDRO's original).
  - `globalSpeedsFromUMecanum` replaced with OMNIBASE main.c:319-347
    corrected forward kinematics.
  - `start_UART_RX_Task` updates `g_last_cmd_tick` on every successful
    30-float parse so the watchdog clears. Default `data` initialiser
    seeds `x_off = y_off = 0.195`, `r = 0.0762`.
  - `Start_UART_TX_Task` switched to non-blocking queue reads and emits
    nine new BNO085 telemetry keys plus the `robot_state` byte.
  - `StartControlTask` rewritten around the `RobotState` machine:
    - Sense → encoders + IMU globals + tick delta
    - Read command queue (non-blocking) — first non-ESTOP command
      promotes `IDLE/STOP → RUNNING`; `x_desired == ESTOP_CMD_KEY`
      latches `ESTOP`.
    - Watchdog — `RUNNING → STOP` if no command for
      `CMD_WATCHDOG_TIMEOUT_MS` (500 ms), motors zeroed and braked.
    - Compute u[] either by twist-mode IK (`x_desired`, `y_desired`,
      `phi_end` non-zero, all u_desired = 0) or per-wheel passthrough.
    - Apply `g_wheel_sign[i]` on the command side; per-wheel PID on
      `wheel_sign[i]*u_cmd − ω_meas`; deadzone/saturation; signed PWM
      split into direction + duty.
    - Production path drives `TIM5/12/14/15` and `setMotorDirection()`
      on `PD4/5/6/7` + `PE2/4/3/6`.
    - `DEBUG_LEDS=1` path holds all H-bridge pins in brake, zeros all
      PWM compare registers, and instead lights LD1 (M0||M2 active)
      and LD2 (M1||M3 active). LD3 (PB14) is skipped because PB14 is
      M1's PWM output (TIM12_CH1).
    - Odometry: undo `wheel_sign` on measured ω, call
      `globalSpeedsFromUMecanum(odom->phi, x_off, y_off, radius, …)`,
      integrate q_dot into `odom`, wrap φ ∈ (−π, π].
  - `StartDefaultTask` body reduced to an idle stub (was the legacy
    MCP2515/ODrive test loop).
  - `MutexUART_DataHandle` is still created (CubeMX-generated) but never
    used — left in place to avoid touching CubeMX-generated init code.

- **`CM7/.cproject`**
  - Added `../Core/Inc/sh2` to the assembler and C-compiler include
    paths in both Debug and Release configurations.

### Files not deleted but now dead in the merged build

- `CM7/Core/Inc/bno055.h`, `CM7/Core/Inc/bno055_stm32.h`,
  `CM7/Core/Src/bno055.c` — BNO055 driver. Still compiled. Its glue
  functions live in `bno055_stm32.h` which `main.c` keeps including so
  the link resolves. `--gc-sections` strips the bodies.
- `CM7/Core/Inc/mcp2515.h`, `CM7/Core/Inc/mcp2515_consts.h`,
  `CM7/Core/Inc/can.h`, `CM7/Core/Src/mcp2515.c` — MCP2515 driver.
  Still compiled (depends on `hspi1` which is still declared) but the
  symbols are unreferenced and stripped at link.
- `MX_SPI1_Init()` function body remains (CubeMX-generated) but the
  call from `main()` is commented out.

---

## FINAL TELEMETRY FORMAT (STM32 → host, USART3, 115200 8-N-1)

One line per ~100 Hz cycle from `Start_UART_TX_Task`, comma-separated
`key=value` pairs, `\r\n`-terminated. Field order (matches the order in
the `printf`):

```
x_desired, y_desired, phi_desired, d, r,
roll, pitch, yaw,                                ← BNO085 rotation-vector, radians
IMU_qx, IMU_qy, IMU_qz, IMU_qw,                  ← quaternion (ROS x,y,z,w)
IMU_wx, IMU_wy, IMU_wz,                          ← gyro, rad/s, body frame
IMU_ax, IMU_ay, IMU_az,                          ← linear accel, m/s², gravity removed
TIM1, TIM2, TIM4, TIM8,                          ← raw uint16 encoder counters
Enc_Wheel_Omega1..4,                             ← rev/s per wheel (signed)
Inertial_ang_vel_calc,
Inertial_x_vel_calc, Inertial_y_vel_calc,        ← odom q_dot
ODOM_phi, ODOM_x_pos, ODOM_y_pos,                ← integrated pose (φ wrapped)
ODOM_Err_x, ODOM_Err_y, ODOM_Err_phi,
U_Err_1..4,                                      ← per-wheel ω error (cmd − meas)
Ctrl_Inertial_x_dot, Ctrl_Inertial_y_dot,
Ctrl_Inertial_phi_dot,
Ctrl_necc_u1..4,                                 ← commanded wheel ω (rev/s, pre-sign)
ts_current, ts_previous, ts_delta,               ← FreeRTOS ticks (ms)
Ctrl_duty_u1..4,                                 ← uint16, 0..19999, post-deadzone
robot_state,                                     ← 0=IDLE, 1=RUNNING, 2=STOP, 3=ESTOP
xKp, xKi, xKd,  yKp, yKi, yKd,  phiKp, phiKi, phiKd,
u0Kp, u0Ki, u0Kd,  u1Kp, u1Ki, u1Kd,
u2Kp, u2Ki, u2Kd,  u3Kp, u3Ki, u3Kd
```

The original PEDRO telemetry keys are all preserved — the host
`serial_comm/serial_communication.py` regex `(\w+)=(...)` picks up any
new keys via its dict-based decoder, and the new keys are additive
(`IMU_q*`, `IMU_w*`, `IMU_a*`, `robot_state`).

---

## FINAL UART COMMAND FORMAT (host → STM32, USART3, 115200 8-N-1)

`start_UART_RX_Task` accepts the **same 30 space-separated floats** the
original PEDRO firmware accepted, `\n` or `\r` terminated. Field order:

```
x_desired  y_desired  phi_end  d  r
u1_desired  u2_desired  u3_desired  u4_desired
x_Kp  x_Ki  x_Kd
y_Kp  y_Ki  y_Kd
phi_Kp phi_Ki phi_Kd
u0_Kp u0_Ki u0_Kd
u1_Kp u1_Ki u1_Kd
u2_Kp u2_Ki u2_Kd
u3_Kp u3_Ki u3_Kd
```

Types: `x_desired..r` are `double`; `u1..u4_desired` and all 21 PID
gains are `float`. The host node `pedro_ws/omnibase_ws/src/serial_comm/
serial_comm/serial_communication.py` already produces this line at
10 Hz — no client-side change required.

Semantics inside `StartControlTask`:

- If `u1..u4_desired` are **all zero** and at least one of
  `x_desired/y_desired/phi_end` is non-zero, the firmware interprets the
  line as a body-frame twist: `vx = x_desired`, `vy = y_desired`,
  `wz = phi_end`, and runs `computeNecessaryWheelSpeedsMecanum(phi=0,
  0.195, 0.195, 0.0762, …)` to fill `u[0..3]`. This matches the OMNIBASE
  Type-1 SET_VEL semantics with the corrected mecanum signs.
- Otherwise the firmware treats `u1..u4_desired` as direct per-wheel
  angular-velocity setpoints (PEDRO's original behaviour).
- **ESTOP sentinel**: any line with `x_desired == -9999.0`
  (`ESTOP_CMD_KEY`) latches `ROBOT_STATE_ESTOP` for the remainder of the
  session — only an MCU reset clears it.
- Watchdog: if no successfully-parsed line arrives within 500 ms
  (`CMD_WATCHDOG_TIMEOUT_MS`) while in RUNNING, the SM falls to STOP and
  motors are zeroed. A subsequent valid command re-enters RUNNING.

The host node has no concept of "STOP" or "ESTOP" — those are emitted
purely by the firmware's `robot_state` telemetry field.

---

## STATE MACHINE — final transition table

```
                  ┌────────── any non-ESTOP cmd ─────────┐
                  │                                      ▼
   ┌──── IDLE ────┤                                  RUNNING ─── ESTOP cmd ──► ESTOP
   │              │                                      │
   │              └──── ESTOP cmd ──► ESTOP              ├── no cmd 500 ms ──► STOP
   │                                                      │
   │                                  ┌── any non-ESTOP cmd ─┘
   │                                  ▼
   │                               RUNNING
   │
   └──► (on boot / reset) initial state
```

- `STOP → RUNNING` on the next valid (non-ESTOP) command line.
- `ESTOP → *` requires a hardware reset.
- All non-RUNNING states force `dutyCycles[]=0`, `M_dirs[]=brake` and
  reset all per-wheel PID integral accumulators to zero.

---

## TODOS (unresolved questions / unknowns)

Each item below was not invented; the answer is genuinely unknown from
the available source code and needs hardware verification or a design
decision before the firmware ships.

1. **BNO085 wiring (PD14 INT, PD15 RST, PA4 WAKE).** These pins are
   borrowed from `STM32H7_OMNIBASE_CAN_BNO085` because no CubeMX
   schematic exists for the PEDRO frame with the BNO085 installed. They
   may need to change once the user wires the sensor into the real
   PEDRO board. `PD14` was MCP2515_CS in the original PEDRO design — we
   freed it by dropping the MCP2515 path. The `.ioc` file has not been
   regenerated to reflect this; next time CubeMX is opened it may
   re-overwrite `MX_GPIO_Init` if the user lets it.

2. **Wheel-index → physical wheel-position mapping.** Neither PEDRO
   nor OMNIBASE source labels which of FL/FR/RL/RR each `u[i]`
   corresponds to. We preserved the OMNIBASE wheel_sign pattern
   `{-1, +1, -1, +1}` verbatim, but the *physical* mounting of motors
   M0..M3 on PEDRO must be confirmed against OMNIBASE for the IK/FK to
   produce correct global frame motion. Two practical checks:
   - With a unit `x_desired = +1.0` (forward), all wheels should spin
     **forward** in the robot frame. After the `wheel_sign` flip,
     motors 0 and 2 will see a *negative* PWM command — verify that
     direction physically corresponds to "forward" for each wheel.
   - With a unit `phi_end = +1.0` (yaw CCW), the IK output sign pattern
     should be `(−, +, −, +)` after `wheel_sign`. Confirm this matches
     OMNIBASE's behaviour in hardware.

3. **PEDRO frame `x_off` / `y_off`.** Hardcoded to `0.195 m` (matching
   OMNIBASE's mecanum base). PEDRO may have a different wheelbase —
   measure and update the constants in `StartControlTask` and the
   `data` initialisers in `start_UART_RX_Task` / `Start_UART_TX_Task`.

4. **PEDRO wheel `radius`.** Hardcoded to `0.0762 m` (OMNIBASE value).
   Verify against the actual wheels mounted on PEDRO and update if
   different.

5. **PB14 conflict in DEBUG_LEDS mode.** Nucleo-H755 LD3 is `PB14`,
   which is also `TIM12_CH1` (M1 PWM). When `DEBUG_LEDS=1` we skip LD3
   to avoid double-driving the pin, but M1's PWM is still configured by
   `MX_TIM12_Init()` even though `__HAL_TIM_SET_COMPARE` writes 0 in
   the loop. If you want LD3 to actually light, comment out the
   `HAL_TIM_PWM_Start(&htim12, …)` call in `main()` under the same
   `#if DEBUG_LEDS` and add explicit LD3 toggling in the loop.

6. **R_counts = 424.0** is the small-motor encoder constant. The
   original PEDRO comment lists `17380.0` for "motores grandes". If the
   physical motors on PEDRO_OMNIBASE differ, update this constant.

7. **ESP32 Bluetooth path.** Not ported. USART2 (PD5/PD6) is consumed
   by H-bridge IN2/IN3 on PEDRO, so adding the BT path requires
   re-routing USART2 to alternate pins (PA2/PA3 candidate). Skipped
   per merge plan §3.

8. **No hardware IWDG/WWDG.** The watchdog is purely software (the
   500 ms command timeout in `StartControlTask`). If the firmware
   itself hangs (e.g. inside the SH2 service loop) the motors keep
   their last duty cycle. Consider enabling IWDG once the rest of the
   path is verified.

9. **ESTOP recovery.** Currently latches until MCU reset. If the host
   wants to clear ESTOP without a power-cycle, add a "clear" sentinel
   (e.g. `x_desired = -9998.0`) and a transition `ESTOP → IDLE`.

---

## SYNTAX / COMPILE CHECK

`arm-none-eabi-gcc` is not installed on this machine, so no compile
check ran. Manual review focus:

- `main.c` includes match the SH2 library headers exactly as in
  OMNIBASE (`#include "sh2.h"` etc.); the `../Core/Inc/sh2` include path
  is added in `CM7/.cproject`.
- All references to `bno055_*` and `mcp2515_*` functions are removed
  from the control-path code. `bno055_stm32.h` is still included to
  satisfy `bno055.c`'s link; `mcp2515.c` links against `hspi1` which is
  still declared.
- `MX_I2C1_Init()` is called from `main()`; `MX_SPI1_Init()` is not.
- `StartIMUTask` is forward-declared with the other task prototypes
  and instantiated in `main()` via `osThreadNew`.
- All Cortex-M7 atomic-float reads/writes for `g_bno085_*` are 32-bit
  aligned (file-scope `volatile float`); no mutex is required.
- Queue reads in `StartControlTask` and `Start_UART_TX_Task` are
  non-blocking (timeout = 0), so a stalled producer no longer deadlocks
  consumers (the old `osWaitForever` reads were a latent bug).

---

## 18. ROS 2 — `serial_comm` host node update (session 3)

### 18.1 Files changed

- `omnibase_ws/src/serial_comm/serial_comm/serial_communication.py` —
  rewritten to consume the session-2 telemetry/command format (the
  PEDRO 30-float command and the extended telemetry with `IMU_q*`,
  `IMU_w*`, `IMU_a*`, `robot_state`).
- `omnibase_ws/src/serial_comm/package.xml` — declared the previously
  implicit run-deps (`rclpy`, `std_msgs`, `geometry_msgs`,
  `sensor_msgs`, `nav_msgs`, `python3-serial`).
- `omnibase_ws/src/serial_comm/serial_comm/simple_rx.py` — unchanged.

### 18.2 Behaviour changes vs. previous `serial_comm`

| Area | Before | After |
|---|---|---|
| TX payload | Always parameter-driven: pose + per-wheel u + 21 PID gains. | Same 30-float layout, but `/cmd_vel` (`geometry_msgs/Twist`) now takes priority when active. Setting `use_cmd_vel=false` restores the parameter-only behaviour. |
| TX cadence | Hardcoded 10 Hz. | Parameter `tx_rate_hz` (default 10 Hz). |
| TX rounding | `round(v, 2)` then `' '.join(map(str, ...))` — could collapse a tiny twist to zero. | `round(v, 4)` and an explicit format that keeps small values; whole-number values still emitted as `1.0` so the firmware `sscanf("%lf %lf ... %f %f")` accepts them. |
| Serial config | `/dev/ttyACM0` 115200 hardcoded. | Parameters `serial_port`, `baud` (defaults unchanged). |
| RX regex | `(\w+)=([-+]?\d*\.\d+|\d+)` — silently dropped scientific notation. | `(\w+)=([-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)` — accepts `1e-3` too. |
| `/cmd_vel` watchdog | n/a | If no `/cmd_vel` for `cmd_vel_timeout` (default 0.5 s), the bridge stops echoing the last twist. The firmware still has its own 500 ms watchdog (§12, §FINAL UART COMMAND FORMAT). |
| ESTOP | n/a | `send_estop()` helper writes `x_desired = -9999.0` (= `ESTOP_CMD_KEY`). Not wired to a topic yet; exposed for future service/action. |
| Serial close on shutdown | Port left open. | `node.ser.close()` in `main()` finally-block. |

### 18.3 Topic inventory (after session 3)

Subscribers:

| Topic | Type | Notes |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | Linear x/y → body-frame `vx`, `vy`; angular z → `wz`. Disabled when parameter `use_cmd_vel=false`. |

Publishers — **legacy (kept verbatim for back-compat)**:

| Topic | Type |
|---|---|
| `stm32/raw` | `std_msgs/String` |
| `stm32_debug` | `std_msgs/String` (multi-line human summary, now includes `IMU_q*`, `IMU_w*`, `IMU_a*`, `robot_state`) |
| `stm32/pose` | `Float32MultiArray` `[x, y, phi, d, r]` |
| `stm32/imu` | `Float32MultiArray` `[roll, pitch, yaw]` |
| `stm32/omegas` | `Float32MultiArray` `[ω1..ω4]` (rev/s, signed) |
| `stm32/real_speeds` | `Float32MultiArray` `[φ_dot, x_dot, y_dot]` (world frame) |
| `stm32/odom` | `Float32MultiArray` `[phi, x, y]` |
| `stm32/errors` | `Float32MultiArray` `[dx, dy, dphi]` |
| `stm32/u_errors` | `Float32MultiArray` per-wheel ω errors |
| `stm32/ctrl_speeds` | `Float32MultiArray` `[x_dot, y_dot, phi_dot]` |
| `stm32/ctrl_u` | `Float32MultiArray` `[u1..u4]` |
| `stm32/pwm` | `Int32MultiArray` `[duty1..duty4]` |
| `stm32/timing` | `Int32MultiArray` `[ts_current, ts_previous, ts_delta]` |
| `stm32/encoders` | `Int32MultiArray` `[TIM1, TIM2, TIM4, TIM8]` |
| `stm32/pid_gains/{x,y,phi,u0..u3}` | `Float32MultiArray` `[Kp, Ki, Kd]` |

Publishers — **new (standard messages)**:

| Topic | Type | Frame / contents |
|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | `imu_link` (parameter `imu_frame`). Orientation = `IMU_qx,qy,qz,qw`. Angular vel = `IMU_w*` (rad/s, body). Linear accel = `IMU_a*` (m/s², gravity removed). Covariance unknown (`[0] = -1` per REP). |
| `odom` | `nav_msgs/Odometry` | header `odom_frame` (default `odom`), child `base_frame` (default `base_link`). Pose = `ODOM_{x_pos,y_pos,phi}`. Twist rotated from world to body using `ODOM_phi`. |
| `stm32/cmd_setpoint` | `geometry_msgs/TwistStamped` | Echo of `x_desired, y_desired, phi_desired` (body-frame) for plotting. |
| `stm32/robot_state` | `std_msgs/Int32` | 0=IDLE, 1=RUNNING, 2=STOP, 3=ESTOP. |
| `stm32/robot_state_name` | `std_msgs/String` | Human-readable mirror of `robot_state`. |
| `stm32/encoder/wheel{0..3}` | `std_msgs/Int32` | Per-wheel encoder counters (TIM1/TIM2/TIM4/TIM8). |
| `stm32/omega/wheel{0..3}` | `std_msgs/Float32` | Per-wheel `Enc_Wheel_Omega{1..4}`. |
| `stm32/pwm/wheel{0..3}` | `std_msgs/Int32` | Per-wheel `Ctrl_duty_u{1..4}`. |

### 18.4 Parameters

| Name | Default | Effect |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | pyserial device. |
| `baud` | `115200` | Must match USART3 (§4.1, §FINAL UART COMMAND FORMAT). |
| `tx_rate_hz` | `10.0` | TX cadence; the firmware watchdog is 500 ms so anything ≥ 2 Hz is safe. |
| `odom_frame` | `odom` | `Odometry.header.frame_id`. |
| `base_frame` | `base_link` | `Odometry.child_frame_id` and `TwistStamped.header.frame_id`. |
| `imu_frame` | `imu_link` | `Imu.header.frame_id`. |
| `use_cmd_vel` | `true` | If false, the node ignores `/cmd_vel` and only sends parameter-driven setpoints. |
| `cmd_vel_timeout` | `0.5` (s) | Host-side fresh-command window. Independent of the firmware's 500 ms watchdog (`CMD_WATCHDOG_TIMEOUT_MS`). |
| Pose setpoints | `x=0, y=0, phi=0, d=0.195, r=0.0762` | `d` is overloaded as `x_off` in the merged firmware; `r` is wheel radius. |
| Per-wheel setpoints | `u1..u4_desired = 0.0` | Used only when `use_cmd_vel=false` and no twist is active. |
| PID gains | unchanged from previous defaults | 7 × {Kp, Ki, Kd} sent on every TX frame. |

### 18.5 Firmware-format adjustments — **none required**

The session-2 telemetry/command format works cleanly for ROS 2. The
host node is the only place that needed change. Some cosmetic mismatches
remain but are not worth a firmware change:

1. The telemetry key for the heading setpoint is `phi_desired` while
   the command field is `phi_end`. The host parses both as floats; no
   ambiguity. **No firmware change.**
2. `Inertial_x_vel_calc` / `Inertial_y_vel_calc` are world-frame
   (`q_dot[1]`, `q_dot[2]` in §11). `nav_msgs/Odometry.twist` is
   conventionally in `child_frame_id`. The host rotates world→body
   using `ODOM_phi`, so `odom.twist.twist.linear.{x,y}` is body-frame.
   The firmware could instead emit `vx_body, vy_body` (already
   computed inside `globalSpeedsFromUMecanum`) as additional keys —
   marginal benefit, would shave a sin/cos per cycle on the host.
   **Optional future cleanup**; logged as TODO #10 below.
3. `Enc_Wheel_Omega{N}` is rev/s; `IMU_w*` is rad/s. Mixed units are
   acceptable as long as a downstream consumer reading the back-compat
   `stm32/omegas` topic knows the convention — documented here.

### 18.6 Verification

```
$ cd omnibase_ws && colcon build --packages-select serial_comm --symlink-install
Starting >>> serial_comm
Finished <<< serial_comm [0.96s]
Summary: 1 package finished [1.07s]
```

`pyflakes` on the rewritten module — clean. Module-load smoke test
(import + `yaw_to_quaternion(π/2)`) — clean. No hardware-in-the-loop
test was performed (no STM32 connected).

### 18.7 Added to TODOS (unresolved questions)

10. **Optional: emit body-frame velocities from firmware.** Inside
    `globalSpeedsFromUMecanum` the firmware already computes
    `vx_body` and `vy_body` but only the world-frame `q_dot[1..2]`
    are printed. Adding `Body_x_vel_calc` and `Body_y_vel_calc` keys
    to `Start_UART_TX_Task` would let the host publish
    `Odometry.twist` directly without a sin/cos rotation. Marginal
    win; touches only the printf format string.

---

## 19. ROS 2 — PEDRO dashboard node update (session 4)

### 19.1 Files changed

- `omnibase_ws/src/serial_comm/serial_comm/pedro_dashboard.py` —
  new ROS 2 dashboard node. It subscribes to the topics published by
  `serial_communication.py` and serves a Socket.IO web dashboard.
- `omnibase_ws/src/serial_comm/serial_comm/dashboard.html` — replaced
  the ODrive/CAN-oriented UI with a PEDRO H-bridge/PWM/encoder display.
- `omnibase_ws/src/serial_comm/setup.py` — added the
  `pedro_dashboard` console script.
- `omnibase_ws/src/serial_comm/package.xml` — added web dashboard run
  dependencies (`python3-flask`, `python3-flask-socketio`).

### 19.2 Dashboard node

Console script:

```
ros2 run serial_comm pedro_dashboard
```

Default web UI:

```
http://localhost:5000
```

Parameters:

| Name | Default | Effect |
|---|---|---|
| `enable_web_gui` | `true` | Starts the Flask + Flask-SocketIO dashboard server. |
| `web_gui_port` | `5000` | TCP port for the web dashboard. |
| `stale_timeout_s` | `0.5` | Telemetry age threshold for `STALE`. |
| `lost_timeout_s` | `2.0` | Telemetry age threshold for `LOST`. |

### 19.3 Subscribed topics

The dashboard does **not** open the STM32 serial port. It only consumes
ROS topics already published by `serial_communication.py`:

| Topic | Type | Dashboard use |
|---|---|---|
| `stm32/raw` | `std_msgs/String` | Telemetry freshness and raw preview. |
| `stm32/cmd_setpoint` | `geometry_msgs/TwistStamped` | Commanded `vx`, `vy`, `wz`. |
| `imu/data` | `sensor_msgs/Imu` | Orientation, gyro, linear accel. |
| `stm32/encoders` | `std_msgs/Int32MultiArray` | TIM1/TIM2/TIM4/TIM8 counters. |
| `stm32/omegas` | `std_msgs/Float32MultiArray` | Per-wheel measured speed. |
| `odom` | `nav_msgs/Odometry` | Pose and body-frame twist. |
| `stm32/pwm` | `std_msgs/Int32MultiArray` | Per-wheel PWM duty, 0..19999. |
| `stm32/ctrl_u` | `std_msgs/Float32MultiArray` | Commanded wheel speeds. |
| `stm32/errors` | `std_msgs/Float32MultiArray` | Pose error. |
| `stm32/u_errors` | `std_msgs/Float32MultiArray` | Per-wheel velocity error. |
| `stm32/robot_state` | `std_msgs/Int32` | State id, 0=IDLE, 1=RUNNING, 2=STOP, 3=ESTOP. |
| `stm32/robot_state_name` | `std_msgs/String` | State label. |
| `stm32_debug` | `std_msgs/String` | Lightweight error-flag hints (`ESTOP`, `STOP`, `ERROR`, `WATCHDOG`, `TIMEOUT`). |

### 19.4 Displayed data

The web dashboard displays:

- Connection status derived from time since the last `stm32/raw`
  message: `UNKNOWN`, `OK`, `STALE`, or `LOST`.
- Commanded body velocity (`vx`, `vy`, `wz`).
- Odometry pose (`x`, `y`, yaw) and body twist.
- IMU roll/pitch/yaw plus gyro values.
- Four motor panels with raw encoder count, measured wheel speed,
  wheel error, PWM duty, and a duty-cycle bar scaled to the 19999
  firmware PWM period.
- Robot state machine status and error flags.
- Raw STM32 telemetry preview.

All ODrive/CAN-specific fields and actions were removed: no axis state,
node id, bus voltage/current, ODrive configuration, CAN status, startup,
clear-errors, reboot, or calibration controls remain in the PEDRO UI.

### 19.5 Verification

`python3 -m py_compile` passed for
`serial_comm/serial_comm/pedro_dashboard.py` and
`serial_comm/serial_comm/serial_communication.py`.

The ROS 2 package build also passed:

```
cd /home/roger/Github/pedro_ws/omnibase_ws
colcon build --packages-select serial_comm --symlink-install
```

Result:

```
Finished <<< serial_comm [1min 1s]
Summary: 1 package finished [1min 1s]
```
