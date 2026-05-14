# STM32H7_PEDRO_OMNIBASE

## Project Overview
Mecanum-wheeled robot base. Target of migration from OMNIBASE_CAN_BNO085.
- MCU: STM32H7 (CM7 core)
- Motors: 4x DC with encoders, 2x L298N H-bridges
- IMU: BNO085 over I2C
- Comms: UART to ROS 2

## Key Rules
- NEVER modify STM32H7_PEDRO (original base), only STM32H7_PEDRO_OMNIBASE
- Mecanum wheel mapping MUST match OMNIBASE_CAN_BNO085/CM7/main.c exactly
- Reuse existing CubeMX pin definitions, never invent pin assignments
- Mark missing hardware constants as TODO, don't guess

## Repo Layout
- Firmware source: CM7/Inc and CM7/Src
- Reference omnibase: /home/roger/Github/home-custom-base/firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7
- ROS 2 workspace: /home/roger/Github/pedro_ws/omnibase_ws/src/serial_comm
