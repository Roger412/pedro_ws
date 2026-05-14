Read status.md first:
/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE/status.md

This is your reference for the analysis already done. Do not re-analyze from scratch.

Now modify ONLY the target project:
/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE

Rules:
- Never modify STM32H7_PEDRO (the original base)
- Reuse all existing CubeMX-generated pin and timer definitions from main.h and main.c
- Never invent pin assignments; mark genuinely unknown items as TODO
- The mecanum wheel mapping (motor index, wheel position, sign convention) must match OMNIBASE_CAN_BNO085 CM7 main.c exactly

Tasks:
1. Copy/adapt the BNO085 I2C driver files from OMNIBASE_CAN_BNO085 into the target CM7/Inc and CM7/Src folders
2. Implement the state machine (IDLE, RUNNING, STOP, ESTOP) in main.c, adapted for PWM+L298N, not ODrive
3. Implement mecanum kinematics in main.c using the exact wheel mapping from OMNIBASE_CAN_BNO085
4. Implement PWM + digital direction output for 2 L298N H-bridges controlling 4 motors, using existing PEDRO pin definitions
5. Implement UART command reception and parsing, matching the format expected by serial_comm
6. Implement encoder reading using the encoder timer logic already in the PEDRO project
7. Implement odometry calculation if encoder data is sufficient
8. Implement telemetry transmission back over UART
9. Implement watchdog/timeout/emergency stop behavior
10. Add a debug/test mode where motor outputs drive LEDs instead of real motors, with a clear #define or flag to switch modes

After all changes, update status.md:
- List every file modified or added
- Document the final telemetry format
- Document the final UART command format
- List all TODOs with the reason each item is unknown

Then run a syntax/compile check if the STM32 toolchain is available, or review the code for obvious errors if not.
