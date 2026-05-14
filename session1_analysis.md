Read and analyze the following projects. Do NOT modify any code in this session.

Projects to read:
- /home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO (CM7/Inc and CM7/Src, especially main.c)
- /home/roger/Github/home-custom-base/firmware/STM32H7_OMNIBASE_CAN_BNO085/CM7 (main.c and all Inc/Src files)
- /home/roger/Github/pedro_ws/omnibase_ws/src/serial_comm (all Python nodes)
- /home/roger/Github/home-custom-base/omnibase_ws/src/odrive_comm/odrive_comm/odrive_dashboard.py
- conversation.txt (if present in the working directory)

From the PEDRO project, extract and document:
- Full pinout as defined in main.h and main.c
- L298N H-bridge signal mapping (direction pins + PWM pins per motor)
- Timer/PWM configuration
- Encoder configuration and which timers are used
- UART configuration and any existing command/telemetry format
- FreeRTOS task structure if present
- Any IMU code if present

From the OMNIBASE_CAN_BNO085 project, extract and document:
- The exact mecanum wheel mapping from main.c: motor index, physical wheel position, sign convention
- The state machine states and transitions
- The BNO085 I2C task structure and all involved files
- The UART telemetry format and command parsing format
- Odometry calculation logic
- Watchdog/timeout/emergency stop behavior
- FreeRTOS task and queue structure
- Which files are ODrive-specific and should NOT be carried over
- Which files are reusable independently of ODrive

From serial_comm, extract:
- The expected UART command format sent to STM32
- The expected telemetry format received from STM32
- Which ROS 2 topics are published and subscribed

Write all findings into:
/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE/status.md

Structure status.md with clear sections for each item above.
End with two sections:
- MERGE PLAN: what will be copied/adapted from OMNIBASE into PEDRO_OMNIBASE
- REJECT LIST: what is ODrive/CAN-specific and will be excluded

Do not write any code yet. Analysis and documentation only.
