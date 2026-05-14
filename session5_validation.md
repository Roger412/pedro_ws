Read status.md:
/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE/status.md

Perform validation across all modified files.

STM32 firmware:
- Check CM7/Src and CM7/Inc for broken includes, missing definitions, or obvious logic errors
- Verify the mecanum wheel mapping in main.c matches what is documented in status.md
- Verify the telemetry format in firmware matches what is documented in status.md
- Verify the UART command parsing matches what serial_comm sends
- If STM32 toolchain (arm-none-eabi-gcc or STM32CubeIDE build) is available, attempt a build and report errors
- Verify the LED test mode is present and functional as a standalone test

ROS 2 package:
- Check serial_comm for broken imports and missing dependencies
- Verify topic names and message types are consistent between serial_communication.py and the dashboard node
- If ROS 2 toolchain is available, run: cd /home/roger/Github/pedro_ws/omnibase_ws && colcon build --packages-select serial_comm
- Report any build or lint errors

Cross-check:
- Confirm the STM32 telemetry format and the ROS 2 parser are consistent
- Confirm the ROS 2 command format and the STM32 command parser are consistent

Update status.md with:
- Validation results
- Any remaining errors or mismatches found
- Final TODO list with priority (hardware-dependent TODOs vs software TODOs)
