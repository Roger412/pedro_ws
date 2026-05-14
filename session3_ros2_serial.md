Read status.md first:
/home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE/status.md

Focus on the sections describing the final STM32 telemetry format and UART command format.

Now modify ONLY:
/home/roger/Github/pedro_ws/omnibase_ws/src/serial_comm

Tasks:
1. Update serial_communication.py to:
   - Send UART commands in the format the STM32 firmware expects (from status.md)
   - Parse incoming STM32 telemetry in the format defined in status.md
   - Publish ROS 2 topics for all available data:
     * Commanded velocities (geometry_msgs/Twist or equivalent)
     * IMU data (sensor_msgs/Imu)
     * Encoder data (per wheel)
     * Odometry (nav_msgs/Odometry)
     * Motor/PWM status
     * State machine status
     * Error/status flags

2. If the telemetry format in the firmware needs adjustment to make the ROS 2 side cleaner, note the required change clearly and update status.md. Do not silently change the firmware format without documenting it.

3. Make sure all existing useful functionality in serial_comm is preserved.

After changes:
- Verify there are no broken imports or missing dependencies
- Try to build/lint the package if ROS 2 toolchain is available
- Update status.md with what was changed in the ROS 2 package
