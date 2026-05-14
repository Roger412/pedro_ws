Read these files before doing anything else:
- /home/roger/Github/home-custom-base/omnibase_ws/src/odrive_comm/odrive_comm/odrive_dashboard.py
- /home/roger/Github/pedro_ws/firmware/STM32H7_PEDRO_OMNIBASE/status.md

Understand:
- What the ODrive dashboard does, what topics it subscribes to, how it displays data
- What data the PEDRO H-bridge robot actually has available (from status.md)

Then create a new dashboard node inside:
/home/roger/Github/pedro_ws/omnibase_ws/src/serial_comm

The dashboard should:
- Subscribe to the ROS 2 topics published by serial_communication.py
- Display: commanded velocities, IMU data, encoder values, odometry, motor/PWM status, state machine status, error flags
- Show a connection status indicator based on telemetry freshness (time since last STM32 message)
- Handle missing or stale data gracefully without crashing
- Remove all ODrive/CAN-specific fields (axis state, ODrive configuration, CAN status)
- Reflect the L298N/PWM/encoder architecture instead

Register the dashboard as a console_scripts entry point in setup.py or package.xml as appropriate.

After changes:
- Verify no broken imports
- Update status.md with the dashboard node name, how to launch it, and what it displays
