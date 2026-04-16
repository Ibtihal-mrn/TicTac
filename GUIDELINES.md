# TicTac / Eurobot Robot Guidelines

## Project Summary
This project controls an Eurobot robot based on 2 **ESP32-S3*:

- **robot_master**: main robot brain
- **sensor_hub**: ultrasonic + stop sensor coprocessor

The robot is driven by:
- **FreeRTOS tasks**
- **BLE command bridge**
- **FSM command dispatcher**
- **PID motion control**
- **encoders + IMU feedback**
- **I2C communication with the sensor hub**

## Hardware Stack
- **MCU**: ESP32-S3 DevKitC-1
- **Motors**: GB37-100 DC gear motors
- **Motor driver**: PWM + direction pins
- **Encoders**: left/right wheel encoders
- **IMU**: MPU6050
- **Ultrasonic sensors**: connected to the sensor hub
- **Emergency stop**: handled by sensor hub and FSM
- **BLE**: Nordic UART Service for PC ↔ robot commands

## Software Stack
- **PlatformIO** for building and flashing
- **Arduino framework**
- **NimBLE-Arduino** for BLE
- **FreeRTOS queues** for command transport
- **Python BLE sender/UI** on the PC side

## Main Architecture
### `robot_master`
Runs:
- BLE bridge
- FSM
- PID motion
- motors
- encoders
- IMU
- command queue consumer

### `sensor_hub`
Runs:
- ultrasonic sensors
- obstacle detection
- stop signal generation
- I2C slave communication with the master

## Command Flow
1. PC sends BLE text command
2. `BLEBridge` parses command and pushes it into the command queue
3. `fsm.cpp` reads queue commands
4. FSM dispatches to motion functions
5. `pid.cpp` handles linear/rotate motion
6. Motors are driven from encoder + IMU feedback
7. Sensor hub can trigger emergency stop independently

## Key Files
### Robot master
- `src/main.cpp` — boot, task creation, setup
- `lib/FSM/fsm.cpp` — FSM, command dispatch, test routines
- `lib/PID/pid.cpp` — linear/rotate PID control
- `lib/BLEBridge/BLEBridge.cpp` — BLE command parsing and log bridge
- `lib/encoders/` — encoder reading and distance helpers
- `lib/MPU6050/` — IMU init and gyro reading
- `lib/motors/` — motor driver helpers
- `src/config.h` — motor pins, encoder pins, geometry constants
- `src/globals.h` — shared flags like emergency stop

### Sensor hub
- `src/coprocessor_hub.cpp` — ultrasonic hub main loop
- `lib/UltrasonicFunction/` — ultrasonic sensor management
- `src/config_coprocessor.h` — hub pinout and sensor mapping

### PC tools
- `UI/ui.py` — Tkinter BLE UI
- `cerebros/ble_sender.py` — Python BLE bridge for command sending

## Build Environments
- `robot_master`: main robot firmware
- `sensor_hub`: coprocessor firmware

Keep these builds separated. Some libraries are master-only and must not be compiled into the hub.

## Motion Notes
- Linear and rotation use PID control.
- `maxPwm` is the command speed limit, not the target distance.
- Commands use signed control values:
  - positive = forward / turn direction A
  - negative = backward / turn direction B
- Minimum PWM exists to overcome motor deadband, but too much floor causes jitter or drift.

## Important Conventions
- Use `QueueHandle_t` for command queues, not `std::queue`.
- BLE `STOP` must interrupt motion immediately.
- Sensor-hub stop is separate from BLE stop logic.
- Prefer explicit command formats:
  - `FORWARD <distance_mm> <speed>`
  - `BACKWARD <distance_mm> <speed>`
  - `ROTATE <angle_deg> <speed>`
  - `WAIT <ms>`
  - `STOP`
  - `CLEAR`

## Debugging Tips
- Use Serial logs for master-side debugging.
- Enable PID/FSM debug flags when tuning motion.
- If the robot hums but does not move, check:
  - `PWM_MIN`
  - max PWM passed in the command
  - encoder direction
  - motor deadband
  - floor applied too close to target
- If the sensor hub build breaks, check for master-only includes leaking into shared headers.

## Recommended Reading Order for New AI
1. `src/main.cpp`
2. `lib/FSM/fsm.cpp`
3. `lib/BLEBridge/BLEBridge.cpp`
4. `lib/PID/pid.cpp`
5. `src/config.h`
6. `src/coprocessor_hub.cpp`
7. `src/config_coprocessor.h`

## Goal
When changing code, keep the architecture clean:
- master controls motion and command flow
- hub handles safety and sensors
- BLE is only the transport layer
- PID should stay simple, readable, and tunable