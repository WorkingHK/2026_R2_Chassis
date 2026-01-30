# WT901C485 IMU Integration Guide

## Hardware Overview

The WT901C485 is a 9-axis IMU (accelerometer, gyroscope, magnetometer) with Modbus protocol support. It provides:
- 3-axis acceleration
- 3-axis angular velocity (gyroscope)
- 3-axis orientation (roll, pitch, yaw)
- 3-axis magnetic field

## Connection Options

### Option 1: Direct UART (Recommended for ESP32)

**Pros:**
- Simpler wiring (no RS485 converter needed)
- Fewer components
- Lower cost
- Direct connection to ESP32 UART

**Cons:**
- Limited cable length (~1-2 meters max)
- Not suitable for noisy industrial environments

**Wiring:**
```
WT901C485          ESP32
---------          -----
VCC (5V)    --->   5V or 3.3V (check IMU voltage rating)
GND         --->   GND
TX          --->   RX2 (GPIO 16)
RX          --->   TX2 (GPIO 17)
```

**Note:** The WT901C485 can work in standard UART mode without RS485. Check if your module supports 3.3V or needs 5V.

### Option 2: RS485/Modbus (Industrial/Long Distance)

**Pros:**
- Long cable runs (up to 1200m)
- Noise immunity
- Multi-drop capability (multiple sensors on one bus)

**Cons:**
- Requires RS485 converter module (e.g., HW-97, MAX485)
- More complex wiring
- Needs DE/RE control pin

**Wiring with MAX485/HW-97 Module:**
```
WT901C485          MAX485          ESP32
---------          ------          -----
VCC         --->                   5V
GND         --->                   GND
A           --->   A
B           --->   B
                   VCC      --->   3.3V
                   GND      --->   GND
                   RO       --->   RX2 (GPIO 16)
                   DI       --->   TX2 (GPIO 17)
                   DE+RE    --->   GPIO 21 (control pin)
```

## Recommended Configuration for Your Robot

**Use Option 1 (Direct UART)** because:
1. Short cable distance on robot chassis
2. Simpler implementation
3. Your ESP32 has available UART2 (Serial2)
4. CAN bus already handles motor communication

## Pin Assignment

Current pin usage:
- GPIO 4: CAN RX (motors)
- GPIO 5: CAN TX (motors)
- GPIO 16: Available for IMU RX
- GPIO 17: Available for IMU TX

Proposed IMU pins:
```cpp
#define IMU_RX_GPIO 16  // ESP32 RX2 <- IMU TX
#define IMU_TX_GPIO 17  // ESP32 TX2 -> IMU RX
#define IMU_BAUD 115200 // Default baud rate
```

## Software Integration

### 1. IMU Library Structure
```
imu_wt901.h/cpp  - Wrapper for WitMotion SDK
  ├─ init()
  ├─ update()
  ├─ getYaw()
  ├─ getRoll()
  ├─ getPitch()
  ├─ getAccel()
  └─ getGyro()
```

### 2. Heading Correction
Use yaw angle to maintain straight line:
```cpp
// PID controller for heading correction
float targetYaw = getCurrentYaw();  // Lock heading when starting straight motion
float yawError = targetYaw - getCurrentYaw();
float rotationCorrection = kP * yawError;
mecanumDrive(vx, vy, rotationCorrection, maxSpeed);
```

### 3. ROS2 Integration
Publish to standard ROS2 topics:
- `/imu/data` (sensor_msgs/Imu) - orientation, angular velocity, linear acceleration
- `/imu/mag` (sensor_msgs/MagneticField) - magnetometer data

### 4. Odometry Fusion
Combine with wheel odometry using:
- `robot_localization` package (EKF/UKF)
- Custom complementary filter
- Kalman filter

## Next Steps

1. Wire IMU to ESP32 using Option 1 (Direct UART)
2. Test IMU communication with simple read example
3. Integrate into mecanum_ps5.ino
4. Implement heading correction
5. Add micro-ROS publisher
6. Test and tune PID parameters
