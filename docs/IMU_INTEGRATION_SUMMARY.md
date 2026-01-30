# IMU Integration Summary

## What Was Created

### 1. Documentation
- **`docs/imu_integration_guide.md`** - Wiring guide and integration overview
- **`docs/odometry_fusion_architecture.md`** - Advanced fusion strategies for ROS2

### 2. IMU Library
- **`imu_wt901.h`** - Header file with IMU interface
- **`imu_wt901.cpp`** - Implementation for WT901C485 IMU

### 3. Test Code
- **`phase3_tests/test_imu_basic.ino`** - Simple IMU test to verify communication

### 4. Integration Code
- **`mecanum_ps5_imu.ino`** - PS5 control + IMU heading correction
- **`mecanum_full_integration.ino`** - Complete system with ROS2 publishing

## Features Implemented

### ✓ Heading Correction for Straight Driving
- PID controller using IMU yaw angle
- L1 button to lock/unlock heading
- Automatic correction when driving straight
- Prevents drift during forward/backward motion

### ✓ ROS2 Integration
- Publishes `sensor_msgs/Imu` on `/imu/data` topic
- Publishes status messages on `/mecanum/status`
- 50Hz IMU data rate
- Full quaternion orientation
- Angular velocity and linear acceleration

### ✓ Odometry Fusion Architecture
- Documentation for integrating wheel encoders
- Guide for using `robot_localization` package
- EKF/UKF configuration examples
- Tuning guidelines

## Hardware Setup

### Recommended Wiring (Direct UART)
```
WT901C485          ESP32
---------          -----
VCC         --->   5V or 3.3V
GND         --->   GND
TX          --->   GPIO 16 (RX2)
RX          --->   GPIO 17 (TX2)
```

### Current Pin Usage
- GPIO 4: CAN RX (motors)
- GPIO 5: CAN TX (motors)
- GPIO 16: IMU RX
- GPIO 17: IMU TX

## Getting Started

### Step 1: Test IMU Communication
1. Wire IMU to ESP32 as shown above
2. Upload `phase3_tests/test_imu_basic.ino`
3. Open Serial Monitor (115200 baud)
4. Verify you see roll, pitch, yaw data

### Step 2: Test Heading Correction
1. Upload `mecanum_ps5_imu.ino`
2. Connect PS5 controller
3. Press L1 to lock heading
4. Drive forward - robot should maintain straight line
5. Tune PID parameters if needed:
   - `HEADING_KP` - increase for stronger correction
   - `HEADING_KD` - increase to reduce oscillation

### Step 3: Full ROS2 Integration
1. Upload `mecanum_full_integration.ino`
2. Connect ESP32 to Orange Pi via USB
3. Start micro-ROS agent:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```
4. Verify topics:
   ```bash
   ros2 topic list
   ros2 topic echo /imu/data
   ```

### Step 4: Odometry Fusion (Future)
Follow the guide in `docs/odometry_fusion_architecture.md` to:
1. Add wheel odometry calculation
2. Publish wheel odometry to ROS2
3. Set up `robot_localization` on Orange Pi
4. Tune covariance matrices

## PID Tuning Guide

### Current Parameters
```cpp
#define HEADING_KP 0.02f      // Proportional gain
#define HEADING_KI 0.0f       // Integral gain (disabled)
#define HEADING_KD 0.005f     // Derivative gain
#define HEADING_DEADZONE 2.0f // Degrees
```

### Tuning Process
1. **Start with KP only** (set KI=0, KD=0)
   - Increase KP until robot corrects drift
   - Too high = oscillation

2. **Add KD if oscillating**
   - Increase KD to dampen oscillation
   - Start with KD = KP/4

3. **Add KI if steady-state error**
   - Only if robot consistently drifts one direction
   - Start very small (0.001)

### Expected Behavior
- **Good tuning:** Smooth correction, minimal oscillation
- **KP too high:** Robot wobbles side-to-side
- **KP too low:** Robot drifts, slow correction
- **KD too high:** Sluggish response
- **KD too low:** Overshoots and oscillates

## Troubleshooting

### IMU Not Responding
- Check wiring (TX/RX might be swapped)
- Verify IMU voltage (3.3V or 5V?)
- Check baud rate (default 115200)
- Try different UART pins

### Heading Correction Not Working
- Verify IMU data is updating (check Serial output)
- Increase HEADING_KP
- Check that L1 button locks heading
- Ensure you're driving mostly straight (vx > 0.1)

### ROS2 Topics Not Appearing
- Verify micro-ROS agent is running
- Check USB connection
- Restart ESP32 after agent starts
- Check Serial output for errors

### Robot Oscillates When Driving Straight
- Decrease HEADING_KP
- Increase HEADING_KD
- Increase HEADING_DEADZONE

## Next Steps

1. **Test basic IMU** - Verify sensor works
2. **Test heading correction** - Tune PID parameters
3. **Test ROS2 publishing** - Verify data in ROS2
4. **Add wheel odometry** - Calculate position from wheels
5. **Implement fusion** - Use robot_localization
6. **Test and validate** - Drive patterns and measure accuracy

## Files Reference

| File | Purpose |
|------|---------|
| `imu_wt901.h/cpp` | IMU driver library |
| `mecanum_ps5_imu.ino` | Motors + PS5 + IMU heading correction |
| `mecanum_full_integration.ino` | Complete system with ROS2 |
| `test_imu_basic.ino` | Simple IMU test |
| `docs/imu_integration_guide.md` | Wiring and setup guide |
| `docs/odometry_fusion_architecture.md` | Advanced fusion guide |

## Questions?

Common questions answered in the documentation:
- Why use IMU? → Better straight-line driving, no wheel slip
- Why CRC/checksums? → Detect corrupted sensor data
- Why fusion? → Combine strengths of multiple sensors
- Which connection method? → Direct UART (simpler for robot)
