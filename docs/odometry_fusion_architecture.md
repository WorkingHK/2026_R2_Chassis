# Odometry Fusion Architecture

## Overview

Odometry fusion combines multiple sensor sources to estimate the robot's position and orientation more accurately than any single sensor alone.

## Why Fusion?

### Wheel Odometry Alone
**Pros:**
- High frequency updates
- Good short-term accuracy
- Measures actual wheel motion

**Cons:**
- Wheel slip causes drift
- Accumulates error over time
- No absolute reference

### IMU Alone
**Pros:**
- No wheel slip issues
- Detects rotation accurately
- High update rate

**Cons:**
- Gyro drift over time
- Double integration of acceleration = huge position errors
- No absolute position reference

### Combined (Fusion)
**Result:**
- IMU corrects wheel slip
- Wheels provide velocity reference
- Complementary strengths
- Much better accuracy

## Sensor Comparison

| Sensor | Position | Velocity | Orientation | Drift |
|--------|----------|----------|-------------|-------|
| Wheel Encoders | ✓ (drifts) | ✓✓ | ✓ (drifts) | High with slip |
| IMU Gyro | ✗ | ✗ | ✓✓ | Medium |
| IMU Accel | ✗ | ✗ | ✗ | Very high |
| **Fusion** | ✓✓ | ✓✓ | ✓✓✓ | Low |

## Architecture Options

### Option 1: Simple Complementary Filter (Easiest)

**Concept:** Use IMU for orientation, wheels for position

```cpp
// Update orientation from IMU (high trust)
robot_yaw = imu.getYaw();

// Update position from wheel odometry
float wheel_vx, wheel_vy, wheel_w;
calculateWheelOdometry(&wheel_vx, &wheel_vy, &wheel_w);

// Use IMU yaw to correct wheel odometry
float global_vx = wheel_vx * cos(robot_yaw) - wheel_vy * sin(robot_yaw);
float global_vy = wheel_vx * sin(robot_yaw) + wheel_vy * cos(robot_yaw);

robot_x += global_vx * dt;
robot_y += global_vy * dt;
```

**Pros:**
- Simple to implement
- Low computational cost
- Works well for mecanum robots

**Cons:**
- No statistical optimality
- Fixed trust ratios
- Doesn't handle sensor noise optimally

### Option 2: Extended Kalman Filter (EKF) - Recommended

**Concept:** Statistically optimal fusion using covariance

**Implementation:** Use ROS2 `robot_localization` package

```bash
# On Orange Pi
sudo apt install ros-humble-robot-localization
```

**Configuration:** `robot_localization_config.yaml`
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0

    # Sensors
    odom0: /wheel_odometry
    odom0_config: [false, false, false,  # x, y, z position
                   false, false, false,  # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz velocity
                   false, false, true,   # roll, pitch, yaw rates
                   false, false, false]  # ax, ay, az acceleration

    imu0: /imu/data
    imu0_config: [false, false, false,   # position (IMU doesn't provide)
                  false, false, true,    # orientation (use yaw only)
                  false, false, false,   # velocity (IMU doesn't provide)
                  false, false, true,    # angular velocity (use yaw rate)
                  false, false, false]   # acceleration (too noisy)
```

**Pros:**
- Industry standard
- Handles sensor noise optimally
- Proven in thousands of robots
- Easy to tune

**Cons:**
- Requires ROS2 setup
- More complex than complementary filter
- Need to tune covariance matrices

### Option 3: Unscented Kalman Filter (UKF) - Advanced

**When to use:** If you have highly nonlinear motion or need maximum accuracy

**Pros:**
- Better than EKF for nonlinear systems
- More accurate

**Cons:**
- More computationally expensive
- Overkill for most mecanum robots

## Recommended Implementation Path

### Phase 1: IMU-Only Heading (DONE ✓)
You've already implemented this in `mecanum_ps5_imu.ino`:
- Use IMU yaw for heading correction
- Simple and effective for straight driving

### Phase 2: Add Wheel Odometry
**Next step:** Calculate position from wheel speeds

```cpp
// In your mecanum code, add:
void updateWheelOdometry(float fl, float fr, float rl, float rr, float dt) {
  // Mecanum inverse kinematics
  float vx = (fl + fr + rl + rr) / 4.0;
  float vy = (-fl + fr + rl - rr) / 4.0;
  float w = (-fl + fr - rl + rr) / (4.0 * (WHEEL_BASE + TRACK_WIDTH));

  // Use IMU yaw for global frame
  float yaw = imu.getYaw() * DEG_TO_RAD;

  // Transform to global frame
  odom_x += (vx * cos(yaw) - vy * sin(yaw)) * dt;
  odom_y += (vx * sin(yaw) + vy * cos(yaw)) * dt;
  odom_yaw = yaw;  // From IMU
}
```

### Phase 3: Publish Odometry to ROS2
**Add to your micro-ROS code:**

```cpp
#include <nav_msgs/msg/odometry.h>

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

// In setup:
rclc_publisher_init_default(
  &odom_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  "wheel_odometry"
);

// In loop (50Hz):
void publishOdometry() {
  odom_msg.header.stamp.sec = millis() / 1000;
  odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  odom_msg.header.frame_id.data = "odom";
  odom_msg.child_frame_id.data = "base_link";

  // Position
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (from IMU)
  odom_msg.pose.pose.orientation = imu_msg.orientation;

  // Velocity
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = imu.getGyroZ() * DEG_TO_RAD;

  rcl_publish(&odom_publisher, &odom_msg, NULL);
}
```

### Phase 4: ROS2 Fusion (On Orange Pi)

**Launch file:** `mecanum_localization.launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/path/to/robot_localization_config.yaml']
        )
    ])
```

**Result:**
- ESP32 publishes: `/wheel_odometry` and `/imu/data`
- Orange Pi runs: `ekf_node`
- Output: `/odometry/filtered` (fused, accurate odometry)

## Tuning Guide

### Covariance Matrices

**Wheel Odometry Covariance** (how much to trust wheels):
```yaml
# Low values = high trust
# High values = low trust

# Good starting point for mecanum:
odom0_pose_covariance: [0.1, 0,   0,   0,   0,   0,
                        0,   0.1, 0,   0,   0,   0,
                        0,   0,   1e6, 0,   0,   0,
                        0,   0,   0,   1e6, 0,   0,
                        0,   0,   0,   0,   1e6, 0,
                        0,   0,   0,   0,   0,   0.5]
```

**IMU Covariance** (how much to trust IMU):
```yaml
# Yaw is accurate, roll/pitch less so on moving robot
imu0_orientation_covariance: [1e6, 0,   0,
                              0,   1e6, 0,
                              0,   0,   0.01]  # Trust yaw

imu0_angular_velocity_covariance: [1e6, 0,   0,
                                   0,   1e6, 0,
                                   0,   0,   0.05]  # Trust yaw rate
```

### Tuning Process

1. **Start conservative** (high covariances = low trust)
2. **Test straight line driving**
   - If drifts left/right: increase wheel covariance, decrease IMU yaw covariance
3. **Test rotation**
   - If rotation estimate lags: decrease IMU angular velocity covariance
4. **Test figure-8 pattern**
   - Check for accumulated error
5. **Iterate**

## Expected Performance

### Without Fusion
- Position error: 5-10% of distance traveled
- Orientation error: 2-5° per 360° rotation

### With Fusion
- Position error: 1-2% of distance traveled
- Orientation error: 0.5-1° per 360° rotation

## Next Steps

1. ✓ IMU integration (completed)
2. ✓ IMU publishing to ROS2 (completed)
3. **TODO:** Add wheel odometry calculation
4. **TODO:** Publish wheel odometry to ROS2
5. **TODO:** Set up robot_localization on Orange Pi
6. **TODO:** Tune covariance matrices
7. **TODO:** Test and validate

## References

- [robot_localization documentation](http://docs.ros.org/en/humble/p/robot_localization/)
- [REP-105: Coordinate Frames](https://www.ros.org/reps/rep-0105.html)
- [Kalman Filter Tutorial](https://www.kalmanfilter.net/)
