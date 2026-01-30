# Phase 3: micro-ROS Integration - Implementation Plan

## Executive Summary

This plan implements ROS 2 control for the mecanum robot while maintaining PS5 override capability. The approach uses **incremental testing** with standalone test code for each stage, building from basic micro-ROS connectivity to full arbitration logic.

**Timeline:** 5 implementation stages, each with dedicated test code
**Communication:** Serial/USB between ESP32 and Orange Pi (115200 baud)
**Safety:** PS5 always overrides ROS, timeout watchdogs prevent runaway

---

## Architecture Overview

### System Components

**ESP32 (Real-time Controller):**
- Motor control via CAN bus
- PS5 controller input handling
- micro-ROS node (subscribes to /cmd_vel)
- Arbitration logic (PS5 vs ROS priority)
- Safety watchdogs

**Orange Pi 5 Max (High-level Planner):**
- ROS 2 Humble
- micro-ROS agent (Serial/USB transport)
- Publishes /cmd_vel commands
- Future: Navigation stack

**Communication:**
- Serial/USB: ESP32 ↔ Orange Pi
- Baudrate: 115200 (reliable, tested)
- micro-ROS agent bridges Serial ↔ ROS 2 DDS

### Control Flow State Machine

```
┌─────────────┐
│   STARTUP   │
└──────┬──────┘
       │
       v
┌─────────────┐     PS5 Connected
│   STOPPED   │────────────────────┐
└──────┬──────┘                    │
       │                           v
       │ ROS Active         ┌──────────────┐
       └───────────────────>│ MANUAL_MODE  │
                            │ (PS5 Active) │
       ┌────────────────────┤              │
       │ PS5 Disconnected   └──────────────┘
       v
┌─────────────┐     ROS cmd_vel received
│  AUTO_MODE  │<────────────────────┐
│ (ROS Active)│                     │
└──────┬──────┘                     │
       │ Timeout (500ms)            │
       └────────────────────────────┘
       │
       v
┌─────────────┐
│   STOPPED   │
└─────────────┘
```

**Arbitration Rules:**
1. PS5 connected → MANUAL_MODE (always wins, ROS ignored)
2. PS5 disconnected + ROS active → AUTO_MODE
3. PS5 disconnected + ROS timeout → STOPPED
4. Any PS5 button press → instant MANUAL_MODE override

---

## Implementation Stages

### Stage 1: micro-ROS Basic Connectivity Test
**Goal:** Verify ESP32 can connect to micro-ROS agent and publish messages

**Test File:** `test_microros_connectivity.ino`

**What it tests:**
- Serial connection to micro-ROS agent
- Publisher creation
- Heartbeat message publishing to `/esp32/heartbeat`
- No motor control, no PS5

**Success Criteria:**
- ESP32 connects to micro-ROS agent
- `ros2 topic echo /esp32/heartbeat` shows messages
- No connection errors in serial monitor
- Built-in LED blinks with each publish

---

### Stage 2: /cmd_vel Subscription Test
**Goal:** Verify ESP32 can receive and parse /cmd_vel messages

**Test File:** `test_microros_cmdvel.ino`

**What it tests:**
- Subscribing to `/cmd_vel` (geometry_msgs/Twist)
- Parsing linear.x, linear.y, angular.z
- Timeout detection (500ms)
- No motor control, no PS5

**Success Criteria:**
- `ros2 topic pub /cmd_vel ...` commands appear in serial monitor
- Values parsed correctly
- Timeout warning appears 500ms after last command

---

### Stage 3: ROS-Only Motor Control Test
**Goal:** Verify ROS commands can drive motors

**Test File:** `test_microros_motors.ino`

**What it tests:**
- /cmd_vel → motor control pipeline
- Mecanum kinematics with ROS input
- Velocity scaling (ROS m/s → motor rad/s)
- Timeout safety (motors stop after 500ms)
- No PS5 integration

**Success Criteria:**
- `ros2 run teleop_twist_keyboard teleop_twist_keyboard` drives robot
- All motion types work (forward, strafe, rotate)
- Motors stop after 500ms without commands
- No CAN errors

---

### Stage 4: Arbitration Logic Test
**Goal:** Verify PS5 override and mode switching

**Test File:** `test_arbitration.ino`

**What it tests:**
- Full arbitration state machine
- PS5 input handling
- ROS /cmd_vel subscription
- Mode switching logic (STOPPED/MANUAL/AUTO)
- PS5 override behavior

**Success Criteria:**
- PS5 connected → MANUAL_MODE (ROS ignored)
- PS5 disconnected + ROS active → AUTO_MODE
- PS5 button press → instant override
- Timeout → STOPPED mode
- Mode transitions logged to serial

---

### Stage 5: Full Integration
**Goal:** Production-ready code with all features

**Final File:** `mecanum_ros.ino`

**What it includes:**
- All features from Stage 4
- Status publishing to `/robot/status`
- Enhanced diagnostics
- Optimized performance
- Ready for Phase 4 (odometry)

---

## Detailed Implementation Steps

### Step 1: Environment Setup

#### ESP32 Setup

**1.1 Install micro-ROS Arduino Library**

In Arduino IDE:
1. Sketch → Include Library → Manage Libraries
2. Search: "micro_ros_arduino"
3. Install: "micro_ros_arduino" by Pablo Garrido

**1.2 Configure Arduino IDE**
- Board: ESP32 Dev Module
- Upload Speed: 921600
- CPU Frequency: 240MHz
- Flash Size: 4MB
- Partition Scheme: Default 4MB with spiffs

**1.3 Required Libraries**
- micro_ros_arduino (for ROS 2 communication)
- PS5Controller (already installed from Phase 2)
- ESP32 TWAI driver (built-in)

#### Orange Pi 5 Max Setup

**2.1 Verify ROS 2 Humble Installation**

```bash
# Check ROS 2 is installed
source /opt/ros/humble/setup.bash
ros2 --version

# Should show: ros2 cli version: 0.18.x
```

**2.2 Install micro-ROS Agent**

```bash
# Install micro-ROS agent
sudo apt update
sudo apt install ros-humble-micro-ros-agent -y

# Verify installation
ros2 run micro_ros_agent micro_ros_agent --help
```

**2.3 Configure USB Permissions**

```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Create udev rule for ESP32
sudo tee /etc/udev/rules.d/99-esp32.rules > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# IMPORTANT: Logout and login for group changes to take effect
```

**2.4 Test Serial Connection**

```bash
# Find ESP32 device
ls -l /dev/ttyUSB* /dev/ttyACM*

# Should see something like /dev/ttyUSB0 or /dev/ttyACM0
```

**2.5 Create Launch Script**

Create `~/mecanum_ws/start_microros_agent.sh`:

```bash
#!/bin/bash
DEVICE="/dev/ttyUSB0"  # Change if different
BAUDRATE="115200"

echo "Starting micro-ROS agent..."
echo "Device: $DEVICE"
echo "Baudrate: $BAUDRATE"
echo ""

ros2 run micro_ros_agent micro_ros_agent serial --dev $DEVICE -b $BAUDRATE
```

Make executable:
```bash
chmod +x ~/mecanum_ws/start_microros_agent.sh
```

---

### Step 2: Stage 1 Implementation - Basic Connectivity

**Goal:** Verify ESP32 can connect to micro-ROS agent and publish heartbeat messages.

**Create:** `test_microros_connectivity.ino`

**Test Code:**

```cpp
/**
 * Stage 1: micro-ROS Basic Connectivity Test
 *
 * Tests:
 * - Serial connection to micro-ROS agent
 * - Publisher creation
 * - Heartbeat message publishing
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2  // Built-in LED

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  Stage 1: micro-ROS Connectivity Test");
  Serial.println("========================================\n");

  // Set micro-ROS transport to Serial
  set_microros_transports();
  delay(2000);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_test_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32/heartbeat"
  );

  // Allocate message memory
  msg.data.data = (char*)malloc(100 * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = 100;

  Serial.println("✓ micro-ROS initialized");
  Serial.println("✓ Publisher: /esp32/heartbeat");
  Serial.println("\nPublishing heartbeat...\n");
}

void loop() {
  static unsigned long lastPublish = 0;
  static int counter = 0;

  if (millis() - lastPublish >= 1000) {
    lastPublish = millis();

    sprintf(msg.data.data, "Heartbeat %d", counter++);
    msg.data.size = strlen(msg.data.data);

    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret == RCL_RET_OK) {
      Serial.printf("✓ Published: %s\n", msg.data.data);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    } else {
      Serial.printf("✗ Publish failed: %d\n", ret);
    }
  }
  delay(10);
}
```

**Testing Procedure:**

1. **Upload to ESP32:**
   - Open `test_microros_connectivity.ino` in Arduino IDE
   - Select ESP32 board and port
   - Upload

2. **Start micro-ROS agent on Orange Pi:**
   ```bash
   ~/mecanum_ws/start_microros_agent.sh
   ```

   Expected output:
   ```
   [1234567890.123456] info     | TermiosAgentLinux.cpp | init | running...
   [1234567890.234567] info     | Root.cpp | create_client | create
   ```

3. **Verify connection in ESP32 Serial Monitor:**
   ```
   ✓ micro-ROS initialized
   ✓ Publisher: /esp32/heartbeat

   Publishing heartbeat...

   ✓ Published: Heartbeat 0
   ✓ Published: Heartbeat 1
   ✓ Published: Heartbeat 2
   ```

4. **Check ROS 2 topics on Orange Pi:**
   ```bash
   # In new terminal
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

   Should see:
   ```
   /esp32/heartbeat
   /parameter_events
   /rosout
   ```

5. **Echo heartbeat messages:**
   ```bash
   ros2 topic echo /esp32/heartbeat
   ```

   Should see:
   ```
   data: 'Heartbeat 0'
   ---
   data: 'Heartbeat 1'
   ---
   ```

**Success Criteria:**
- ✓ ESP32 LED blinks every second
- ✓ Serial monitor shows successful publishes
- ✓ `ros2 topic list` shows `/esp32/heartbeat`
- ✓ `ros2 topic echo` shows messages
- ✓ No connection errors

**Troubleshooting:**
- **Agent can't connect:** Check USB device path (`ls /dev/ttyUSB*`)
- **No topics appear:** Verify baudrate is 115200 on both sides
- **Publish fails:** Check micro-ROS library version (need 2.0.5+)

---

### Step 3: Stage 2 Implementation - /cmd_vel Subscription

**Goal:** Verify ESP32 can receive and parse /cmd_vel messages.

**Create:** `test_microros_cmdvel.ino`

**Test Code:**

```cpp
/**
 * Stage 2: /cmd_vel Subscription Test
 *
 * Tests:
 * - Subscribing to /cmd_vel
 * - Parsing Twist messages
 * - Timeout detection
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// micro-ROS objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Velocity tracking
float current_vx = 0.0f;
float current_vy = 0.0f;
float current_wz = 0.0f;
unsigned long last_cmd_time = 0;

#define CMD_TIMEOUT_MS 500

// Callback for /cmd_vel
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  current_vx = msg->linear.x;
  current_vy = msg->linear.y;
  current_wz = msg->angular.z;
  last_cmd_time = millis();

  Serial.printf("ROS cmd_vel: vx=%.2f vy=%.2f wz=%.2f\n",
                current_vx, current_vy, current_wz);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  Stage 2: /cmd_vel Subscription Test");
  Serial.println("========================================\n");

  // Initialize micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_cmdvel_test", "", &support);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg,
                                  &cmdVelCallback, ON_NEW_DATA);

  Serial.println("✓ Subscribed to /cmd_vel");
  Serial.println("✓ Timeout watchdog: 500ms");
  Serial.println("\nWaiting for commands...\n");
}

void loop() {
  // Spin executor to process callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Check timeout
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS && last_cmd_time > 0) {
    if (current_vx != 0 || current_vy != 0 || current_wz != 0) {
      current_vx = 0;
      current_vy = 0;
      current_wz = 0;
      Serial.println("⚠ TIMEOUT: No cmd_vel for 500ms");
    }
  }

  // Status update every 2 seconds
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus >= 2000) {
    lastStatus = millis();

    if (last_cmd_time == 0) {
      Serial.println("Status: No commands received yet");
    } else {
      unsigned long age = millis() - last_cmd_time;
      Serial.printf("Status: Last cmd %lu ms ago | vx=%.2f vy=%.2f wz=%.2f\n",
                    age, current_vx, current_vy, current_wz);
    }
  }

  delay(10);
}
```

**Testing Procedure:**

1. **Upload to ESP32**

2. **Start micro-ROS agent**

3. **Publish test commands on Orange Pi:**

   ```bash
   # Forward
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

   # Strafe right
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

   # Rotate clockwise
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

   # Combined motion
   ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.5, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
   ```

4. **Verify ESP32 serial output:**
   ```
   ROS cmd_vel: vx=1.00 vy=0.00 wz=0.00
   Status: Last cmd 234 ms ago | vx=1.00 vy=0.00 wz=0.00
   ```

5. **Test timeout:**
   - Stop publishing commands
   - Wait 500ms
   - Should see: `⚠ TIMEOUT: No cmd_vel for 500ms`

**Success Criteria:**
- ✓ Commands appear in serial monitor with correct values
- ✓ Timeout warning appears 500ms after last command
- ✓ No crashes or connection drops
- ✓ All velocity components parsed correctly

---

### Step 4: Stage 3 Implementation - ROS Motor Control

**Goal:** Verify ROS commands can drive motors (no PS5 yet).

**Create:** `test_microros_motors.ino`

**Key Components:**
- Combines Stage 2 with CAN motor control from `mecanum_ps5.ino`
- Adds velocity scaling (ROS m/s → motor rad/s)
- Includes motor enable sequence and warm-up
- Timeout safety stops motors

**Test Code Structure:**

```cpp
/**
 * Stage 3: ROS-Only Motor Control Test
 *
 * Tests:
 * - /cmd_vel → motor control pipeline
 * - Mecanum kinematics with ROS input
 * - Timeout safety
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <driver/twai.h>
#include "motor_config.h"

// micro-ROS objects
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Velocity tracking
float ros_vx = 0.0f;
float ros_vy = 0.0f;
float ros_wz = 0.0f;
unsigned long last_ros_cmd_time = 0;

#define CMD_TIMEOUT_MS 500
#define MAX_LINEAR_SPEED 0.5f   // m/s
#define MAX_ANGULAR_SPEED 1.0f  // rad/s
#define MAX_MOTOR_SPEED 6.0f    // rad/s

// CAN initialization (from mecanum_ps5.ino)
void canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_GPIO,
    (gpio_num_t)CAN_RX_GPIO,
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

// Enable motor (from mecanum_ps5.ino)
bool enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 8;
  for (int i = 0; i < 7; i++) msg.data[i] = 0xFF;
  msg.data[7] = 0xFC;
  return (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);
}

// Send speed (from mecanum_ps5.ino)
bool sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;
  memcpy(msg.data, &speed, 4);
  return (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);
}

// Mecanum drive (from mecanum_ps5.ino)
void mecanumDrive(float vx, float vy, float w, float maxSpeed) {
  // Calculate wheel speeds
  float fl = vx - vy - w;
  float fr = vx + vy + w;
  float rl = vx + vy - w;
  float rr = vx - vy + w;

  // Normalize if needed
  float maxVal = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
  if (maxVal > 1.0f) {
    fl /= maxVal; fr /= maxVal;
    rl /= maxVal; rr /= maxVal;
  }

  // Scale and apply direction multipliers
  fl = fl * maxSpeed * MOTOR_FL_DIR;
  fr = fr * maxSpeed * MOTOR_FR_DIR;
  rl = rl * maxSpeed * MOTOR_RL_DIR;
  rr = rr * maxSpeed * MOTOR_RR_DIR;

  // Send to motors
  sendSpeed(MOTOR_FL, fl);
  sendSpeed(MOTOR_FR, fr);
  sendSpeed(MOTOR_RL, rl);
  sendSpeed(MOTOR_RR, rr);
}

// ROS callback
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  ros_vx = msg->linear.x / MAX_LINEAR_SPEED;   // Normalize
  ros_vy = msg->linear.y / MAX_LINEAR_SPEED;
  ros_wz = msg->angular.z / MAX_ANGULAR_SPEED;
  last_ros_cmd_time = millis();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  Stage 3: ROS Motor Control Test");
  Serial.println("========================================\n");

  // Initialize CAN
  canInit();
  delay(100);

  // Enable motors sequentially (CRITICAL!)
  Serial.println("Enabling motors...");
  enableMotor(MOTOR_FL); sendSpeed(MOTOR_FL, 0.0f); delay(50);
  enableMotor(MOTOR_FR); sendSpeed(MOTOR_FR, 0.0f); delay(50);
  enableMotor(MOTOR_RL); sendSpeed(MOTOR_RL, 0.0f); delay(50);
  enableMotor(MOTOR_RR); sendSpeed(MOTOR_RR, 0.0f); delay(50);
  Serial.println("✓ Motors enabled\n");

  // 2-second warm-up
  Serial.println("Motor warm-up (2s)...");
  unsigned long warmupStart = millis();
  while (millis() - warmupStart < 2000) {
    sendSpeed(MOTOR_FL, 0.0f);
    sendSpeed(MOTOR_FR, 0.0f);
    sendSpeed(MOTOR_RL, 0.0f);
    sendSpeed(MOTOR_RR, 0.0f);
    delay(100);
  }
  Serial.println("✓ Warm-up complete\n");

  // Initialize micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_control", "", &support);

  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg,
                                  &cmdVelCallback, ON_NEW_DATA);

  Serial.println("✓ Subscribed to /cmd_vel");
  Serial.println("✓ Ready for ROS commands\n");
}

void loop() {
  // Process ROS callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Check if ROS is active
  bool ros_active = (millis() - last_ros_cmd_time < CMD_TIMEOUT_MS) &&
                    (last_ros_cmd_time > 0);

  if (!ros_active) {
    // Stop motors
    mecanumDrive(0, 0, 0, MAX_MOTOR_SPEED);
  } else {
    // Drive motors with ROS input
    mecanumDrive(ros_vx, ros_vy, ros_wz, MAX_MOTOR_SPEED);
  }

  // Diagnostic output every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    if (ros_active) {
      Serial.printf("ROS ACTIVE: vx=%.2f vy=%.2f wz=%.2f\n",
                    ros_vx, ros_vy, ros_wz);
    } else {
      Serial.println("STOPPED: No ROS commands");
    }
  }

  delay(100);  // 10Hz control loop
}
```

**Testing Procedure:**

1. **SAFETY FIRST: Elevate robot (wheels off ground)**

2. **Upload to ESP32**

3. **Start micro-ROS agent**

4. **Test with teleop keyboard:**
   ```bash
   # Install if not already installed
   sudo apt install ros-humble-teleop-twist-keyboard

   # Run teleop
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

5. **Test all motion types:**
   - Press 'i' for forward
   - Press 'k' for stop
   - Press ',' for backward
   - Press 'j' for strafe left
   - Press 'l' for strafe right
   - Press 'u' for rotate CCW
   - Press 'o' for rotate CW

6. **Verify timeout:**
   - Stop sending commands (press 'k')
   - Wait 500ms
   - Serial should show: "STOPPED: No ROS commands"

**Success Criteria:**
- ✓ All wheels respond to ROS commands
- ✓ Forward/strafe/rotate work correctly
- ✓ Motors stop after 500ms timeout
- ✓ No CAN errors in serial monitor
- ✓ Smooth motion control

---

### Step 5: Stage 4 Implementation - Arbitration Logic

**Goal:** Verify PS5 override and mode switching work correctly.

**Create:** `test_arbitration.ino`

**Key Features:**
- Full state machine (STOPPED/MANUAL/AUTO)
- PS5 input handling
- ROS /cmd_vel subscription
- Mode switching logic
- Diagnostic output showing current mode

**State Machine:**
```
STOPPED → PS5 connected → MANUAL
STOPPED → ROS active → AUTO
MANUAL → PS5 disconnected → STOPPED
AUTO → ROS timeout → STOPPED
AUTO → PS5 connected → MANUAL (instant override)
```

**Test Code (Part 1 - Setup and Declarations):**

```cpp
/**
 * Stage 4: Arbitration Logic Test
 *
 * Tests:
 * - PS5 override (always wins)
 * - ROS control when PS5 disconnected
 * - Mode switching
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <driver/twai.h>
#include <PS5Controller.h>
#include "motor_config.h"

// Control modes
enum ControlMode {
  MODE_STOPPED,
  MODE_MANUAL,
  MODE_AUTO
};

ControlMode current_mode = MODE_STOPPED;
const char* mode_names[] = {"STOPPED", "MANUAL", "AUTO"};

// micro-ROS objects
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Velocity sources
float ps5_vx = 0.0f, ps5_vy = 0.0f, ps5_wz = 0.0f;
float ros_vx = 0.0f, ros_vy = 0.0f, ros_wz = 0.0f;
unsigned long last_ros_cmd_time = 0;

// Final output velocities
float output_vx = 0.0f, output_vy = 0.0f, output_wz = 0.0f;

#define CMD_TIMEOUT_MS 500
#define DEADZONE 0.05f
#define MAX_SPEED 6.0f
#define MAX_LINEAR_SPEED 0.5f
#define MAX_ANGULAR_SPEED 1.0f

// PS5 MAC address (replace with your controller's MAC)
#define PS5_MAC "00:00:00:00:00:00"

// CAN functions (same as Stage 3)
void canInit() { /* ... same as Stage 3 ... */ }
bool enableMotor(uint8_t id) { /* ... */ }
bool sendSpeed(uint8_t id, float speed) { /* ... */ }
void mecanumDrive(float vx, float vy, float w, float maxSpeed) { /* ... */ }

// ROS callback
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  ros_vx = msg->linear.x / MAX_LINEAR_SPEED;
  ros_vy = msg->linear.y / MAX_LINEAR_SPEED;
  ros_wz = msg->angular.z / MAX_ANGULAR_SPEED;
  last_ros_cmd_time = millis();
}

// Read PS5 input
void readPS5Input() {
  if (!PS5.isConnected()) {
    ps5_vx = ps5_vy = ps5_wz = 0;
    return;
  }

  ps5_vx = PS5.LStickY() / 128.0f;
  ps5_vy = PS5.LStickX() / 128.0f;
  ps5_wz = PS5.RStickX() / 128.0f;

  if (abs(ps5_vx) < DEADZONE) ps5_vx = 0;
  if (abs(ps5_vy) < DEADZONE) ps5_vy = 0;
  if (abs(ps5_wz) < DEADZONE) ps5_wz = 0;
}

// Arbitration logic
void updateControlMode() {
  ControlMode prev_mode = current_mode;

  // Rule 1: PS5 connected → MANUAL
  if (PS5.isConnected()) {
    current_mode = MODE_MANUAL;
    output_vx = ps5_vx;
    output_vy = ps5_vy;
    output_wz = ps5_wz;
  }
  // Rule 2: ROS active (no PS5) → AUTO
  else if (millis() - last_ros_cmd_time < CMD_TIMEOUT_MS && last_ros_cmd_time > 0) {
    current_mode = MODE_AUTO;
    output_vx = ros_vx;
    output_vy = ros_vy;
    output_wz = ros_wz;
  }
  // Rule 3: No input → STOPPED
  else {
    current_mode = MODE_STOPPED;
    output_vx = output_vy = output_wz = 0;
  }

  // Log mode changes
  if (prev_mode != current_mode) {
    Serial.printf("\n>>> MODE CHANGE: %s → %s <<<\n\n",
                  mode_names[prev_mode], mode_names[current_mode]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  Stage 4: Arbitration Logic Test");
  Serial.println("========================================\n");

  // Initialize CAN and motors (same as Stage 3)
  canInit();
  delay(100);

  Serial.println("Enabling motors...");
  enableMotor(MOTOR_FL); sendSpeed(MOTOR_FL, 0.0f); delay(50);
  enableMotor(MOTOR_FR); sendSpeed(MOTOR_FR, 0.0f); delay(50);
  enableMotor(MOTOR_RL); sendSpeed(MOTOR_RL, 0.0f); delay(50);
  enableMotor(MOTOR_RR); sendSpeed(MOTOR_RR, 0.0f); delay(50);
  Serial.println("✓ Motors enabled\n");

  Serial.println("Motor warm-up (2s)...");
  unsigned long warmupStart = millis();
  while (millis() - warmupStart < 2000) {
    sendSpeed(MOTOR_FL, 0.0f);
    sendSpeed(MOTOR_FR, 0.0f);
    sendSpeed(MOTOR_RL, 0.0f);
    sendSpeed(MOTOR_RR, 0.0f);
    delay(100);
  }
  Serial.println("✓ Warm-up complete\n");

  // Initialize PS5
  PS5.begin(PS5_MAC);
  Serial.println("✓ PS5 initialized (waiting for connection)\n");

  // Initialize micro-ROS
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_arbitration", "", &support);

  rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                  &cmdVelCallback, ON_NEW_DATA);

  Serial.println("✓ Subscribed to /cmd_vel");
  Serial.println("✓ Arbitration ready\n");
}

void loop() {
  // Process ROS callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Read PS5 input
  readPS5Input();

  // Update control mode and output velocities
  updateControlMode();

  // Drive motors with arbitrated output
  mecanumDrive(output_vx, output_vy, output_wz, MAX_SPEED);

  // Diagnostic output every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf("[%s] vx=%.2f vy=%.2f wz=%.2f | PS5:%s ROS:%s\n",
                  mode_names[current_mode],
                  output_vx, output_vy, output_wz,
                  PS5.isConnected() ? "✓" : "✗",
                  (millis() - last_ros_cmd_time < CMD_TIMEOUT_MS && last_ros_cmd_time > 0) ? "✓" : "✗");
  }

  delay(100);  // 10Hz control loop
}
```

**Testing Procedure:**

1. **SAFETY: Elevate robot**

2. **Upload to ESP32**

3. **Start micro-ROS agent**

4. **Test Mode Transitions:**

   **Test A: ROS Control (PS5 disconnected)**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   - Serial should show: `>>> MODE CHANGE: STOPPED → AUTO <<<`
   - Robot responds to keyboard commands
   - Status shows: `[AUTO] ... PS5:✗ ROS:✓`

   **Test B: PS5 Override**
   - Press PS button on PS5 controller
   - Serial should show: `>>> MODE CHANGE: AUTO → MANUAL <<<`
   - Robot now responds to PS5, ignores ROS
   - Status shows: `[MANUAL] ... PS5:✓ ROS:✓`

   **Test C: PS5 Disconnect**
   - Turn off PS5 controller
   - Serial should show: `>>> MODE CHANGE: MANUAL → STOPPED <<<`
   - Then if ROS active: `>>> MODE CHANGE: STOPPED → AUTO <<<`

   **Test D: Timeout**
   - Stop ROS commands (press 'k')
   - Wait 500ms
   - Serial should show: `>>> MODE CHANGE: AUTO → STOPPED <<<`

**Success Criteria:**
- ✓ PS5 connected → MANUAL_MODE (ROS ignored)
- ✓ PS5 disconnected + ROS active → AUTO_MODE
- ✓ PS5 instant override works
- ✓ Timeout → STOPPED mode
- ✓ Mode transitions logged correctly

---

### Step 6: Stage 5 Implementation - Full Integration

**Goal:** Production-ready code combining all features.

**Create:** `mecanum_ros.ino`

**Key Enhancements over Stage 4:**
- Status publishing to `/robot/status` topic
- Enhanced error handling
- Optimized performance
- Better diagnostic output
- Ready for Phase 4 (odometry integration)

**Implementation Notes:**

This file is essentially Stage 4 code with these additions:

1. **Status Publisher:**
   ```cpp
   rcl_publisher_t status_pub;
   std_msgs__msg__String status_msg;

   // In setup():
   rclc_publisher_init_default(
     &status_pub, &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
     "robot/status"
   );

   // In loop() every 1 second:
   sprintf(status_msg.data.data, "Mode:%s PS5:%d ROS:%d",
           mode_names[current_mode],
           PS5.isConnected() ? 1 : 0,
           ros_active ? 1 : 0);
   rcl_publish(&status_pub, &status_msg, NULL);
   ```

2. **Error Recovery:**
   - Detect CAN TX failures
   - Attempt motor re-enable if needed
   - Log errors to serial and ROS

3. **Performance Optimization:**
   - Reduce unnecessary serial prints
   - Optimize executor spin timing

**Testing:**
- Same as Stage 4, plus verify `/robot/status` topic publishes correctly
- Monitor with: `ros2 topic echo /robot/status`

---

## File Structure

After completing Phase 3, your project will have:

```
/Users/anthony/Mecanum/
├── mecanum_ps5.ino                    # Phase 2 (PS5 only)
├── mecanum_ros.ino                    # Phase 3 FINAL (PS5 + ROS)
├── motor_config.h                     # Motor configuration
├── test_microros_connectivity.ino     # Stage 1 test
├── test_microros_cmdvel.ino          # Stage 2 test
├── test_microros_motors.ino          # Stage 3 test
├── test_arbitration.ino              # Stage 4 test
├── STATUS.md                          # Project status (update after Phase 3)
├── PROJECT.md                         # Project plan
└── docs/
    └── phase3/
        ├── stage1_results.md          # Stage 1 test results
        ├── stage2_results.md          # Stage 2 test results
        ├── stage3_results.md          # Stage 3 test results
        └── stage4_results.md          # Stage 4 test results
```

**Orange Pi:**
```
~/mecanum_ws/
└── start_microros_agent.sh           # Agent launch script
```

---

## Verification Steps

After completing all stages, verify the full system:

### 1. Basic Connectivity
```bash
# Terminal 1: Start agent
~/mecanum_ws/start_microros_agent.sh

# Terminal 2: Check topics
ros2 topic list
# Should see: /cmd_vel, /robot/status, /parameter_events, /rosout
```

### 2. ROS Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Verify robot responds to all motion types
```

### 3. PS5 Override
- Press PS button on controller
- Verify robot switches to PS5 control
- Verify ROS commands are ignored
- Check serial output shows mode change

### 4. Timeout Safety
- Stop all commands
- Verify robot stops after 500ms
- Check serial shows STOPPED mode

### 5. Status Publishing
```bash
ros2 topic echo /robot/status
# Should show current mode and connection status
```

---

## Troubleshooting Guide

### micro-ROS Connection Issues

**Problem:** Agent can't connect to ESP32

**Solutions:**
- Check USB device path: `ls -l /dev/ttyUSB* /dev/ttyACM*`
- Verify baudrate matches (115200)
- Check user permissions: `groups` (should include dialout)
- Try different USB port
- Restart ESP32 after uploading code

**Problem:** Topics don't appear in ROS

**Solutions:**
- Verify agent is running without errors
- Check ESP32 serial output for connection status
- Restart both agent and ESP32
- Check micro-ROS library version (need 2.0.5+)

### Motor Control Issues

**Problem:** Motors don't respond to ROS commands

**Solutions:**
- Verify Stage 3 test works (ROS-only motor control)
- Check CAN bus termination (120Ω resistors)
- Verify motor enable sequence in serial output
- Check motor_config.h matches your hardware
- Test individual motors with test_motor_simple.ino

**Problem:** Only some motors respond

**Solutions:**
- Check CAN TX status in serial output
- Verify all motors enabled successfully in setup
- Check motor IDs in motor_config.h
- Test each motor individually

### Arbitration Issues

**Problem:** PS5 doesn't override ROS

**Solutions:**
- Verify PS5 is connected (check serial output)
- Check PS5 MAC address in code matches controller
- Verify arbitration logic in updateControlMode()
- Add debug prints to show which mode is active

**Problem:** Robot doesn't stop on timeout

**Solutions:**
- Verify CMD_TIMEOUT_MS is set correctly (500ms)
- Check timeout logic in loop()
- Add debug prints to show time since last command
- Verify mecanumDrive(0,0,0) is called when stopped

---

## Summary

This plan provides a complete, incremental approach to Phase 3 implementation:

**5 Test Stages:**
1. Basic connectivity (heartbeat publishing)
2. /cmd_vel subscription (parsing only)
3. ROS motor control (no PS5)
4. Arbitration logic (PS5 + ROS)
5. Full integration (production ready)

**Key Safety Features:**
- PS5 always overrides ROS (instant)
- 500ms timeout watchdog
- Motors stop on disconnect/timeout
- Sequential motor enable sequence preserved

**Testing Strategy:**
- Each stage has standalone test code
- Clear success criteria for each stage
- Incremental validation before moving forward
- Safety first (wheels elevated during testing)

**Next Steps After Phase 3:**
- Phase 4: Wheel odometry (ESP32 publishes /odom)
- Phase 5: IMU fusion (optional, for drift reduction)
- Phase 6: Nav2 integration (autonomous navigation)

**Critical Files:**
- `mecanum_ros.ino` - Final production code
- `motor_config.h` - Hardware configuration
- `start_microros_agent.sh` - Orange Pi agent launcher

**Estimated Timeline:**
- Stage 1: 30 minutes (setup + connectivity test)
- Stage 2: 20 minutes (subscription test)
- Stage 3: 45 minutes (motor control integration)
- Stage 4: 45 minutes (arbitration logic)
- Stage 5: 30 minutes (final integration)
- **Total: ~3 hours** (excluding troubleshooting)

---

## Ready to Implement

This plan is ready for execution. Follow the stages sequentially, validate each stage before proceeding, and document results in `docs/phase3/` directory.

**Start with:** Step 1 (Environment Setup) → Step 2 (Stage 1 Implementation)



