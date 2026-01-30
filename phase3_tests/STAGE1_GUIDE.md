# Phase 3 Testing - Stage 1: Basic Connectivity

## Goal
Verify ESP32 can connect to micro-ROS agent and publish heartbeat messages.

## Prerequisites

### ESP32 Setup
1. **Install micro-ROS Arduino Library:**
   - Open Arduino IDE
   - Sketch → Include Library → Manage Libraries
   - Search: "micro_ros_arduino"
   - Install: "micro_ros_arduino" by Pablo Garrido

2. **Configure Arduino IDE:**
   - Board: ESP32 Dev Module
   - Upload Speed: 921600
   - CPU Frequency: 240MHz
   - Flash Size: 4MB
   - Partition Scheme: Default 4MB with spiffs

### Orange Pi Setup
1. **Verify ROS 2 Humble is installed:**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 --version
   ```

2. **Install micro-ROS agent:**
   ```bash
   sudo apt update
   sudo apt install ros-humble-micro-ros-agent -y
   ```

3. **Configure USB permissions:**
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout and login for changes to take effect
   ```

4. **Find ESP32 device:**
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   # Note the device path (e.g., /dev/ttyUSB0)
   ```

## Testing Procedure

### Step 1: Upload to ESP32
1. Open `test_microros_connectivity.ino` in Arduino IDE
2. Select correct board and port
3. Upload the sketch
4. Open Serial Monitor (115200 baud)

**Expected Serial Output:**
```
========================================
  Stage 1: micro-ROS Connectivity Test
========================================

✓ micro-ROS initialized
✓ Publisher: /esp32/heartbeat

Publishing heartbeat...

✓ Published: Heartbeat 0
✓ Published: Heartbeat 1
✓ Published: Heartbeat 2
...
```

### Step 2: Start micro-ROS Agent on Orange Pi
```bash
# Replace /dev/ttyUSB0 with your device path
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

**Expected Agent Output:**
```
[timestamp] info | TermiosAgentLinux.cpp | init | running...
[timestamp] info | Root.cpp | create_client | create
```

### Step 3: Verify ROS Topics
Open a new terminal on Orange Pi:
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

**Expected Topics:**
```
/esp32/heartbeat
/parameter_events
/rosout
```

### Step 4: Echo Heartbeat Messages
```bash
ros2 topic echo /esp32/heartbeat
```

**Expected Output:**
```
data: 'Heartbeat 0'
---
data: 'Heartbeat 1'
---
data: 'Heartbeat 2'
---
```

## Success Criteria
- ✓ ESP32 LED blinks every second
- ✓ Serial monitor shows successful publishes
- ✓ `ros2 topic list` shows `/esp32/heartbeat`
- ✓ `ros2 topic echo` shows messages
- ✓ No connection errors

## Troubleshooting

### Agent can't connect to ESP32
- Check USB device path: `ls /dev/ttyUSB*`
- Verify baudrate is 115200 on both sides
- Check user is in dialout group: `groups`
- Try different USB port
- Restart ESP32 after uploading

### No topics appear in ROS
- Verify agent is running without errors
- Check ESP32 serial output for connection status
- Restart both agent and ESP32
- Check micro-ROS library version (need 2.0.5+)

### Publish fails
- Check micro-ROS library installation
- Verify Serial connection is stable
- Check ESP32 has enough memory
- Try reducing publish rate

## Results
Document your test results in `stage1_results.md`
