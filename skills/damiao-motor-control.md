---
name: damiao-motor-control
description: Knowledge base for controlling Damiao 3519 BLDC motors via CAN bus with ESP32. Includes common mistakes, correct patterns, and troubleshooting guide.
---

# Damiao Motor Control Guide

## Overview

This skill contains hard-won knowledge about controlling Damiao 3519 BLDC motors via CAN bus using ESP32 TWAI driver. It documents common mistakes, correct usage patterns, and troubleshooting steps.

**Use this skill when:**
- Setting up new Damiao motors with ESP32
- Debugging motor initialization issues
- Motors not responding to commands
- Only some motors working in multi-motor setup
- Planning CAN bus motor control architecture

---

## Motor Specifications

**Damiao 3519 BLDC Motors:**
- Communication: CAN bus (1 Mbps)
- Control Mode: Speed control (rad/s)
- Command Frame ID: `0x200 + motor_id` (e.g., motor 1 = 0x201)
- Enable Command: 8 bytes (0xFF × 7, 0xFC)
- Speed Command: 4 bytes (float, little-endian)

**Tested Speed Range:**
- Safe testing: 3.0-5.0 rad/s
- Normal operation: 6.0 rad/s
- Boost mode: 12.0 rad/s (2x)
- Maximum: Unknown (not tested beyond 12.0)

---

## Critical Requirements

### 1. CAN Bus Hardware

**Required:**
- CAN transceiver module (e.g., TJA1050, MCP2551)
- 120Ω termination resistors at BOTH ends of CAN bus
- Twisted pair wiring for CAN-H and CAN-L

**Common Mistake #1: Missing Termination**
```
❌ WRONG: No termination resistors
Result: CAN TX Error Counter = 128, Bus Error count > 1,000,000
Symptom: ESP_ERR_TIMEOUT on every transmission (88-97% failure rate)

✅ CORRECT: 120Ω resistor at each end
Result: All transmissions succeed, error counters stay at 0
```

**Diagnostic:**
```cpp
twai_status_info_t status;
twai_get_status_info(&status);
Serial.printf("TX Error: %d, Bus Error: %d\n",
              status.tx_error_counter, status.bus_error_count);

// Healthy: both should be 0 or very low
// Problem: TX error = 128, bus error > 1000
```

### 2. Motor Initialization Sequence

**THIS IS THE MOST CRITICAL PART - GET IT WRONG AND MOTORS WON'T WORK**

**Common Mistake #2: Parallel/Batch Enabling**
```cpp
❌ WRONG: Enable all motors quickly without delays
void setup() {
  canInit();
  delay(100);

  enableMotor(MOTOR_1);
  enableMotor(MOTOR_2);
  enableMotor(MOTOR_3);
  enableMotor(MOTOR_4);

  delay(100);
  // Start sending speed commands
}

Result: Only one motor (often motor 2) works, others timeout/ignore commands
```

**Common Mistake #3: No Initial Speed Command**
```cpp
❌ WRONG: Enable without immediate 0 speed
void setup() {
  canInit();
  delay(100);

  enableMotor(MOTOR_1);
  delay(100);  // Gap with no speed command

  // Motor times out during this gap
}

Result: Motor disables itself, doesn't respond to later commands
```

**Common Mistake #4: No Warm-up Period**
```cpp
❌ WRONG: Send non-zero speed immediately
void setup() {
  enableMotor(MOTOR_1);
  sendSpeed(MOTOR_1, 0.0f);
}

void loop() {
  sendSpeed(MOTOR_1, 5.0f);  // Immediately send non-zero speed
  delay(100);
}

Result: Motor doesn't respond or behaves erratically
```

**✅ CORRECT INITIALIZATION SEQUENCE:**

```cpp
unsigned long testStart = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize CAN bus
  canInit();
  delay(100);

  // Enable motors ONE AT A TIME with immediate speed command
  enableMotor(MOTOR_FL);
  sendSpeed(MOTOR_FL, 0.0f);  // Immediate 0 speed to keep motor alive
  delay(50);                   // Allow motor to process

  enableMotor(MOTOR_FR);
  sendSpeed(MOTOR_FR, 0.0f);
  delay(50);

  enableMotor(MOTOR_RL);
  sendSpeed(MOTOR_RL, 0.0f);
  delay(50);

  enableMotor(MOTOR_RR);
  sendSpeed(MOTOR_RR, 0.0f);
  delay(50);

  testStart = millis();  // Set timing BEFORE loop starts
}

void loop() {
  unsigned long elapsed = millis() - testStart;
  float speed = 0.0f;

  // CRITICAL: 2-second warm-up with 0 speed
  if (elapsed < 2000) {
    speed = 0.0f;
  } else {
    speed = 5.0f;  // Now safe to send non-zero speed
  }

  sendSpeed(MOTOR_FL, speed);
  sendSpeed(MOTOR_FR, speed);
  sendSpeed(MOTOR_RL, speed);
  sendSpeed(MOTOR_RR, speed);

  delay(100);  // 100ms loop timing is critical
}
```

**Why This Works:**
1. Sequential enabling prevents CAN bus congestion
2. Immediate 0 speed prevents motor timeout
3. 50ms delay allows motor to process enable command
4. Warm-up period lets motors stabilize before load
5. testStart set in setup ensures correct timing from first loop

---

## CAN Communication Patterns

### Timing Requirements

**Common Mistake #5: Wrong Timeout Values**
```cpp
❌ WRONG: Too short timeout
twai_transmit(&msg, pdMS_TO_TICKS(10));  // 10ms timeout

Result: Works for single motor, fails for multiple motors
```

**Common Mistake #6: Wrong Loop Timing**
```cpp
❌ WRONG: Too fast loop rate
void loop() {
  sendSpeed(motor, speed);
  delay(10);  // 100Hz is too fast
}

Result: Motors don't respond reliably
```

**✅ CORRECT TIMING:**
```cpp
// CAN transmit timeout: 100ms
esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));

// Loop delay: 100ms (10Hz update rate)
void loop() {
  // Send commands to all motors
  sendSpeed(MOTOR_1, speed1);
  sendSpeed(MOTOR_2, speed2);
  sendSpeed(MOTOR_3, speed3);
  sendSpeed(MOTOR_4, speed4);

  delay(100);  // Critical: 100ms delay
}
```

### Enable Command Format

```cpp
void enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = 0x200 + id;  // Frame ID
  msg.extd = 0;                  // Standard frame (not extended)
  msg.data_length_code = 8;      // 8 bytes

  // Enable command: 0xFF × 7, then 0xFC
  for (int i = 0; i < 7; i++) {
    msg.data[i] = 0xFF;
  }
  msg.data[7] = 0xFC;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}
```

### Speed Command Format

```cpp
void sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = 0x200 + id;  // Frame ID
  msg.extd = 0;                  // Standard frame
  msg.data_length_code = 4;      // 4 bytes for float

  // Copy float as little-endian bytes
  memcpy(msg.data, &speed, 4);

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}
```

---

## Multi-Motor Control

### Motor Direction Compensation

**Reality: Motors may have opposite polarity**

When testing individual motors with positive speed:
- Some motors spin forward
- Some motors spin backward

**Common Mistake #7: Ignoring Direction Differences**
```cpp
❌ WRONG: Send same speed to all motors for forward motion
sendSpeed(MOTOR_FL, 5.0);
sendSpeed(MOTOR_FR, 5.0);
sendSpeed(MOTOR_RL, 5.0);
sendSpeed(MOTOR_RR, 5.0);

Result: Robot doesn't go straight (some wheels backward)
```

**✅ CORRECT: Test and Apply Direction Multipliers**

**Step 1: Test Each Motor Individually**
```cpp
// Upload test_motor_simple.ino
#define MOTOR_ID 1  // Change to 1, 2, 3, 4

void loop() {
  float speed = 5.0f;  // Positive speed
  sendSpeed(MOTOR_ID, speed);
  // Observe: Does motor spin forward or backward?
}

// Record results:
// Motor 1 (FL): Positive = Backward → multiplier = -1
// Motor 2 (FR): Positive = Forward  → multiplier = +1
// Motor 3 (RL): Positive = Backward → multiplier = -1
// Motor 4 (RR): Positive = Forward  → multiplier = +1
```

**Step 2: Apply Multipliers in Code**
```cpp
#define MOTOR_FL_DIR -1
#define MOTOR_FR_DIR  1
#define MOTOR_RL_DIR -1
#define MOTOR_RR_DIR  1

void driveForward(float speed) {
  sendSpeed(MOTOR_FL, speed * MOTOR_FL_DIR);  // 5.0 * -1 = -5.0
  sendSpeed(MOTOR_FR, speed * MOTOR_FR_DIR);  // 5.0 * +1 =  5.0
  sendSpeed(MOTOR_RL, speed * MOTOR_RL_DIR);  // 5.0 * -1 = -5.0
  sendSpeed(MOTOR_RR, speed * MOTOR_RR_DIR);  // 5.0 * +1 =  5.0
}
```

**Important:** For mecanum kinematics, apply direction multipliers AFTER kinematics calculation:
```cpp
// Calculate raw speeds
float fl = vx - vy - w;
float fr = vx + vy + w;
float rl = vx + vy - w;
float rr = vx - vy + w;

// THEN apply direction multipliers
fl *= MOTOR_FL_DIR;
fr *= MOTOR_FR_DIR;
rl *= MOTOR_RL_DIR;
rr *= MOTOR_RR_DIR;
```

---

## Troubleshooting Guide

### Problem: Motors Don't Respond At All

**Checklist:**
1. ✓ CAN termination resistors installed?
2. ✓ CAN wiring correct (CAN-H to CAN-H, CAN-L to CAN-L)?
3. ✓ Motors powered on?
4. ✓ Using correct GPIO pins (TX=5, RX=4)?
5. ✓ Correct CAN baudrate (1 Mbps)?

**Diagnostic Code:**
```cpp
void checkCAN() {
  twai_status_info_t status;
  twai_get_status_info(&status);

  Serial.printf("State: %d (2=running)\n", status.state);
  Serial.printf("TX Error: %d\n", status.tx_error_counter);
  Serial.printf("RX Error: %d\n", status.rx_error_counter);
  Serial.printf("Bus Err: %d\n", status.bus_error_count);
  Serial.printf("TX Pending: %d\n", status.msgs_to_tx);

  // Healthy: state=2, errors=0
  // Problem: TX error=128, bus errors high → check termination
}
```

### Problem: Only One Motor Works (Others Don't)

**This was our exact problem in Phase 1!**

**Root Cause:** Incorrect enable sequence

**Solution:**
1. Enable motors ONE AT A TIME
2. Send 0 speed immediately after each enable
3. Add 50ms delay between each motor
4. Include 2-second warm-up period
5. See "CORRECT INITIALIZATION SEQUENCE" above

**Quick Test:**
Upload test_motor_simple.ino and test each motor individually:
- Change MOTOR_ID to 1, 2, 3, 4
- If individual motors work but not together → enable sequence issue
- If individual motors don't work → check wiring/power for that motor

### Problem: Motors Timeout After a While

**Cause:** Not sending commands frequently enough

**Solution:**
- Send commands every 100ms (10Hz minimum)
- Don't have long delays or blocking code in loop
- If controller disconnects, send 0 speed commands to keep motors alive

```cpp
void loop() {
  if (!PS5.isConnected()) {
    // Safety: still send commands even if controller disconnected
    sendSpeed(MOTOR_FL, 0.0f);
    sendSpeed(MOTOR_FR, 0.0f);
    sendSpeed(MOTOR_RL, 0.0f);
    sendSpeed(MOTOR_RR, 0.0f);
  } else {
    // Normal operation
  }

  delay(100);  // Consistent timing
}
```

### Problem: Erratic Motor Behavior

**Possible Causes:**
1. No warm-up period → Add 2-second 0 speed phase
2. Timing inconsistent → Ensure 100ms loop delay
3. Speed values too high → Stay under 12 rad/s
4. Direction multipliers wrong → Re-test individual motors

### Problem: CAN TX Failures (ESP_ERR_TIMEOUT)

**Diagnostic:**
```cpp
esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
if (result != ESP_OK) {
  Serial.printf("TX Failed: %d\n", result);
  // ESP_ERR_TIMEOUT = motor not ACKing (check termination)
  // ESP_ERR_INVALID_STATE = CAN not started
}
```

**Solutions:**
- ESP_ERR_TIMEOUT → Check termination resistors (120Ω)
- High failure rate → Add/fix termination
- Occasional failures → Increase timeout to 100ms

---

## Complete Working Example

```cpp
#include <driver/twai.h>

// Motor configuration
#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_RL 3
#define MOTOR_RR 4

#define MOTOR_FL_DIR -1
#define MOTOR_FR_DIR  1
#define MOTOR_RL_DIR -1
#define MOTOR_RR_DIR  1

#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

unsigned long testStart = 0;

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

bool enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = 0x200 + id;
  msg.extd = 0;
  msg.data_length_code = 8;

  for (int i = 0; i < 7; i++) msg.data[i] = 0xFF;
  msg.data[7] = 0xFC;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

bool sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = 0x200 + id;
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4);
  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Initializing...");
  canInit();
  delay(100);

  // CORRECT: Sequential enable with immediate 0 speed
  Serial.println("Enabling motors...");

  enableMotor(MOTOR_FL);
  sendSpeed(MOTOR_FL, 0.0f);
  delay(50);
  Serial.println("FL: OK");

  enableMotor(MOTOR_FR);
  sendSpeed(MOTOR_FR, 0.0f);
  delay(50);
  Serial.println("FR: OK");

  enableMotor(MOTOR_RL);
  sendSpeed(MOTOR_RL, 0.0f);
  delay(50);
  Serial.println("RL: OK");

  enableMotor(MOTOR_RR);
  sendSpeed(MOTOR_RR, 0.0f);
  delay(50);
  Serial.println("RR: OK");

  Serial.println("Starting in 2 seconds...");
  testStart = millis();
}

void loop() {
  unsigned long elapsed = millis() - testStart;
  float speed = 0.0f;

  // 2-second warm-up period
  if (elapsed < 2000) {
    speed = 0.0f;
  } else if (elapsed < 5000) {
    speed = 3.0f;  // Forward
  } else {
    speed = 0.0f;  // Stop
  }

  // Apply direction multipliers
  sendSpeed(MOTOR_FL, speed * MOTOR_FL_DIR);
  sendSpeed(MOTOR_FR, speed * MOTOR_FR_DIR);
  sendSpeed(MOTOR_RL, speed * MOTOR_RL_DIR);
  sendSpeed(MOTOR_RR, speed * MOTOR_RR_DIR);

  delay(100);  // 100ms timing
}
```

---

## Summary: Golden Rules

1. **Always add 120Ω termination resistors** at both ends of CAN bus
2. **Enable motors sequentially** with 50ms delays, not in parallel
3. **Send 0 speed immediately** after each enable command
4. **Include 2-second warm-up** with 0 speed before non-zero speeds
5. **Use 100ms timeouts** for CAN transmit
6. **Use 100ms loop delay** for consistent command timing
7. **Test individual motors** to determine direction multipliers
8. **Apply direction multipliers** after kinematics calculations
9. **Send commands continuously** (every 100ms) to prevent timeout
10. **Set timing variables in setup**, not in loop

**Follow these rules and your Damiao motors will work reliably!**

---

## References

- Tested with: Damiao 3519 BLDC motors
- ESP32 TWAI driver documentation
- CAN bus specification (ISO 11898)
- Project: Mecanum wheel robot (2026-01-24 to 2026-01-25)
- Phases completed: Phase 0 (hardware validation), Phase 1 (mecanum drive), Phase 2 (PS5 control)
