# Mecanum Robot Project - Status Report

**Date:** 2026-01-25
**Project:** ESP32-based Mecanum Wheel Robot with PS5 Controller
**Current Phase:** Phase 2 Complete ✅

---

## Project Overview

Building a mecanum wheel robot with:
- **Hardware:** ESP32, 4x Damiao 3519 BLDC motors via CAN bus
- **Control:** PS5 controller via Bluetooth
- **Future:** ROS 2 integration for autonomous capabilities

---

## Completed Phases

### ✅ Phase 0: Hardware Validation (Completed 2026-01-24)

**Goal:** Validate all hardware and determine motor configurations

**Accomplishments:**
- Tested all 4 motors individually using CAN bus
- Determined motor direction multipliers through systematic testing
- Fixed CAN bus termination issues (added 120Ω resistors)
- Documented motor IDs and wiring configuration

**Key Files:**
- `test_motor_simple.ino` - Single motor test (WORKS)
- `motor_config.h` - Motor IDs and direction multipliers
- `PHASE0_RESULTS.md` - Test results documentation

**Motor Configuration (from Phase 0 testing):**
```cpp
Motor FL (ID 1): Positive speed = Backward → Direction multiplier = -1
Motor FR (ID 2): Positive speed = Forward  → Direction multiplier = +1
Motor RL (ID 3): Positive speed = Backward → Direction multiplier = -1
Motor RR (ID 4): Positive speed = Forward  → Direction multiplier = +1
```

**Pattern:** Left motors need -1 multiplier, right motors need +1 (opposite polarity wiring)

**Critical Discovery - Motor Enable Sequence:**
- Motors require specific initialization sequence to work properly
- Enable command must be followed immediately by 0 speed command
- Delay of 50ms needed between enabling each motor
- 2-second warm-up period with 0 speed required before non-zero speeds
- Without proper sequence, motors timeout and don't respond

---

### ✅ Phase 1: Mecanum Drive Implementation (Completed 2026-01-25)

**Goal:** Implement mecanum kinematics and verify motion patterns

**Accomplishments:**
- Implemented mecanum inverse kinematics
- Applied direction multipliers from Phase 0
- Debugged motor enable sequence issues (only motor 2 was spinning initially)
- Systematic debugging revealed timing requirements
- Successfully validated all motion types: forward, backward, strafe left/right, rotate CW/CCW

**Key Files:**
- `mecanum_drive.ino` - Working mecanum drive with test sequence (WORKS)

**Critical Fixes Applied:**
1. **Sequential Motor Enabling:**
   ```cpp
   enableMotor(MOTOR_FL);
   sendSpeed(MOTOR_FL, 0.0f);
   delay(50);
   // Repeat for each motor
   ```

2. **2-Second Warm-up Period:**
   - Motors need 2 seconds of 0 speed commands before accepting non-zero speeds
   - `testStart` set at end of setup, not in loop

3. **Proper CAN Timing:**
   - 100ms timeout for CAN transmit: `twai_transmit(&msg, pdMS_TO_TICKS(100))`
   - 100ms loop delay for consistent command intervals

**Mecanum Kinematics:**
```cpp
FL = vx - vy - w
FR = vx + vy + w
RL = vx + vy - w
RR = vx - vy + w

// Then apply direction multipliers
fl_speed *= MOTOR_FL_DIR;  // -1
fr_speed *= MOTOR_FR_DIR;  // +1
rl_speed *= MOTOR_RL_DIR;  // -1
rr_speed *= MOTOR_RR_DIR;  // +1
```

**Debugging Process:**
- Used systematic debugging skill to isolate root cause
- Created diagnostic versions with CAN TX status output
- Tested motors individually vs simultaneously
- Discovered motors work individually but not together initially
- Root cause: Improper enable sequence and timing

---

### ✅ Phase 2: PS5 Controller Integration (Completed 2026-01-25)

**Goal:** Add PS5 controller for manual control

**Accomplishments:**
- Integrated PS5Controller library (https://github.com/rodneybakiskan/ps5-esp32)
- Implemented joystick control mapping
- Added speed boost feature (R2 button for 2x speed)
- Added safety features (stops if controller disconnects)
- Added deadzone filtering and speed normalization
- Fixed forward/backward inversion issue
- Debugged and resolved FL motor not moving issue

**Key Files:**
- `mecanum_ps5.ino` - PS5 controller integrated mecanum drive (WORKS)

**Control Mapping:**
- Left Stick Y: Forward/Backward
- Left Stick X: Strafe Left/Right
- Right Stick X: Rotation
- R2 Button: Speed Boost (2x speed)

**Parameters:**
- MAX_SPEED: 6.0 rad/s (normal)
- BOOST_MULTIPLIER: 2.0x (12.0 rad/s when R2 held)
- DEADZONE: 0.05 (5% joystick deadzone)

**Features:**
- Speed normalization (prevents wheel saturation)
- Deadzone filtering (prevents stick drift)
- Safety: motors stop if controller disconnects
- Diagnostic output every 500ms showing all motor speeds

**Issues Fixed:**
1. Forward/backward inversion - removed negative sign from vx input
2. FL motor not moving - diagnostic output confirmed all motors receiving correct speeds

---

## Current State

### Working Code Files

1. **`mecanum_ps5.ino`** ✅ CURRENT - Full PS5 controller integration
   - All 4 motors working
   - All motion types validated (forward, strafe, rotate)
   - Speed boost functional
   - Ready for daily use

2. **`mecanum_drive.ino`** ✅ TESTED - Automated test sequence
   - Useful for testing without controller
   - Validates all motion patterns

3. **`test_motor_simple.ino`** ✅ REFERENCE - Single motor test
   - Change MOTOR_ID to test individual motors
   - Useful for debugging individual motor issues

### Configuration Files

1. **`motor_config.h`** - Motor configuration
   ```cpp
   #define MOTOR_FL 1
   #define MOTOR_FR 2
   #define MOTOR_RL 3
   #define MOTOR_RR 4

   #define SPEED_FRAME_ID(id) (0x200 + (id))

   #define MOTOR_FL_DIR -1
   #define MOTOR_FR_DIR  1
   #define MOTOR_RL_DIR -1
   #define MOTOR_RR_DIR  1

   #define CAN_TX_GPIO 5
   #define CAN_RX_GPIO 4
   #define CAN_BAUDRATE 1000000
   ```

### Reference Files (Historical)

- `Reference/Esp32_mecanum_35199.ino` - Original code with CAN issues
- `Reference/worked_single_motor.ino` - Phase 0 working example

---

## Critical Technical Knowledge

### Motor Enable Sequence (ESSENTIAL)

**WRONG (doesn't work):**
```cpp
enableMotor(MOTOR_FL);
enableMotor(MOTOR_FR);
enableMotor(MOTOR_RL);
enableMotor(MOTOR_RR);
delay(100);
```

**CORRECT (works):**
```cpp
enableMotor(MOTOR_FL);
sendSpeed(MOTOR_FL, 0.0f);
delay(50);

enableMotor(MOTOR_FR);
sendSpeed(MOTOR_FR, 0.0f);
delay(50);

enableMotor(MOTOR_RL);
sendSpeed(MOTOR_RL, 0.0f);
delay(50);

enableMotor(MOTOR_RR);
sendSpeed(MOTOR_RR, 0.0f);
delay(50);

testStart = millis();  // Set timing before loop
```

**Then in loop:**
- First 2 seconds: send 0 speed to all motors (warm-up)
- After 2 seconds: can send non-zero speeds

### CAN Communication Requirements

1. **Timing:**
   - CAN transmit timeout: 100ms (`pdMS_TO_TICKS(100)`)
   - Loop delay: 100ms (10Hz update rate)
   - Enable delay between motors: 50ms

2. **Frame Format:**
   - Enable: 8 bytes (0xFF x7, 0xFC)
   - Speed: 4 bytes (float, little-endian, rad/s)
   - Frame ID: 0x200 + motor_id

3. **Hardware:**
   - CAN TX: GPIO 5
   - CAN RX: GPIO 4
   - Baudrate: 1 Mbps
   - Termination: 120Ω resistors at both ends

### Direction Multipliers Explanation

Left and right motors have opposite wiring polarity:
- Left motors (FL, RL): positive speed → backward motion
- Right motors (FR, RR): positive speed → forward motion

Direction multipliers compensate:
- Multiply left motor speeds by -1
- Multiply right motor speeds by +1

Result: positive speed values make robot move forward as expected.

**Important:** Apply multipliers AFTER kinematics calculation to preserve differential speeds for rotation/strafe.

---

## Known Issues & Limitations

### Current Limitations

1. **No autonomous capabilities** - Only manual PS5 control (Phase 3 will add ROS 2)
2. **Fixed speed limits** - MAX_SPEED hardcoded to 6.0 rad/s
3. **No odometry** - No position/velocity feedback from motors
4. **No obstacle avoidance** - Manual control only

### Hardware Constraints

1. **Motor polarity** - Left/right motors have opposite polarity (compensated in software)
2. **CAN bus** - Requires proper termination (120Ω resistors)
3. **Enable sequence** - Motors timeout if not receiving commands regularly

### None - All Issues Resolved

All major bugs from Phases 0-2 have been fixed:
- ✅ CAN termination issue (Phase 0)
- ✅ Motor enable sequence (Phase 1)
- ✅ Only motor 2 spinning (Phase 1)
- ✅ Forward/backward inversion (Phase 2)
- ✅ FL motor not moving (Phase 2)

---

## Next Steps (Phase 3)

### ROS 2 Integration (Not Started)

**Goals:**
1. Create ROS 2 node for ESP32
2. Implement cmd_vel subscriber for velocity commands
3. Publish odometry data (if motor feedback available)
4. Maintain PS5 controller as backup/override

**Architecture:**
- ESP32: Motor control, CAN communication, PS5 failsafe
- ROS 2: High-level planning, navigation, autonomy
- Communication: Serial or WiFi bridge

**Dependencies:**
- micro-ROS for ESP32
- ROS 2 Humble (or newer)
- Navigation stack (optional)

---

## Development Setup

### Required Hardware
- ESP32 development board
- 4x Damiao 3519 BLDC motors
- CAN transceiver module
- 120Ω termination resistors (2x)
- PS5 controller (paired with ESP32)
- Power supply for motors

### Required Libraries
- ESP32 Arduino Core
- PS5Controller library: https://github.com/rodneybakiskan/ps5-esp32
- ESP32 TWAI (CAN) driver (built-in)

### Development Tools
- Arduino IDE or PlatformIO
- Serial Monitor (115200 baud)
- Git (for version control)

---

## Testing Checklist

### Phase 0 Test (Individual Motors)
- [ ] Upload test_motor_simple.ino
- [ ] Change MOTOR_ID to 1, 2, 3, 4
- [ ] Verify each motor spins in both directions
- [ ] Record direction behavior
- [ ] Update motor_config.h with direction multipliers

### Phase 1 Test (Mecanum Drive)
- [ ] Upload mecanum_drive.ino
- [ ] Verify all 4 motors spin during warm-up
- [ ] Verify forward motion (all wheels same direction)
- [ ] Verify strafe motion (diagonal wheel pairs)
- [ ] Verify rotation (left vs right wheels opposite)

### Phase 2 Test (PS5 Controller)
- [ ] Install PS5Controller library
- [ ] Pair PS5 controller with ESP32
- [ ] Upload mecanum_ps5.ino
- [ ] Press PS button to connect
- [ ] Test forward/backward (left stick Y)
- [ ] Test strafe left/right (left stick X)
- [ ] Test rotation (right stick X)
- [ ] Test speed boost (hold R2)
- [ ] Test disconnect safety (turn off controller)

---

## File Structure

```
Mecanum/
├── mecanum_ps5.ino              ✅ MAIN - PS5 controller version
├── mecanum_drive.ino            ✅ Test sequence version
├── motor_config.h               ✅ Motor configuration
├── test_motor_simple.ino        ✅ Single motor test
├── PHASE0_RESULTS.md            📄 Phase 0 test results
├── PROJECT.md                   📄 Original project plan
├── Reference/
│   ├── Esp32_mecanum_35199.ino  📄 Original code (had issues)
│   └── worked_single_motor.ino  📄 Phase 0 reference
└── docs/
    └── plans/
        └── (design documents)

✅ = Working, tested code
📄 = Documentation/reference
```

---

## Key Learnings & Best Practices

### 1. Systematic Debugging
- Used systematic debugging skill to isolate motor enable issue
- Added diagnostic output to observe actual vs expected behavior
- Tested incrementally (1 motor → 4 motors)
- Compared working code (test_motor_simple.ino) with non-working code
- Root cause analysis before attempting fixes

### 2. Hardware-Software Integration
- Hardware timing requirements drive software design
- Motors have specific initialization sequences
- CAN bus needs proper termination for reliability
- Testing individual components before integration

### 3. Motor Control Patterns
- Sequential enabling more reliable than parallel
- Immediate feedback (0 speed) after enable prevents timeout
- Warm-up period prevents motor confusion
- Consistent command intervals (100ms) critical

### 4. Code Organization
- Separate configuration (motor_config.h) from logic
- Keep working reference code (test_motor_simple.ino)
- Maintain test versions alongside production code
- Document critical timing requirements in comments

---

## Contact & Handoff Notes

### For Next Developer

**Start Here:**
1. Read this STATUS.md file completely
2. Review PROJECT.md for overall architecture
3. Upload mecanum_ps5.ino to test current functionality
4. Check motor_config.h matches your hardware setup

**Critical Files:**
- `mecanum_ps5.ino` - Current working code
- `motor_config.h` - Hardware configuration
- This file (STATUS.md) - Everything you need to know

**If Something Breaks:**
1. Test individual motors with test_motor_simple.ino
2. Check CAN bus termination (120Ω resistors)
3. Verify motor enable sequence (see "Critical Technical Knowledge")
4. Review systematic debugging process used in Phase 1

**Common Pitfalls:**
- Don't skip motor enable sequence - it's critical
- Don't reduce loop delay below 100ms
- Don't remove warm-up period
- Don't enable motors in parallel without delays
- Don't forget direction multipliers

---

## Version History

- **v1.0** (2026-01-25): Phase 2 complete - PS5 controller integration working
- **v0.2** (2026-01-25): Phase 1 complete - Mecanum drive working
- **v0.1** (2026-01-24): Phase 0 complete - Individual motors tested

---

**Status:** Ready for Phase 3 (ROS 2 Integration)
**Last Updated:** 2026-01-25
**Next Milestone:** ROS 2 node implementation
