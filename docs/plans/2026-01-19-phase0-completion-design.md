# Phase 0 Completion Design
**Date:** 2026-01-19
**Status:** Ready for Implementation

## Goal
Systematically validate all 4 Damiao 3519 motors and document everything needed for Phase 1 (mecanum kinematics implementation).

## Current State
- Chassis built
- Basic motor testing completed (1-2 motors spinning via ESP32 TWAI)
- Need to validate all 4 motors, document directions, and ensure CAN reliability

## Overall Approach

### Test Progression Strategy
Build three test sketches, each building on the previous:

1. **Single Motor Test** (`test_single_motor.ino`)
   - Validate one motor at a time
   - Confirm CAN timing, speed control, and direction
   - Isolate issues to specific motors

2. **Four Motor Sequential Test** (`test_four_motors_sequential.ino`)
   - Test all 4 motors one after another in the same program
   - Confirm no CAN ID conflicts
   - Verify independent motor addressing

3. **Four Motor Simultaneous Test** (`test_four_motors_simultaneous.ino`)
   - Command all 4 motors at once with different speeds
   - Stress test CAN bus traffic handling
   - Validate timing under real control loop conditions

### Documentation Outputs
- `motor_config.h` - Motor CAN IDs, direction multipliers, physical positions
- `PHASE0_RESULTS.md` - Test results, observations, quirks discovered

### Hardware Setup Requirements
- Motors mounted in standard positions (FL, FR, RL, RR)
- Wheels off ground or robot on blocks for safe testing
- CAN bus properly terminated
- Adequate power supply for all 4 motors

## Test Code Design

### Single Motor Test
**Purpose:** Validate each motor individually

**Test Sequence:**
1. Enable motor
2. Ramp up to +5 rad/s (forward)
3. Hold for 2 seconds
4. Ramp down to 0
5. Ramp to -5 rad/s (reverse)
6. Hold for 2 seconds
7. Stop and disable

**Usage:** Manually change CAN ID in code to test each motor (0x01, 0x02, 0x03, 0x04). Observe and note which direction is "positive" for each physical position.

### Four Motor Sequential Test
**Purpose:** Confirm independent control and no CAN ID conflicts

**Test Sequence:**
1. Test motor 1 (full sequence)
2. Pause 1 second
3. Test motor 2 (full sequence)
4. Pause 1 second
5. Test motors 3 and 4

**Validation:** Each motor responds only to its own commands.

### Four Motor Simultaneous Test
**Purpose:** Validate CAN bus under realistic load

**Test Sequence:**
1. Enable all 4 motors
2. Command different speeds (e.g., FL=5, FR=3, RL=4, RR=2 rad/s)
3. Hold for 3 seconds
4. Reverse all speeds
5. Hold for 3 seconds
6. Stop all motors

**Critical Check:** No CAN TX failures during rapid multi-motor commands.

## Direction Mapping Methodology

### The Challenge
Mecanum wheels have specific orientation patterns, and motors can be wired/mounted differently. Need to determine direction multipliers so "forward" commands work correctly.

### Standard Mecanum Wheel Pattern
```
Front
FL ╱╲ FR    (FL and RR rollers angle forward-right)
RL ╲╱ RR    (FR and RL rollers angle forward-left)
```

### Direction Discovery Process

1. **Mark Positive Direction**
   - Run single motor test on each motor
   - Note which way each wheel spins for positive speed commands
   - Mark physically with tape or in notes

2. **Apply Mecanum Theory**
   - For forward motion (vx > 0), all wheels spin the same direction
   - Use FL as reference direction
   - If FR, RL, or RR spin opposite to FL for forward motion, they need direction multiplier of -1

3. **Strafe Validation Test**
   - For pure strafe right (vy > 0):
     - FL and RR should spin forward
     - FR and RL should spin backward
   - Validates wheel orientation matches standard pattern

### Output Format
Create `motor_config.h`:
```cpp
// Motor CAN IDs
#define MOTOR_FL_ID 0x01
#define MOTOR_FR_ID 0x02
#define MOTOR_RL_ID 0x03
#define MOTOR_RR_ID 0x04

// Direction multipliers (1 or -1)
#define MOTOR_FL_DIR 1
#define MOTOR_FR_DIR 1    // Adjust based on testing
#define MOTOR_RL_DIR 1    // Adjust based on testing
#define MOTOR_RR_DIR 1    // Adjust based on testing
```

## CAN Reliability Testing

### What We're Testing
- CAN TX buffer overflows (commands sent too fast)
- CAN RX handling (motor feedback if available)
- Bus-off conditions (excessive errors)
- Timing consistency at 1 Mbps

### Reliability Test Approach

1. **TX Failure Detection**
   - Check return value of every CAN transmit
   - Count and log failures
   ```cpp
   if (twai_transmit(&message, pdMS_TO_TICKS(10)) != ESP_OK) {
       tx_failures++;
   }
   ```

2. **Sustained Load Test**
   - Run all 4 motors at varying speeds for 60 seconds
   - Send commands at 50Hz (every 20ms)
   - Simulates real control loop conditions
   - Log any TX failures or timing issues

3. **Error Recovery Test**
   - Intentionally disconnect one motor's CAN connection mid-test
   - Verify ESP32 doesn't crash
   - Confirm recovery when reconnected

4. **Timing Validation**
   - Use `micros()` to measure loop timing
   - Ensure control loop sends 4 motor commands within target period
   - Target: 20ms for 50Hz (±1ms jitter acceptable)

### Success Criteria
- ✅ Zero TX failures during 60-second sustained test
- ✅ Consistent loop timing (±1ms jitter acceptable)
- ✅ Graceful handling of disconnected motors
- ✅ No bus-off conditions

## Deliverables

1. **Test Sketches**
   - `test_single_motor.ino`
   - `test_four_motors_sequential.ino`
   - `test_four_motors_simultaneous.ino`

2. **Configuration Files**
   - `motor_config.h` - Motor IDs and direction multipliers

3. **Documentation**
   - `PHASE0_RESULTS.md` - Test results, observations, quirks

## Phase 0 Exit Criteria
- ✅ All motors spin reliably
- ✅ No CAN TX failures under sustained load
- ✅ Speed unit confirmed (rad/s)
- ✅ Direction multipliers documented
- ✅ Motor mapping table complete

## Next Steps After Phase 0
Once Phase 0 is complete and exit criteria met:
- Proceed to Phase 1: Mecanum Drive kinematics implementation
- Use motor_config.h in mecanum inverse kinematics function
- Implement `mecanum(vx, vy, wz)` function using validated motor configuration
