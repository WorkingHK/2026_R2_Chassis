/**
 * Motor Configuration for Mecanum Robot
 * Damiao 3519 BLDC Motors via CAN
 *
 * This file contains motor CAN IDs and direction multipliers.
 * Direction multipliers should be determined during Phase 0 testing.
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ============================================================================
// MOTOR CAN IDs
// ============================================================================
#define MOTOR_FL 1   // Front Left
#define MOTOR_FR 2   // Front Right
#define MOTOR_RL 3   // Rear Left
#define MOTOR_RR 4   // Rear Right

#define SPEED_FRAME_ID(id) (0x200 + (id))

// ============================================================================
// DIRECTION MULTIPLIERS
// ============================================================================
// Determined from Phase 0 testing (2026-01-24)
// Set to 1 or -1 based on which direction is "forward" for each motor
//
// Test results:
// - Motor 1 (FL): Positive speed = Backward → multiplier = -1
// - Motor 2 (FR): Positive speed = Forward → multiplier = +1
// - Motor 3 (RL): Positive speed = Backward → multiplier = -1
// - Motor 4 (RR): Positive speed = Forward → multiplier = +1
//
// Pattern: Left side motors need -1, right side motors need +1

#define MOTOR_FL_DIR -1   // Tested: Positive = Backward (needs inversion)
#define MOTOR_FR_DIR 1    // Tested: Positive = Forward
#define MOTOR_RL_DIR -1   // Tested: Positive = Backward (needs inversion)
#define MOTOR_RR_DIR 1    // Tested: Positive = Forward

// ============================================================================
// MOTOR LIMITS
// ============================================================================
#define MAX_MOTOR_SPEED 10.0f  // rad/s - adjust based on your requirements
#define TEST_SPEED 5.0f        // rad/s - safe speed for testing

// ============================================================================
// CAN CONFIGURATION
// ============================================================================
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4
#define CAN_BAUDRATE 1000000   // 1 Mbps
#define CAN_TX_TIMEOUT_MS 10   // Timeout for CAN transmit

// ============================================================================
// MECANUM WHEEL CONFIGURATION
// ============================================================================
// Standard mecanum wheel pattern:
//   Front
//   FL ╱╲ FR    (FL and RR rollers angle forward-right)
//   RL ╲╱ RR    (FR and RL rollers angle forward-left)
//
// For forward motion (vx > 0): all wheels spin same direction
// For strafe right (vy > 0): FL,RR forward; FR,RL backward
// For rotate CW (wz > 0): FL,RL forward; FR,RR backward

#endif // MOTOR_CONFIG_H
