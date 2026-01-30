/**
 * Mecanum Drive - Phase 3: IMU Integration with Heading Correction
 *
 * Controls all 4 Damiao 3519 motors via CAN using PS5 controller
 * Integrates WT901C485 IMU for straight-line driving assistance
 *
 * Controls:
 *   Left Stick Y: Forward/Backward (with heading correction)
 *   Left Stick X: Strafe Left/Right
 *   Right Stick X: Rotation
 *   R2 Button: Speed Boost (2x speed)
 *   L1 Button: Lock/Reset heading for straight driving
 *
 * Features:
 *   - Automatic heading correction when driving straight
 *   - IMU-based drift compensation
 *   - Manual heading lock with L1 button
 *
 * Safety: Motors stop if controller disconnects
 */

#include <driver/twai.h>
#include <PS5Controller.h>
#include "motor_config.h"
#include "imu_wt901.h"

// ================= PARAMETERS =================
#define MAX_SPEED 6.0f        // rad/s - normal speed
#define BOOST_MULTIPLIER 2.0f // Speed boost multiplier
#define DEADZONE 0.05f        // Joystick deadzone

// Heading correction PID parameters
#define HEADING_KP 0.02f      // Proportional gain (tune this!)
#define HEADING_KI 0.0f       // Integral gain
#define HEADING_KD 0.005f     // Derivative gain
#define HEADING_DEADZONE 2.0f // Degrees - don't correct if error is small

// ================= GLOBAL OBJECTS =================
IMU_WT901 imu;

// Heading control state
float targetHeading = 0.0f;
bool headingLocked = false;
float headingErrorIntegral = 0.0f;
float lastHeadingError = 0.0f;
unsigned long lastHeadingUpdate = 0;

// ================= CAN INIT =================
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

// ================= ENABLE MOTOR =================
bool enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 8;

  for (int i = 0; i < 7; i++) msg.data[i] = 0xFF;
  msg.data[7] = 0xFC;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

// ================= SEND SPEED =================
bool sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4);
  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

// ================= HEADING CORRECTION =================
float calculateHeadingCorrection(float vx, float vy, float w) {
  // Only apply heading correction when:
  // 1. Moving forward/backward (vx != 0)
  // 2. Not strafing much (vy ~= 0)
  // 3. Not manually rotating (w ~= 0)
  // 4. Heading is locked

  if (!headingLocked) {
    return 0.0f;
  }

  // Don't correct if manually rotating
  if (abs(w) > 0.1f) {
    return 0.0f;
  }

  // Don't correct if strafing significantly
  if (abs(vy) > 0.3f) {
    return 0.0f;
  }

  // Don't correct if not moving forward/backward
  if (abs(vx) < 0.1f) {
    return 0.0f;
  }

  // Calculate heading error
  float currentHeading = imu.getYaw();
  float error = targetHeading - currentHeading;

  // Normalize error to [-180, 180]
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  // Apply deadzone
  if (abs(error) < HEADING_DEADZONE) {
    error = 0.0f;
  }

  // Calculate dt
  unsigned long now = millis();
  float dt = (now - lastHeadingUpdate) / 1000.0f;
  lastHeadingUpdate = now;

  if (dt > 0.5f) dt = 0.1f; // Prevent large dt on first run

  // PID calculation
  float P = HEADING_KP * error;

  headingErrorIntegral += error * dt;
  headingErrorIntegral = constrain(headingErrorIntegral, -10.0f, 10.0f); // Anti-windup
  float I = HEADING_KI * headingErrorIntegral;

  float D = 0.0f;
  if (dt > 0.001f) {
    D = HEADING_KD * (error - lastHeadingError) / dt;
  }
  lastHeadingError = error;

  float correction = P + I + D;

  // Limit correction magnitude
  correction = constrain(correction, -0.5f, 0.5f);

  return correction;
}

// ================= MECANUM DRIVE =================
void mecanumDrive(float vx, float vy, float w, float maxSpeed) {
  // Apply heading correction
  float headingCorrection = calculateHeadingCorrection(vx, vy, w);
  w += headingCorrection;

  // Calculate wheel speeds using mecanum kinematics
  float fl_speed = vx - vy - w;
  float fr_speed = vx + vy + w;
  float rl_speed = vx + vy - w;
  float rr_speed = vx - vy + w;

  // Normalize to prevent any wheel from exceeding 1.0
  float maxVal = max(max(abs(fl_speed), abs(fr_speed)),
                     max(abs(rl_speed), abs(rr_speed)));
  if (maxVal > 1.0f) {
    fl_speed /= maxVal;
    fr_speed /= maxVal;
    rl_speed /= maxVal;
    rr_speed /= maxVal;
  }

  // Scale to max speed
  fl_speed *= maxSpeed;
  fr_speed *= maxSpeed;
  rl_speed *= maxSpeed;
  rr_speed *= maxSpeed;

  // Apply direction multipliers from Phase 0 testing
  fl_speed *= MOTOR_FL_DIR;
  fr_speed *= MOTOR_FR_DIR;
  rl_speed *= MOTOR_RL_DIR;
  rr_speed *= MOTOR_RR_DIR;

  // Send commands to all motors
  sendSpeed(MOTOR_FL, fl_speed);
  sendSpeed(MOTOR_FR, fr_speed);
  sendSpeed(MOTOR_RL, rl_speed);
  sendSpeed(MOTOR_RR, rr_speed);

  // Diagnostic output every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf("FL:%.1f FR:%.1f RL:%.1f RR:%.1f | Yaw:%.1f° Target:%.1f° Corr:%.3f %s\n",
      fl_speed, fr_speed, rl_speed, rr_speed,
      imu.getYaw(), targetHeading, headingCorrection,
      headingLocked ? "[LOCKED]" : "");
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Mecanum Drive - IMU Integration");
  Serial.println("========================================\n");

  // Initialize IMU
  Serial.println("Initializing IMU...");
  if (imu.begin()) {
    Serial.println("✓ IMU initialized");
  } else {
    Serial.println("✗ IMU initialization failed");
    Serial.println("  Check wiring and continue without IMU");
  }
  delay(500);

  // Initialize CAN
  canInit();
  delay(100);

  // Enable motors sequentially
  Serial.println("\nEnabling motors...");

  enableMotor(MOTOR_FL);
  sendSpeed(MOTOR_FL, 0.0f);
  delay(50);
  Serial.println("Motor FL (1): ✓");

  enableMotor(MOTOR_FR);
  sendSpeed(MOTOR_FR, 0.0f);
  delay(50);
  Serial.println("Motor FR (2): ✓");

  enableMotor(MOTOR_RL);
  sendSpeed(MOTOR_RL, 0.0f);
  delay(50);
  Serial.println("Motor RL (3): ✓");

  enableMotor(MOTOR_RR);
  sendSpeed(MOTOR_RR, 0.0f);
  delay(50);
  Serial.println("Motor RR (4): ✓");

  Serial.println("\nMotors enabled!");

  // Initialize PS5 controller
  PS5.begin("ESP32-PS5");
  Serial.println("Waiting for PS5 controller...");
  Serial.println("Press PS button to connect");
  Serial.println("\nControls:");
  Serial.println("  L1: Lock/Reset heading for straight driving");
  Serial.println("  R2: Speed boost\n");

  lastHeadingUpdate = millis();
}

// ================= LOOP =================
void loop() {
  // Update IMU data
  imu.update();

  // Safety: stop if controller disconnected
  if (!PS5.isConnected()) {
    mecanumDrive(0, 0, 0, MAX_SPEED);
    headingLocked = false;
    delay(100);
    return;
  }

  // Check for heading lock button (L1)
  static bool lastL1State = false;
  bool currentL1State = PS5.L1();
  if (currentL1State && !lastL1State) {
    // Button pressed - toggle heading lock
    if (!headingLocked) {
      targetHeading = imu.getYaw();
      headingLocked = true;
      headingErrorIntegral = 0.0f;
      lastHeadingError = 0.0f;
      Serial.printf("Heading LOCKED at %.1f°\n", targetHeading);
    } else {
      headingLocked = false;
      Serial.println("Heading UNLOCKED");
    }
  }
  lastL1State = currentL1State;

  // Read PS5 controller inputs
  float vx = PS5.LStickY() / 128.0f;   // Forward/backward
  float vy = PS5.LStickX() / 128.0f;   // Strafe left/right
  float w  = PS5.RStickX() / 128.0f;   // Rotation

  // Apply deadzone
  if (abs(vx) < DEADZONE) vx = 0;
  if (abs(vy) < DEADZONE) vy = 0;
  if (abs(w)  < DEADZONE) w  = 0;

  // Unlock heading if manually rotating
  if (abs(w) > 0.1f && headingLocked) {
    headingLocked = false;
    Serial.println("Heading UNLOCKED (manual rotation)");
  }

  // Check for speed boost (R2 button)
  float currentMaxSpeed = MAX_SPEED;
  if (PS5.R2()) {
    currentMaxSpeed = MAX_SPEED * BOOST_MULTIPLIER;
  }

  // Drive the robot
  mecanumDrive(vx, vy, w, currentMaxSpeed);

  delay(10);  // 100Hz update rate
}
