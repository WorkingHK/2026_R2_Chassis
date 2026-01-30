/**
 * Mecanum Drive - Phase 2: ps5 Controller Integration
 *
 * Controls all 4 Damiao 3519 motors via CAN using ps5 controller
 * Based on working Phase 1 code with proper motor enable sequence
 *
 * Controls:
 *   Left Stick Y: Forward/Backward
 *   Left Stick X: Strafe Left/Right
 *   Right Stick X: Rotation
 *   R2 Button: Speed Boost (2x speed)
 *
 * Safety: Motors stop if controller disconnects
 */

#include <driver/twai.h>
#include <ps5Controller.h>
#include "motor_config.h"

// ================= PARAMETERS =================
#define MAX_SPEED 6.0f        // rad/s - normal speed
#define BOOST_MULTIPLIER 2.0f // Speed boost multiplier
#define DEADZONE 0.05f        // Joystick deadzone

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

// ================= MECANUM DRIVE =================
void mecanumDrive(float vx, float vy, float w, float maxSpeed) {
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
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Mecanum Drive - ps5 Controller");
  Serial.println("========================================\n");

  // Initialize CAN
  canInit();
  delay(100);

  // Enable motors sequentially (Phase 1 fix)
  Serial.println("Enabling motors...");

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

  // Initialize ps5 controller
  ps5.begin("0c:27:56:30:02:bd");
  Serial.println("Waiting for ps5 controller...");
  Serial.println("Press PS button to connect\n");
}

// ================= LOOP =================
void loop() {
  // Safety: stop if controller disconnected
  if (!ps5.isConnected()) {
    mecanumDrive(0, 0, 0, MAX_SPEED);
    delay(100);
    return;
  }

  // Read ps5 controller inputs
  float vx = -ps5.LStickY() / 128.0f;  // Forward/backward (inverted)
  float vy =  ps5.LStickX() / 128.0f;  // Strafe left/right
  float w  =  ps5.RStickX() / 128.0f;  // Rotation

  // Apply deadzone
  if (abs(vx) < DEADZONE) vx = 0;
  if (abs(vy) < DEADZONE) vy = 0;
  if (abs(w)  < DEADZONE) w  = 0;

  // Check for speed boost (R2 button)
  float currentMaxSpeed = MAX_SPEED;
  if (ps5.R2()) {
    currentMaxSpeed = MAX_SPEED * BOOST_MULTIPLIER;
  }

  // Drive the robot
  mecanumDrive(vx, vy, w, currentMaxSpeed);

  delay(100);  // 10Hz update rate (Phase 1 timing)
}
