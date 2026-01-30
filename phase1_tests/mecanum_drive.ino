/**
 * Mecanum Drive - Phase 1
 *
 * Controls all 4 Damiao 3519 motors via CAN for mecanum wheel drive
 * Uses validated CAN structure from Phase 0 testing
 *
 * Kinematics:
 *   FL = vx - vy - w
 *   FR = vx + vy + w
 *   RL = vx + vy - w
 *   RR = vx - vy + w
 *
 * Where:
 *   vx = forward/backward velocity (positive = forward)
 *   vy = strafe velocity (positive = right)
 *   w = rotation velocity (positive = clockwise)
 */

#include <driver/twai.h>
#include "motor_config.h"

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
void mecanumDrive(float vx, float vy, float w) {
  // Calculate wheel speeds using mecanum kinematics
  float fl_speed = vx - vy - w;
  float fr_speed = vx + vy + w;
  float rl_speed = vx + vy - w;
  float rr_speed = vx - vy + w;

  // Apply direction multipliers from Phase 0 testing
  fl_speed *= MOTOR_FL_DIR;
  fr_speed *= MOTOR_FR_DIR;
  rl_speed *= MOTOR_RL_DIR;
  rr_speed *= MOTOR_RR_DIR;

  // Send commands to all motors and track success
  bool fl_ok = sendSpeed(MOTOR_FL, fl_speed);
  bool fr_ok = sendSpeed(MOTOR_FR, fr_speed);
  bool rl_ok = sendSpeed(MOTOR_RL, rl_speed);
  bool rr_ok = sendSpeed(MOTOR_RR, rr_speed);

  // Print diagnostic info every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf("FL:%.1f%s FR:%.1f%s RL:%.1f%s RR:%.1f%s\n",
      fl_speed, fl_ok ? "✓" : "✗",
      fr_speed, fr_ok ? "✓" : "✗",
      rl_speed, rl_ok ? "✓" : "✗",
      rr_speed, rr_ok ? "✓" : "✗");
  }
}

// ================= TIMING =================
unsigned long testStart = 0;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Mecanum Drive - Phase 1");
  Serial.println("========================================\n");

  canInit();
  delay(100);

  // Enable all motors ONE AT A TIME with immediate speed command
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

  Serial.println("Motors enabled. Starting test sequence...\n");

  testStart = millis();  // Set timing BEFORE loop starts
}

// ================= LOOP =================
void loop() {
  unsigned long elapsed = millis() - testStart;

  float vx = 0.0f;  // Forward/backward
  float vy = 0.0f;  // Strafe left/right
  float w = 0.0f;   // Rotation

  // Test sequence with 2-second warm-up
  if (elapsed < 2000) {
    // Warm-up: 2 seconds with 0 speed
    vx = vy = w = 0.0f;
    if (elapsed < 100) Serial.println("→ Warm-up (2 seconds)");
  }
  else if (elapsed < 5000) {
    // Forward
    vx = 3.0f;
    if (elapsed < 2100) Serial.println("→ Forward (vx=3.0)");
  }
  else if (elapsed < 8000) {
    // Backward
    vx = -3.0f;
    if (elapsed < 5100) Serial.println("→ Backward (vx=-3.0)");
  }
  else if (elapsed < 11000) {
    // Strafe right
    vy = 3.0f;
    if (elapsed < 8100) Serial.println("→ Strafe Right (vy=3.0)");
  }
  else if (elapsed < 14000) {
    // Strafe left
    vy = -3.0f;
    if (elapsed < 11100) Serial.println("→ Strafe Left (vy=-3.0)");
  }
  else if (elapsed < 17000) {
    // Rotate clockwise
    w = 2.0f;
    if (elapsed < 14100) Serial.println("→ Rotate CW (w=2.0)");
  }
  else if (elapsed < 20000) {
    // Rotate counter-clockwise
    w = -2.0f;
    if (elapsed < 17100) Serial.println("→ Rotate CCW (w=-2.0)");
  }
  else {
    // Stop
    vx = vy = w = 0.0f;
    if (elapsed < 20100) {
      Serial.println("→ Stop\n");
      Serial.println("========================================");
      Serial.println("  TEST COMPLETE");
      Serial.println("========================================");
    }
  }

  // Execute mecanum drive
  mecanumDrive(vx, vy, w);

  delay(100);  // 10Hz update rate (same as working code)
}
