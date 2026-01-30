/**
 * Test: Control just ONE motor using mecanum code structure
 * This will tell us if the issue is with controlling multiple motors
 */

#include <driver/twai.h>
#include "motor_config.h"

#define TEST_MOTOR MOTOR_FR  // Motor 2, which worked in Phase 0

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
void enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 8;

  for (int i = 0; i < 7; i++) msg.data[i] = 0xFF;
  msg.data[7] = 0xFC;

  twai_transmit(&msg, pdMS_TO_TICKS(100));
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

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Test: Single Motor from Mecanum Code");
  Serial.println("========================================\n");
  Serial.printf("Testing Motor %d\n\n", TEST_MOTOR);

  canInit();
  delay(100);

  enableMotor(TEST_MOTOR);
  delay(100);

  sendSpeed(TEST_MOTOR, 0.0f);

  Serial.println("Starting test...\n");
}

// ================= LOOP =================
void loop() {
  static unsigned long testStart = millis();
  unsigned long elapsed = millis() - testStart;

  float speed = 0.0f;

  if (elapsed < 3000) {
    speed = 3.0f;
    if (elapsed < 100) Serial.println("→ Forward: 3.0 rad/s");
  }
  else if (elapsed < 6000) {
    speed = -3.0f;
    if (elapsed < 3100) Serial.println("→ Reverse: -3.0 rad/s");
  }
  else {
    speed = 0.0f;
    if (elapsed < 6100) {
      Serial.println("→ Stop\n");
      Serial.println("TEST COMPLETE");
      Serial.println("Did motor 2 spin? If YES, issue is with multiple motors.");
      Serial.println("If NO, something changed since Phase 0.\n");
    }
  }

  bool ok = sendSpeed(TEST_MOTOR, speed);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf("Speed: %.1f %s\n", speed, ok ? "✓" : "✗");
  }

  delay(100);
}
