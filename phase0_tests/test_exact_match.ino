/**
 * Test: Exact match to working code structure
 * This should work if we match the timing exactly
 */

#include <driver/twai.h>
#include "motor_config.h"

#define TEST_MOTOR MOTOR_FR  // Motor 2

unsigned long testStartTime = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Test: Exact Match to Working Code");
  Serial.println("========================================\n");

  // CAN init
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_GPIO,
    (gpio_num_t)CAN_RX_GPIO,
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // Enable motor
  twai_message_t enable_msg;
  enable_msg.identifier = SPEED_FRAME_ID(TEST_MOTOR);
  enable_msg.extd = 0;
  enable_msg.data_length_code = 8;
  for (int i = 0; i < 7; i++) enable_msg.data[i] = 0xFF;
  enable_msg.data[7] = 0xFC;

  twai_transmit(&enable_msg, pdMS_TO_TICKS(100));

  Serial.println("Motor enabled. Starting test...\n");

  testStartTime = millis();  // Set BEFORE loop starts
}

void loop() {
  unsigned long elapsed = millis() - testStartTime;
  float speed = 0.0f;

  // Exact same timing as working code
  if (elapsed < 2000) {
    // Wait 2 seconds with 0 speed
    speed = 0.0f;
  } else if (elapsed < 5000) {
    // Forward at +5 rad/s for 3 seconds
    speed = 5.0f;
    if (elapsed < 2100) Serial.println("→ Forward: +5.0 rad/s");
  } else if (elapsed < 8000) {
    // Reverse at -5 rad/s for 3 seconds
    speed = -5.0f;
    if (elapsed < 5100) Serial.println("→ Reverse: -5.0 rad/s");
  } else {
    // Stop
    speed = 0.0f;
    if (elapsed < 8100) {
      Serial.println("→ Stop\n");
      Serial.println("TEST COMPLETE");
      Serial.println("If this works, the issue is timing/structure.");
    }
  }

  // Send speed command - inline like working code
  twai_message_t speed_msg;
  speed_msg.identifier = SPEED_FRAME_ID(TEST_MOTOR);
  speed_msg.extd = 0;
  speed_msg.data_length_code = 4;

  memcpy(speed_msg.data, &speed, 4);
  twai_transmit(&speed_msg, pdMS_TO_TICKS(100));

  delay(100);
}
