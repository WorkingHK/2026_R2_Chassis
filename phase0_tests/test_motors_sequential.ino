/**
 * Test: Control motors ONE AT A TIME in sequence
 * This will tell us if motors 1, 3, 4 work individually
 */

#include <driver/twai.h>
#include "motor_config.h"

unsigned long testStart = 0;

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
void sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(100));
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Test: Motors One at a Time");
  Serial.println("========================================\n");
  Serial.println("Will test each motor for 3 seconds");
  Serial.println("Watch which motors spin!\n");

  canInit();
  delay(100);

  testStart = millis();
}

// ================= LOOP =================
void loop() {
  unsigned long elapsed = millis() - testStart;

  uint8_t current_motor = 0;
  float speed = 0.0f;
  const char* motor_name = "";

  // Test each motor for 3 seconds with 2-second warm-up
  if (elapsed < 2000) {
    // Warm-up for motor 1
    current_motor = MOTOR_FL;
    speed = 0.0f;
    motor_name = "FL (1)";
    if (elapsed < 100) {
      enableMotor(current_motor);
      Serial.println("→ Testing Motor FL (1) - Warm-up");
    }
  }
  else if (elapsed < 5000) {
    // Test motor 1
    current_motor = MOTOR_FL;
    speed = 3.0f;
    motor_name = "FL (1)";
    if (elapsed < 2100) Serial.println("→ Testing Motor FL (1) - 3.0 rad/s");
  }
  else if (elapsed < 7000) {
    // Warm-up for motor 2
    current_motor = MOTOR_FR;
    speed = 0.0f;
    motor_name = "FR (2)";
    if (elapsed < 5100) {
      enableMotor(current_motor);
      Serial.println("→ Testing Motor FR (2) - Warm-up");
    }
  }
  else if (elapsed < 10000) {
    // Test motor 2
    current_motor = MOTOR_FR;
    speed = 3.0f;
    motor_name = "FR (2)";
    if (elapsed < 7100) Serial.println("→ Testing Motor FR (2) - 3.0 rad/s");
  }
  else if (elapsed < 12000) {
    // Warm-up for motor 3
    current_motor = MOTOR_RL;
    speed = 0.0f;
    motor_name = "RL (3)";
    if (elapsed < 10100) {
      enableMotor(current_motor);
      Serial.println("→ Testing Motor RL (3) - Warm-up");
    }
  }
  else if (elapsed < 15000) {
    // Test motor 3
    current_motor = MOTOR_RL;
    speed = 3.0f;
    motor_name = "RL (3)";
    if (elapsed < 12100) Serial.println("→ Testing Motor RL (3) - 3.0 rad/s");
  }
  else if (elapsed < 17000) {
    // Warm-up for motor 4
    current_motor = MOTOR_RR;
    speed = 0.0f;
    motor_name = "RR (4)";
    if (elapsed < 15100) {
      enableMotor(current_motor);
      Serial.println("→ Testing Motor RR (4) - Warm-up");
    }
  }
  else if (elapsed < 20000) {
    // Test motor 4
    current_motor = MOTOR_RR;
    speed = 3.0f;
    motor_name = "RR (4)";
    if (elapsed < 17100) Serial.println("→ Testing Motor RR (4) - 3.0 rad/s");
  }
  else {
    // Done
    speed = 0.0f;
    if (elapsed < 20100) {
      Serial.println("\n========================================");
      Serial.println("  TEST COMPLETE");
      Serial.println("========================================");
      Serial.println("Which motors spun?");
    }
    return;  // Stop sending commands
  }

  // Send speed command to current motor only
  sendSpeed(current_motor, speed);
  delay(100);
}
