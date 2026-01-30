#include <driver/twai.h>

// ================= CONFIGURATION =================
#define MOTOR_ID 2  // *** CHANGE THIS to test each motor (1, 2, 3, 4) ***
#define SPEED_FRAME_ID (0x200 + MOTOR_ID)
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

// ================= TEST SEQUENCE =================
unsigned long testStartTime = 0;
int testPhase = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.printf("Testing Motor %d\n", MOTOR_ID);
  Serial.println("========================================\n");

  // Initialize CAN (exact copy from working code)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_GPIO,
    (gpio_num_t)CAN_RX_GPIO,
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();

  // Enable motor (exact copy from working code)
  twai_message_t enable_msg;
  enable_msg.identifier = SPEED_FRAME_ID;
  enable_msg.extd = 0;
  enable_msg.data_length_code = 8;
  for (int i = 0; i < 7; i++) enable_msg.data[i] = 0xFF;
  enable_msg.data[7] = 0xFC;

  twai_transmit(&enable_msg, pdMS_TO_TICKS(100));

  Serial.println("Motor enabled. Starting test in 2 seconds...");
  Serial.println("WATCH: Which direction is POSITIVE speed?\n");

  testStartTime = millis();
}

void loop() {
  unsigned long elapsed = millis() - testStartTime;
  float speed = 0.0f;

  // Simple test sequence
  if (elapsed < 2000) {
    // Wait 2 seconds
    speed = 0.0f;
  } else if (elapsed < 5000) {
    // Forward at +5 rad/s for 3 seconds
    speed = 5.0f;
    if (testPhase == 0) {
      Serial.println("→ Forward: +5.0 rad/s");
      testPhase = 1;
    }
  } else if (elapsed < 8000) {
    // Reverse at -5 rad/s for 3 seconds
    speed = -5.0f;
    if (testPhase == 1) {
      Serial.println("→ Reverse: -5.0 rad/s");
      testPhase = 2;
    }
  } else {
    // Stop
    speed = 0.0f;
    if (testPhase == 2) {
      Serial.println("→ Stop\n");
      Serial.println("========================================");
      Serial.println("TEST COMPLETE");
      Serial.println("========================================");
      Serial.println("Record which direction was POSITIVE");
      Serial.println("Change MOTOR_ID and repeat");
      Serial.println("========================================\n");
      testPhase = 3;
    }
  }

  // Send speed command (exact copy from working code)
  twai_message_t speed_msg;
  speed_msg.identifier = SPEED_FRAME_ID;
  speed_msg.extd = 0;
  speed_msg.data_length_code = 4;

  memcpy(speed_msg.data, &speed, 4);
  twai_transmit(&speed_msg, pdMS_TO_TICKS(100));

  delay(100); // Send every 100ms (exact same as working code)
}
