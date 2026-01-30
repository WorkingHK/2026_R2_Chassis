/**
 * Minimal Motor Test - Exact copy of working code structure
 * Just test motor 2 with constant speed
 */

#include <driver/twai.h>

// ================= CAN PINS =================
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

// ================= MOTOR ID =================
#define TEST_MOTOR 2
#define SPEED_FRAME_ID(id) (0x200 + (id))

// ================= CAN INIT =================
void canInit() {
  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(
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
  msg.data[7] = 0xFC;   // Enable command

  twai_transmit(&msg, pdMS_TO_TICKS(100));
}

// ================= SEND SPEED =================
void sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("Minimal Motor 2 Test");
  Serial.println("Motor should spin at 3 rad/s");

  canInit();
  delay(100);

  enableMotor(TEST_MOTOR);
  delay(100);

  Serial.println("Starting...");
}

// ================= LOOP =================
void loop() {
  float testSpeed = 3.0f;  // Constant 3 rad/s

  sendSpeed(TEST_MOTOR, testSpeed);

  Serial.printf("Sending speed: %.1f rad/s\n", testSpeed);

  delay(10);  // 100 Hz, same as your original code
}
