/**
 * PHASE 0 - Single Motor Test
 *
 * Purpose: Test one motor at a time to validate CAN communication,
 *          speed control, and determine positive direction.
 *
 * Instructions:
 * 1. Change TEST_MOTOR_ID to test each motor (1, 2, 3, 4)
 * 2. Observe which direction the wheel spins for positive speed
 * 3. Record results in PHASE0_RESULTS.md
 * 4. Mark the physical direction with tape if helpful
 */

#include <driver/twai.h>

// ================= CAN PINS =================
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

// ================= TEST CONFIGURATION =================
#define TEST_MOTOR_ID 1      // *** CHANGE THIS to test each motor (1, 2, 3, 4) ***
#define TEST_SPEED 5.0f      // rad/s - safe test speed
#define RAMP_STEP 0.5f       // rad/s per step
#define HOLD_TIME_MS 2000    // Hold at target speed for 2 seconds

#define SPEED_FRAME_ID(id) (0x200 + (id))

// ================= STATE MACHINE =================
enum TestState {
  STATE_INIT,
  STATE_RAMP_UP_FORWARD,
  STATE_HOLD_FORWARD,
  STATE_RAMP_DOWN,
  STATE_RAMP_UP_REVERSE,
  STATE_HOLD_REVERSE,
  STATE_STOP,
  STATE_COMPLETE
};

TestState currentState = STATE_INIT;
float currentSpeed = 0.0f;
unsigned long stateStartTime = 0;

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

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("✓ CAN driver installed");
  } else {
    Serial.println("✗ CAN driver install FAILED");
  }

  if (twai_start() == ESP_OK) {
    Serial.println("✓ CAN started");
  } else {
    Serial.println("✗ CAN start FAILED");
  }
}

// ================= ENABLE MOTOR =================
void enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 8;

  for (int i = 0; i < 7; i++) msg.data[i] = 0xFF;
  msg.data[7] = 0xFC;   // Enable command

  if (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.printf("✓ Motor %d enabled\n", id);
  } else {
    Serial.printf("✗ Motor %d enable FAILED\n", id);
  }
}

// ================= SEND SPEED =================
bool sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4); // float, little-endian

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  PHASE 0 - Single Motor Test");
  Serial.println("========================================");
  Serial.printf("Testing Motor ID: %d\n", TEST_MOTOR_ID);
  Serial.printf("Test Speed: %.1f rad/s\n", TEST_SPEED);
  Serial.println("========================================\n");

  canInit();
  delay(100);

  enableMotor(TEST_MOTOR_ID);
  delay(100);

  // Send initial 0 speed to keep motor enabled
  sendSpeed(TEST_MOTOR_ID, 0.0f);

  Serial.println("Starting test sequence in 2 seconds...");
  Serial.println("WATCH THE MOTOR and note which direction is POSITIVE\n");
  delay(2000);

  stateStartTime = millis();
}

// ================= LOOP =================
void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  // Update at 50Hz
  if (now - lastUpdate < 20) {
    return;
  }
  lastUpdate = now;

  // State machine
  switch (currentState) {
    case STATE_INIT:
      Serial.println("→ Ramping up to +5 rad/s (FORWARD)");
      currentState = STATE_RAMP_UP_FORWARD;
      currentSpeed = 0.0f;
      break;

    case STATE_RAMP_UP_FORWARD:
      currentSpeed += RAMP_STEP;
      if (currentSpeed >= TEST_SPEED) {
        currentSpeed = TEST_SPEED;
        currentState = STATE_HOLD_FORWARD;
        stateStartTime = now;
        Serial.printf("→ Holding at +%.1f rad/s for %d ms\n", TEST_SPEED, HOLD_TIME_MS);
      }
      break;

    case STATE_HOLD_FORWARD:
      if (now - stateStartTime >= HOLD_TIME_MS) {
        Serial.println("→ Ramping down to 0");
        currentState = STATE_RAMP_DOWN;
      }
      break;

    case STATE_RAMP_DOWN:
      currentSpeed -= RAMP_STEP;
      if (currentSpeed <= 0.0f) {
        currentSpeed = 0.0f;
        currentState = STATE_RAMP_UP_REVERSE;
        Serial.println("→ Ramping up to -5 rad/s (REVERSE)");
      }
      break;

    case STATE_RAMP_UP_REVERSE:
      currentSpeed -= RAMP_STEP;
      if (currentSpeed <= -TEST_SPEED) {
        currentSpeed = -TEST_SPEED;
        currentState = STATE_HOLD_REVERSE;
        stateStartTime = now;
        Serial.printf("→ Holding at -%.1f rad/s for %d ms\n", TEST_SPEED, HOLD_TIME_MS);
      }
      break;

    case STATE_HOLD_REVERSE:
      if (now - stateStartTime >= HOLD_TIME_MS) {
        Serial.println("→ Stopping motor");
        currentState = STATE_STOP;
      }
      break;

    case STATE_STOP:
      currentSpeed = 0.0f;
      sendSpeed(TEST_MOTOR_ID, currentSpeed);
      delay(500);
      currentState = STATE_COMPLETE;
      break;

    case STATE_COMPLETE:
      Serial.println("\n========================================");
      Serial.println("  TEST COMPLETE");
      Serial.println("========================================");
      Serial.printf("Motor %d tested successfully\n", TEST_MOTOR_ID);
      Serial.println("\nRecord your observations:");
      Serial.println("- Which direction was POSITIVE speed?");
      Serial.println("- Did the motor respond smoothly?");
      Serial.println("- Any unusual behavior?");
      Serial.println("\nChange TEST_MOTOR_ID and repeat for all motors");
      Serial.println("========================================\n");

      while(1) { delay(1000); } // Stop here
      break;
  }

  // Send speed command
  bool txSuccess = sendSpeed(TEST_MOTOR_ID, currentSpeed);

  // Print current speed every 500ms
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    if (txSuccess) {
      Serial.printf("Speed: %+.2f rad/s [TX OK]\n", currentSpeed);
    } else {
      Serial.printf("Speed: %+.2f rad/s [TX FAILED!]\n", currentSpeed);
    }
  }
}
