/**
 * PHASE 0 - Four Motor Sequential Test
 *
 * Purpose: Test all 4 motors one after another to confirm:
 *          - No CAN ID conflicts
 *          - Each motor responds independently
 *          - All motors can be controlled from same program
 *
 * Instructions:
 * 1. Upload and run this sketch
 * 2. Watch each motor spin in sequence
 * 3. Verify each motor only moves when commanded
 * 4. Record any issues in PHASE0_RESULTS.md
 */

#include <driver/twai.h>

// ================= CAN PINS =================
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

// ================= MOTOR IDs =================
#define MOTOR_FL 1   // Front Left
#define MOTOR_FR 2   // Front Right
#define MOTOR_RL 3   // Rear Left
#define MOTOR_RR 4   // Rear Right

#define SPEED_FRAME_ID(id) (0x200 + (id))

// ================= TEST CONFIGURATION =================
#define TEST_SPEED 5.0f      // rad/s
#define RAMP_STEP 0.5f       // rad/s per step
#define HOLD_TIME_MS 2000    // Hold at speed
#define PAUSE_BETWEEN_MS 1000 // Pause between motors

// ================= STATE MACHINE =================
enum TestState {
  STATE_INIT,
  STATE_TEST_MOTOR,
  STATE_PAUSE,
  STATE_COMPLETE
};

TestState currentState = STATE_INIT;
uint8_t currentMotorIndex = 0;
uint8_t motorSequence[] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};
const char* motorNames[] = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
float currentSpeed = 0.0f;
unsigned long stateStartTime = 0;
int testPhase = 0; // 0=ramp up, 1=hold forward, 2=ramp down, 3=ramp reverse, 4=hold reverse, 5=stop

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

  memcpy(msg.data, &speed, 4);

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
  return (result == ESP_OK);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  PHASE 0 - Sequential Motor Test");
  Serial.println("========================================");
  Serial.println("Testing motors in sequence:");
  Serial.println("  1. Front Left (FL)");
  Serial.println("  2. Front Right (FR)");
  Serial.println("  3. Rear Left (RL)");
  Serial.println("  4. Rear Right (RR)");
  Serial.println("========================================\n");

  canInit();
  delay(100);

  // Enable all motors
  Serial.println("Enabling all motors...");
  enableMotor(MOTOR_FL);
  enableMotor(MOTOR_FR);
  enableMotor(MOTOR_RL);
  enableMotor(MOTOR_RR);
  delay(100);

  Serial.println("\nStarting sequential test in 2 seconds...");
  Serial.println("WATCH: Only one motor should move at a time\n");
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

  uint8_t currentMotor = motorSequence[currentMotorIndex];

  switch (currentState) {
    case STATE_INIT:
      Serial.printf("\n→ Testing Motor %d (%s)\n", currentMotor, motorNames[currentMotorIndex]);
      currentState = STATE_TEST_MOTOR;
      testPhase = 0;
      currentSpeed = 0.0f;
      stateStartTime = now;
      break;

    case STATE_TEST_MOTOR:
      // Run through test phases
      switch (testPhase) {
        case 0: // Ramp up forward
          currentSpeed += RAMP_STEP;
          if (currentSpeed >= TEST_SPEED) {
            currentSpeed = TEST_SPEED;
            testPhase = 1;
            stateStartTime = now;
            Serial.printf("  Holding at +%.1f rad/s\n", TEST_SPEED);
          }
          break;

        case 1: // Hold forward
          if (now - stateStartTime >= HOLD_TIME_MS) {
            testPhase = 2;
          }
          break;

        case 2: // Ramp down
          currentSpeed -= RAMP_STEP;
          if (currentSpeed <= 0.0f) {
            currentSpeed = 0.0f;
            testPhase = 3;
          }
          break;

        case 3: // Ramp up reverse
          currentSpeed -= RAMP_STEP;
          if (currentSpeed <= -TEST_SPEED) {
            currentSpeed = -TEST_SPEED;
            testPhase = 4;
            stateStartTime = now;
            Serial.printf("  Holding at -%.1f rad/s\n", TEST_SPEED);
          }
          break;

        case 4: // Hold reverse
          if (now - stateStartTime >= HOLD_TIME_MS) {
            testPhase = 5;
          }
          break;

        case 5: // Stop
          currentSpeed = 0.0f;
          sendSpeed(currentMotor, currentSpeed);
          Serial.printf("✓ Motor %d test complete\n", currentMotor);

          // Move to next motor or complete
          currentMotorIndex++;
          if (currentMotorIndex >= 4) {
            currentState = STATE_COMPLETE;
          } else {
            currentState = STATE_PAUSE;
            stateStartTime = now;
          }
          break;
      }

      // Send speed command
      if (!sendSpeed(currentMotor, currentSpeed)) {
        Serial.println("✗ CAN TX FAILED!");
      }
      break;

    case STATE_PAUSE:
      if (now - stateStartTime >= PAUSE_BETWEEN_MS) {
        currentState = STATE_INIT;
      }
      break;

    case STATE_COMPLETE:
      Serial.println("\n========================================");
      Serial.println("  ALL MOTORS TESTED");
      Serial.println("========================================");
      Serial.println("✓ Front Left (FL)");
      Serial.println("✓ Front Right (FR)");
      Serial.println("✓ Rear Left (RL)");
      Serial.println("✓ Rear Right (RR)");
      Serial.println("\nVerify:");
      Serial.println("- Each motor moved only when commanded");
      Serial.println("- No unexpected movements");
      Serial.println("- No CAN TX failures");
      Serial.println("========================================\n");

      while(1) { delay(1000); }
      break;
  }
}
