/**
 * PHASE 0 - Four Motor Simultaneous Test
 *
 * Purpose: Stress test CAN bus by commanding all 4 motors simultaneously
 *          - Validate CAN bus can handle traffic
 *          - Check for TX failures
 *          - Measure timing consistency
 *          - Test sustained load (60 second test)
 *
 * Instructions:
 * 1. Upload and run this sketch
 * 2. Watch all 4 motors spin with different speeds
 * 3. Monitor serial output for TX failures
 * 4. Record results in PHASE0_RESULTS.md
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
#define CONTROL_RATE_HZ 50           // 50Hz control loop
#define CONTROL_PERIOD_MS (1000/CONTROL_RATE_HZ)
#define SUSTAINED_TEST_DURATION_MS 60000  // 60 seconds

// Test speeds (different for each motor to see independent control)
#define SPEED_FL 5.0f
#define SPEED_FR 3.0f
#define SPEED_RL 4.0f
#define SPEED_RR 2.0f

// ================= STATISTICS =================
unsigned long txFailures = 0;
unsigned long txSuccesses = 0;
unsigned long loopCount = 0;
unsigned long minLoopTime = 999999;
unsigned long maxLoopTime = 0;
unsigned long totalLoopTime = 0;

// ================= STATE MACHINE =================
enum TestState {
  STATE_INIT,
  STATE_FORWARD,
  STATE_REVERSE,
  STATE_SUSTAINED_TEST,
  STATE_COMPLETE
};

TestState currentState = STATE_INIT;
unsigned long stateStartTime = 0;
unsigned long testStartTime = 0;

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

// ================= SEND SPEED (with statistics) =================
bool sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4);

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));

  if (result == ESP_OK) {
    txSuccesses++;
    return true;
  } else {
    txFailures++;
    return false;
  }
}

// ================= SEND ALL MOTORS =================
void sendAllMotors(float fl, float fr, float rl, float rr) {
  sendSpeed(MOTOR_FL, fl);
  sendSpeed(MOTOR_FR, fr);
  sendSpeed(MOTOR_RL, rl);
  sendSpeed(MOTOR_RR, rr);
}

// ================= PRINT STATISTICS =================
void printStatistics() {
  unsigned long totalTx = txSuccesses + txFailures;
  float failureRate = (totalTx > 0) ? (100.0f * txFailures / totalTx) : 0.0f;
  float avgLoopTime = (loopCount > 0) ? (totalLoopTime / (float)loopCount) : 0.0f;

  Serial.println("\n========================================");
  Serial.println("  STATISTICS");
  Serial.println("========================================");
  Serial.printf("Total TX:       %lu\n", totalTx);
  Serial.printf("TX Successes:   %lu\n", txSuccesses);
  Serial.printf("TX Failures:    %lu\n", txFailures);
  Serial.printf("Failure Rate:   %.2f%%\n", failureRate);
  Serial.println("----------------------------------------");
  Serial.printf("Loop Count:     %lu\n", loopCount);
  Serial.printf("Min Loop Time:  %lu ms\n", minLoopTime);
  Serial.printf("Max Loop Time:  %lu ms\n", maxLoopTime);
  Serial.printf("Avg Loop Time:  %.2f ms\n", avgLoopTime);
  Serial.printf("Target Period:  %d ms\n", CONTROL_PERIOD_MS);
  Serial.println("========================================\n");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  PHASE 0 - Simultaneous Motor Test");
  Serial.println("========================================");
  Serial.println("This test will:");
  Serial.println("  1. Command all 4 motors with different speeds");
  Serial.println("  2. Run forward for 3 seconds");
  Serial.println("  3. Run reverse for 3 seconds");
  Serial.println("  4. Run sustained test for 60 seconds");
  Serial.println("  5. Monitor CAN TX failures and timing");
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

  Serial.println("\nStarting test in 2 seconds...");
  Serial.println("WATCH: All 4 motors should spin simultaneously\n");
  delay(2000);

  stateStartTime = millis();
  testStartTime = millis();
}

// ================= LOOP =================
void loop() {
  unsigned long loopStartTime = micros();
  unsigned long now = millis();

  switch (currentState) {
    case STATE_INIT:
      Serial.println("→ Phase 1: Forward motion (3 seconds)");
      Serial.printf("  FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f rad/s\n",
                    SPEED_FL, SPEED_FR, SPEED_RL, SPEED_RR);
      currentState = STATE_FORWARD;
      stateStartTime = now;
      break;

    case STATE_FORWARD:
      sendAllMotors(SPEED_FL, SPEED_FR, SPEED_RL, SPEED_RR);

      if (now - stateStartTime >= 3000) {
        Serial.println("\n→ Phase 2: Reverse motion (3 seconds)");
        Serial.printf("  FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f rad/s\n",
                      -SPEED_FL, -SPEED_FR, -SPEED_RL, -SPEED_RR);
        currentState = STATE_REVERSE;
        stateStartTime = now;
      }
      break;

    case STATE_REVERSE:
      sendAllMotors(-SPEED_FL, -SPEED_FR, -SPEED_RL, -SPEED_RR);

      if (now - stateStartTime >= 3000) {
        Serial.println("\n→ Phase 3: Sustained load test (60 seconds)");
        Serial.println("  Varying speeds to simulate real control...");
        currentState = STATE_SUSTAINED_TEST;
        stateStartTime = now;
        txFailures = 0;
        txSuccesses = 0;
        loopCount = 0;
        minLoopTime = 999999;
        maxLoopTime = 0;
        totalLoopTime = 0;
      }
      break;

    case STATE_SUSTAINED_TEST: {
      // Vary speeds sinusoidally to simulate real control
      float t = (now - stateStartTime) / 1000.0f; // time in seconds
      float fl = SPEED_FL * sin(t * 0.5f);
      float fr = SPEED_FR * sin(t * 0.7f);
      float rl = SPEED_RL * sin(t * 0.6f);
      float rr = SPEED_RR * sin(t * 0.8f);

      sendAllMotors(fl, fr, rl, rr);

      // Print progress every 10 seconds
      static unsigned long lastProgressPrint = 0;
      if (now - lastProgressPrint >= 10000) {
        lastProgressPrint = now;
        unsigned long elapsed = (now - stateStartTime) / 1000;
        Serial.printf("  %lu/60 seconds | TX Failures: %lu\n", elapsed, txFailures);
      }

      if (now - stateStartTime >= SUSTAINED_TEST_DURATION_MS) {
        sendAllMotors(0, 0, 0, 0); // Stop all motors
        currentState = STATE_COMPLETE;
      }
      break;
    }

    case STATE_COMPLETE:
      Serial.println("\n========================================");
      Serial.println("  TEST COMPLETE");
      Serial.println("========================================");
      printStatistics();

      if (txFailures == 0) {
        Serial.println("✓ PASS: No CAN TX failures detected");
      } else {
        Serial.println("✗ FAIL: CAN TX failures detected");
      }

      if (maxLoopTime <= CONTROL_PERIOD_MS + 1) {
        Serial.println("✓ PASS: Timing within acceptable range");
      } else {
        Serial.println("✗ FAIL: Loop timing exceeded target");
      }

      Serial.println("\nRecord these results in PHASE0_RESULTS.md");
      Serial.println("========================================\n");

      while(1) { delay(1000); }
      break;
  }

  // Measure loop timing
  unsigned long loopEndTime = micros();
  unsigned long loopDuration = (loopEndTime - loopStartTime) / 1000; // Convert to ms

  if (currentState == STATE_SUSTAINED_TEST) {
    loopCount++;
    totalLoopTime += loopDuration;
    if (loopDuration < minLoopTime) minLoopTime = loopDuration;
    if (loopDuration > maxLoopTime) maxLoopTime = loopDuration;
  }

  // Maintain control rate
  delay(CONTROL_PERIOD_MS);
}
