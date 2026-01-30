/**
 * CAN Bus Diagnostic Tool
 *
 * This sketch provides detailed CAN bus diagnostics to help identify
 * hardware issues. It will show:
 * - Exact error codes from TX attempts
 * - CAN bus state (active, passive, bus-off)
 * - TX queue status
 * - Alerts and error counters
 */

#include <driver/twai.h>

// ================= CAN PINS =================
#define CAN_TX_GPIO 5
#define CAN_RX_GPIO 4

// ================= TEST CONFIG =================
#define TEST_MOTOR_ID 1
#define SPEED_FRAME_ID(id) (0x200 + (id))

unsigned long testCount = 0;
unsigned long successCount = 0;
unsigned long failCount = 0;

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

  esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
  Serial.printf("CAN driver install: %s\n", esp_err_to_name(result));

  result = twai_start();
  Serial.printf("CAN start: %s\n", esp_err_to_name(result));
}

// ================= GET CAN STATE =================
const char* getCanStateName(twai_state_t state) {
  switch(state) {
    case TWAI_STATE_STOPPED: return "STOPPED";
    case TWAI_STATE_RUNNING: return "RUNNING";
    case TWAI_STATE_BUS_OFF: return "BUS_OFF";
    case TWAI_STATE_RECOVERING: return "RECOVERING";
    default: return "UNKNOWN";
  }
}

// ================= PRINT CAN STATUS =================
void printCanStatus() {
  twai_status_info_t status;
  twai_get_status_info(&status);

  Serial.println("\n========== CAN BUS STATUS ==========");
  Serial.printf("State: %s\n", getCanStateName(status.state));
  Serial.printf("TX Pending: %lu\n", status.msgs_to_tx);
  Serial.printf("RX Pending: %lu\n", status.msgs_to_rx);
  Serial.printf("TX Error Counter: %lu\n", status.tx_error_counter);
  Serial.printf("RX Error Counter: %lu\n", status.rx_error_counter);
  Serial.printf("TX Failed: %lu\n", status.tx_failed_count);
  Serial.printf("RX Missed: %lu\n", status.rx_missed_count);
  Serial.printf("Arbitration Lost: %lu\n", status.arb_lost_count);
  Serial.printf("Bus Error: %lu\n", status.bus_error_count);
  Serial.println("====================================\n");
}

// ================= SEND TEST MESSAGE =================
esp_err_t sendTestSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;
  memcpy(msg.data, &speed, 4);

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return result;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("  CAN BUS DIAGNOSTIC TOOL");
  Serial.println("========================================\n");

  canInit();
  delay(500);

  printCanStatus();

  Serial.println("Starting diagnostic tests...\n");
}

// ================= LOOP =================
void loop() {
  testCount++;

  // Try to send a test message
  float testSpeed = 0.0f;
  esp_err_t result = sendTestSpeed(TEST_MOTOR_ID, testSpeed);

  if (result == ESP_OK) {
    successCount++;
    Serial.printf("[%lu] ✓ TX Success\n", testCount);
  } else {
    failCount++;
    Serial.printf("[%lu] ✗ TX FAILED: %s (0x%x)\n", testCount, esp_err_to_name(result), result);
  }

  // Print detailed status every 10 attempts
  if (testCount % 10 == 0) {
    Serial.printf("\n--- Stats: %lu success, %lu failed (%.1f%% fail rate) ---\n",
                  successCount, failCount,
                  100.0f * failCount / testCount);
    printCanStatus();
  }

  delay(500);  // Test every 500ms
}
