/**
 * WT901C485 IMU Test
 *
 * Simple test to verify IMU communication and data reading
 *
 * Wiring:
 *   WT901C485 TX -> ESP32 GPIO 16 (RX2)
 *   WT901C485 RX -> ESP32 GPIO 17 (TX2)
 *   WT901C485 VCC -> 5V or 3.3V
 *   WT901C485 GND -> GND
 *
 * Expected output:
 *   Roll, Pitch, Yaw angles
 *   Gyroscope X, Y, Z
 *   Accelerometer X, Y, Z
 */

#include "imu_wt901.h"

IMU_WT901 imu;

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  WT901C485 IMU Test");
  Serial.println("========================================\n");

  // Initialize IMU
  if (imu.begin()) {
    Serial.println("✓ IMU initialized");
  } else {
    Serial.println("✗ IMU initialization failed");
    while (1) delay(1000);
  }

  Serial.println("\nReading IMU data...\n");
  delay(1000);
}

void loop() {
  // Update IMU data
  imu.update();

  // Print data every 200ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    Serial.println("─────────────────────────────────────");

    // Orientation
    Serial.printf("Orientation:\n");
    Serial.printf("  Roll:  %7.2f°\n", imu.getRoll());
    Serial.printf("  Pitch: %7.2f°\n", imu.getPitch());
    Serial.printf("  Yaw:   %7.2f°\n", imu.getYaw());

    // Gyroscope
    Serial.printf("\nGyroscope (°/s):\n");
    Serial.printf("  X: %8.2f\n", imu.getGyroX());
    Serial.printf("  Y: %8.2f\n", imu.getGyroY());
    Serial.printf("  Z: %8.2f\n", imu.getGyroZ());

    // Accelerometer
    Serial.printf("\nAccelerometer (g):\n");
    Serial.printf("  X: %6.3f\n", imu.getAccelX());
    Serial.printf("  Y: %6.3f\n", imu.getAccelY());
    Serial.printf("  Z: %6.3f\n", imu.getAccelZ());

    Serial.println();
  }

  delay(10);
}
