/**
 * WT901C485 IMU Wrapper for ESP32
 *
 * Simplified interface for WitMotion WT901C485 IMU
 * Provides easy access to orientation, acceleration, and gyroscope data
 *
 * Connection: Direct UART (no RS485 converter needed)
 *   WT901C485 TX -> ESP32 RX2 (GPIO 16)
 *   WT901C485 RX -> ESP32 TX2 (GPIO 17)
 *   VCC -> 5V or 3.3V (check your module)
 *   GND -> GND
 */

#ifndef IMU_WT901_H
#define IMU_WT901_H

#include <Arduino.h>
#include <HardwareSerial.h>

// IMU Configuration
#define IMU_UART_NUM 2          // Use Serial2
#define IMU_RX_GPIO 16          // ESP32 RX2
#define IMU_TX_GPIO 17          // ESP32 TX2
#define IMU_BAUD 115200         // Default baud rate
#define IMU_UPDATE_RATE_HZ 100  // 100Hz update rate

// Data update flags
#define IMU_ACC_UPDATE    0x01
#define IMU_GYRO_UPDATE   0x02
#define IMU_ANGLE_UPDATE  0x04
#define IMU_MAG_UPDATE    0x08

class IMU_WT901 {
public:
  IMU_WT901();

  // Initialize IMU
  bool begin(uint32_t baudRate = IMU_BAUD);

  // Update IMU data (call in loop)
  void update();

  // Get orientation (degrees)
  float getRoll() const { return roll; }
  float getPitch() const { return pitch; }
  float getYaw() const { return yaw; }

  // Get angular velocity (deg/s)
  float getGyroX() const { return gyroX; }
  float getGyroY() const { return gyroY; }
  float getGyroZ() const { return gyroZ; }

  // Get acceleration (g)
  float getAccelX() const { return accelX; }
  float getAccelY() const { return accelY; }
  float getAccelZ() const { return accelZ; }

  // Get magnetometer (raw)
  int16_t getMagX() const { return magX; }
  int16_t getMagY() const { return magY; }
  int16_t getMagZ() const { return magZ; }

  // Check if data is available
  bool isDataReady() const { return dataReady; }

  // Calibration functions
  void startAccelCalibration();
  void startMagCalibration();
  void stopMagCalibration();

  // Reset yaw to zero (useful for heading lock)
  void resetYaw();
  float getRelativeYaw() const { return yaw - yawOffset; }

private:
  HardwareSerial* serial;

  // Sensor data
  float roll, pitch, yaw;
  float gyroX, gyroY, gyroZ;
  float accelX, accelY, accelZ;
  int16_t magX, magY, magZ;

  float yawOffset;  // For relative yaw measurements
  bool dataReady;

  // Data parsing
  uint8_t rxBuffer[11];
  uint8_t rxIndex;

  void parseData();
  bool validateChecksum(uint8_t* data, uint8_t len);
  void sendCommand(uint8_t reg, uint16_t data);
};

#endif // IMU_WT901_H
