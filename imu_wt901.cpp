/**
 * WT901C485 IMU Wrapper Implementation
 */

#include "imu_wt901.h"

// WitMotion protocol constants
#define WIT_ACC    0x51
#define WIT_GYRO   0x52
#define WIT_ANGLE  0x53
#define WIT_MAG    0x54

#define WIT_SAVE   0x00
#define WIT_CALSW  0x01
#define WIT_RSW    0x02
#define WIT_RRATE  0x03
#define WIT_BAUD   0x04
#define WIT_AXOFFSET 0x05
#define WIT_AYOFFSET 0x06
#define WIT_AZOFFSET 0x07
#define WIT_GXOFFSET 0x08
#define WIT_GYOFFSET 0x09
#define WIT_GZOFFSET 0x0A
#define WIT_HXOFFSET 0x0B
#define WIT_HYOFFSET 0x0C
#define WIT_HZOFFSET 0x0D
#define WIT_D0MODE   0x0E
#define WIT_D1MODE   0x0F
#define WIT_D2MODE   0x10
#define WIT_D3MODE   0x11
#define WIT_BAUD1    0x12

IMU_WT901::IMU_WT901()
  : serial(nullptr),
    roll(0), pitch(0), yaw(0),
    gyroX(0), gyroY(0), gyroZ(0),
    accelX(0), accelY(0), accelZ(0),
    magX(0), magY(0), magZ(0),
    yawOffset(0),
    dataReady(false),
    rxIndex(0) {
}

bool IMU_WT901::begin(uint32_t baudRate) {
  // Initialize Serial2 for IMU communication
  serial = &Serial2;
  serial->begin(baudRate, SERIAL_8N1, IMU_RX_GPIO, IMU_TX_GPIO);

  delay(100);

  // Clear any pending data
  while (serial->available()) {
    serial->read();
  }

  Serial.println("IMU WT901C485 initialized");
  return true;
}

void IMU_WT901::update() {
  dataReady = false;

  // Read all available bytes
  while (serial->available()) {
    uint8_t byte = serial->read();

    // Look for packet start (0x55)
    if (rxIndex == 0 && byte != 0x55) {
      continue;
    }

    rxBuffer[rxIndex++] = byte;

    // Complete packet received (11 bytes)
    if (rxIndex >= 11) {
      parseData();
      rxIndex = 0;
    }
  }
}

void IMU_WT901::parseData() {
  // Validate checksum
  if (!validateChecksum(rxBuffer, 11)) {
    return;
  }

  uint8_t type = rxBuffer[1];
  int16_t data[4];

  // Extract 16-bit values
  for (int i = 0; i < 4; i++) {
    data[i] = (int16_t)((rxBuffer[3 + i*2] << 8) | rxBuffer[2 + i*2]);
  }

  switch (type) {
    case WIT_ACC:
      // Acceleration (range: ±16g)
      accelX = data[0] / 32768.0f * 16.0f;
      accelY = data[1] / 32768.0f * 16.0f;
      accelZ = data[2] / 32768.0f * 16.0f;
      break;

    case WIT_GYRO:
      // Angular velocity (range: ±2000°/s)
      gyroX = data[0] / 32768.0f * 2000.0f;
      gyroY = data[1] / 32768.0f * 2000.0f;
      gyroZ = data[2] / 32768.0f * 2000.0f;
      break;

    case WIT_ANGLE:
      // Orientation (range: ±180°)
      roll  = data[0] / 32768.0f * 180.0f;
      pitch = data[1] / 32768.0f * 180.0f;
      yaw   = data[2] / 32768.0f * 180.0f;
      dataReady = true;
      break;

    case WIT_MAG:
      // Magnetometer (raw values)
      magX = data[0];
      magY = data[1];
      magZ = data[2];
      break;
  }
}

bool IMU_WT901::validateChecksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (int i = 0; i < len - 1; i++) {
    sum += data[i];
  }
  return (sum == data[len - 1]);
}

void IMU_WT901::sendCommand(uint8_t reg, uint16_t data) {
  uint8_t cmd[5];
  cmd[0] = 0xFF;
  cmd[1] = 0xAA;
  cmd[2] = reg;
  cmd[3] = data & 0xFF;
  cmd[4] = (data >> 8) & 0xFF;

  serial->write(cmd, 5);
  serial->flush();
  delay(50);
}

void IMU_WT901::startAccelCalibration() {
  Serial.println("Starting accelerometer calibration...");
  Serial.println("Place IMU on level surface");
  sendCommand(WIT_CALSW, 0x0001);
}

void IMU_WT901::startMagCalibration() {
  Serial.println("Starting magnetometer calibration...");
  Serial.println("Rotate IMU in figure-8 pattern");
  sendCommand(WIT_CALSW, 0x0007);
}

void IMU_WT901::stopMagCalibration() {
  Serial.println("Stopping magnetometer calibration...");
  sendCommand(WIT_CALSW, 0x0000);
  sendCommand(WIT_SAVE, 0x0000);  // Save calibration
}

void IMU_WT901::resetYaw() {
  yawOffset = yaw;
  Serial.printf("Yaw reset to 0° (offset: %.2f°)\n", yawOffset);
}
