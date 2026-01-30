/**
 * Mecanum Drive - Full Integration: Motors + PS5 + IMU + ROS2
 *
 * Features:
 * - PS5 controller for manual control
 * - IMU heading correction for straight driving
 * - micro-ROS integration publishing IMU data to ROS2
 * - Publishes sensor_msgs/Imu on /imu/data topic
 *
 * ROS2 Topics Published:
 *   /imu/data (sensor_msgs/Imu) - Full IMU data at 50Hz
 *   /mecanum/status (std_msgs/String) - Robot status messages
 *
 * Controls:
 *   Left Stick Y: Forward/Backward (with heading correction)
 *   Left Stick X: Strafe Left/Right
 *   Right Stick X: Rotation
 *   R2 Button: Speed Boost (2x speed)
 *   L1 Button: Lock/Reset heading for straight driving
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>

#include <driver/twai.h>
#include <PS5Controller.h>
#include "motor_config.h"
#include "imu_wt901.h"

// ================= PARAMETERS =================
#define MAX_SPEED 6.0f
#define BOOST_MULTIPLIER 2.0f
#define DEADZONE 0.05f

// Heading correction PID
#define HEADING_KP 0.02f
#define HEADING_KI 0.0f
#define HEADING_KD 0.005f
#define HEADING_DEADZONE 2.0f

// ROS2 publish rates
#define IMU_PUBLISH_RATE_MS 20    // 50Hz
#define STATUS_PUBLISH_RATE_MS 1000  // 1Hz

// ================= MICRO-ROS OBJECTS =================
rcl_publisher_t imu_publisher;
rcl_publisher_t status_publisher;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__String status_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

bool microros_connected = false;

// ================= GLOBAL OBJECTS =================
IMU_WT901 imu;

// Heading control state
float targetHeading = 0.0f;
bool headingLocked = false;
float headingErrorIntegral = 0.0f;
float lastHeadingError = 0.0f;
unsigned long lastHeadingUpdate = 0;

// ================= MICRO-ROS SETUP =================
void setupMicroROS() {
  Serial.println("Initializing micro-ROS...");

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "mecanum_robot", "", &support);

  // Create IMU publisher
  rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"
  );

  // Create status publisher
  rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "mecanum/status"
  );

  // Initialize IMU message
  imu_msg.header.frame_id.data = (char*)malloc(20);
  imu_msg.header.frame_id.capacity = 20;
  strcpy(imu_msg.header.frame_id.data, "imu_link");
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

  // Set covariance matrices (unknown = -1)
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = -1;
    imu_msg.angular_velocity_covariance[i] = -1;
    imu_msg.linear_acceleration_covariance[i] = -1;
  }

  // Initialize status message
  status_msg.data.data = (char*)malloc(200);
  status_msg.data.capacity = 200;

  microros_connected = true;
  Serial.println("✓ micro-ROS initialized");
  Serial.println("✓ Publisher: /imu/data (sensor_msgs/Imu)");
  Serial.println("✓ Publisher: /mecanum/status (std_msgs/String)");
}

// ================= PUBLISH IMU DATA =================
void publishIMU() {
  if (!microros_connected) return;

  // Update timestamp
  imu_msg.header.stamp.sec = millis() / 1000;
  imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

  // Convert orientation to quaternion (simplified - assumes level robot)
  float roll_rad = imu.getRoll() * DEG_TO_RAD;
  float pitch_rad = imu.getPitch() * DEG_TO_RAD;
  float yaw_rad = imu.getYaw() * DEG_TO_RAD;

  // Simplified quaternion conversion (Euler to quaternion)
  float cy = cos(yaw_rad * 0.5);
  float sy = sin(yaw_rad * 0.5);
  float cp = cos(pitch_rad * 0.5);
  float sp = sin(pitch_rad * 0.5);
  float cr = cos(roll_rad * 0.5);
  float sr = sin(roll_rad * 0.5);

  imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
  imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
  imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
  imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

  // Angular velocity (rad/s)
  imu_msg.angular_velocity.x = imu.getGyroX() * DEG_TO_RAD;
  imu_msg.angular_velocity.y = imu.getGyroY() * DEG_TO_RAD;
  imu_msg.angular_velocity.z = imu.getGyroZ() * DEG_TO_RAD;

  // Linear acceleration (m/s²)
  imu_msg.linear_acceleration.x = imu.getAccelX() * 9.81;
  imu_msg.linear_acceleration.y = imu.getAccelY() * 9.81;
  imu_msg.linear_acceleration.z = imu.getAccelZ() * 9.81;

  // Publish
  rcl_ret_t ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.printf("IMU publish failed: %d\n", ret);
  }
}

// ================= PUBLISH STATUS =================
void publishStatus(const char* status) {
  if (!microros_connected) return;

  snprintf(status_msg.data.data, status_msg.data.capacity, "%s", status);
  status_msg.data.size = strlen(status_msg.data.data);

  rcl_publish(&status_publisher, &status_msg, NULL);
}

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
bool enableMotor(uint8_t id) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 8;

  for (int i = 0; i < 7; i++) msg.data[i] = 0xFF;
  msg.data[7] = 0xFC;

  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

// ================= SEND SPEED =================
bool sendSpeed(uint8_t id, float speed) {
  twai_message_t msg;
  msg.identifier = SPEED_FRAME_ID(id);
  msg.extd = 0;
  msg.data_length_code = 4;

  memcpy(msg.data, &speed, 4);
  esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(100));
  return (result == ESP_OK);
}

// ================= HEADING CORRECTION =================
float calculateHeadingCorrection(float vx, float vy, float w) {
  if (!headingLocked || abs(w) > 0.1f || abs(vy) > 0.3f || abs(vx) < 0.1f) {
    return 0.0f;
  }

  float currentHeading = imu.getYaw();
  float error = targetHeading - currentHeading;

  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  if (abs(error) < HEADING_DEADZONE) error = 0.0f;

  unsigned long now = millis();
  float dt = (now - lastHeadingUpdate) / 1000.0f;
  lastHeadingUpdate = now;
  if (dt > 0.5f) dt = 0.1f;

  float P = HEADING_KP * error;

  headingErrorIntegral += error * dt;
  headingErrorIntegral = constrain(headingErrorIntegral, -10.0f, 10.0f);
  float I = HEADING_KI * headingErrorIntegral;

  float D = 0.0f;
  if (dt > 0.001f) {
    D = HEADING_KD * (error - lastHeadingError) / dt;
  }
  lastHeadingError = error;

  float correction = P + I + D;
  return constrain(correction, -0.5f, 0.5f);
}

// ================= MECANUM DRIVE =================
void mecanumDrive(float vx, float vy, float w, float maxSpeed) {
  float headingCorrection = calculateHeadingCorrection(vx, vy, w);
  w += headingCorrection;

  float fl_speed = vx - vy - w;
  float fr_speed = vx + vy + w;
  float rl_speed = vx + vy - w;
  float rr_speed = vx - vy + w;

  float maxVal = max(max(abs(fl_speed), abs(fr_speed)),
                     max(abs(rl_speed), abs(rr_speed)));
  if (maxVal > 1.0f) {
    fl_speed /= maxVal;
    fr_speed /= maxVal;
    rl_speed /= maxVal;
    rr_speed /= maxVal;
  }

  fl_speed *= maxSpeed * MOTOR_FL_DIR;
  fr_speed *= maxSpeed * MOTOR_FR_DIR;
  rl_speed *= maxSpeed * MOTOR_RL_DIR;
  rr_speed *= maxSpeed * MOTOR_RR_DIR;

  sendSpeed(MOTOR_FL, fl_speed);
  sendSpeed(MOTOR_FR, fr_speed);
  sendSpeed(MOTOR_RL, rl_speed);
  sendSpeed(MOTOR_RR, rr_speed);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.printf("Yaw:%.1f° Target:%.1f° %s | ROS:%s\n",
      imu.getYaw(), targetHeading,
      headingLocked ? "[LOCKED]" : "",
      microros_connected ? "✓" : "✗");
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  Mecanum Robot - Full Integration");
  Serial.println("========================================\n");

  // Initialize IMU
  Serial.println("Initializing IMU...");
  if (imu.begin()) {
    Serial.println("✓ IMU initialized");
  } else {
    Serial.println("✗ IMU initialization failed");
  }
  delay(500);

  // Initialize micro-ROS
  setupMicroROS();
  delay(500);

  // Initialize CAN
  Serial.println("\nInitializing CAN...");
  canInit();
  delay(100);

  // Enable motors
  Serial.println("Enabling motors...");
  enableMotor(MOTOR_FL); sendSpeed(MOTOR_FL, 0.0f); delay(50);
  enableMotor(MOTOR_FR); sendSpeed(MOTOR_FR, 0.0f); delay(50);
  enableMotor(MOTOR_RL); sendSpeed(MOTOR_RL, 0.0f); delay(50);
  enableMotor(MOTOR_RR); sendSpeed(MOTOR_RR, 0.0f); delay(50);
  Serial.println("✓ Motors enabled");

  // Initialize PS5
  PS5.begin("ESP32-PS5");
  Serial.println("\n✓ System ready");
  Serial.println("Waiting for PS5 controller...\n");

  publishStatus("System initialized");
  lastHeadingUpdate = millis();
}

// ================= LOOP =================
void loop() {
  // Update IMU
  imu.update();

  // Publish IMU data to ROS2
  static unsigned long lastIMUPublish = 0;
  if (millis() - lastIMUPublish >= IMU_PUBLISH_RATE_MS) {
    lastIMUPublish = millis();
    publishIMU();
  }

  // Safety: stop if controller disconnected
  if (!PS5.isConnected()) {
    mecanumDrive(0, 0, 0, MAX_SPEED);
    headingLocked = false;
    delay(100);
    return;
  }

  // Handle L1 button (heading lock)
  static bool lastL1State = false;
  bool currentL1State = PS5.L1();
  if (currentL1State && !lastL1State) {
    if (!headingLocked) {
      targetHeading = imu.getYaw();
      headingLocked = true;
      headingErrorIntegral = 0.0f;
      lastHeadingError = 0.0f;
      Serial.printf("Heading LOCKED at %.1f°\n", targetHeading);
      publishStatus("Heading locked");
    } else {
      headingLocked = false;
      Serial.println("Heading UNLOCKED");
      publishStatus("Heading unlocked");
    }
  }
  lastL1State = currentL1State;

  // Read controller
  float vx = PS5.LStickY() / 128.0f;
  float vy = PS5.LStickX() / 128.0f;
  float w  = PS5.RStickX() / 128.0f;

  if (abs(vx) < DEADZONE) vx = 0;
  if (abs(vy) < DEADZONE) vy = 0;
  if (abs(w)  < DEADZONE) w  = 0;

  if (abs(w) > 0.1f && headingLocked) {
    headingLocked = false;
    Serial.println("Heading UNLOCKED (manual rotation)");
  }

  float currentMaxSpeed = PS5.R2() ? MAX_SPEED * BOOST_MULTIPLIER : MAX_SPEED;

  mecanumDrive(vx, vy, w, currentMaxSpeed);

  delay(10);
}
