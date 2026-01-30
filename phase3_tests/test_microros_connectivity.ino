/**
 * Stage 1: micro-ROS Basic Connectivity Test
 *
 * Tests:
 * - Serial connection to micro-ROS agent
 * - Publisher creation
 * - Heartbeat message publishing
 *
 * No motor control, no PS5 - just validates micro-ROS communication
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// micro-ROS objects
rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 2  // Built-in LED

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(2000);

  Serial.println("\n========================================");
  Serial.println("  Stage 1: micro-ROS Connectivity Test");
  Serial.println("========================================\n");

  // Set micro-ROS transport to Serial
  set_microros_transports();
  delay(2000);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_test_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32/heartbeat"
  );

  // Allocate message memory
  msg.data.data = (char*)malloc(100 * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = 100;

  Serial.println("✓ micro-ROS initialized");
  Serial.println("✓ Publisher: /esp32/heartbeat");
  Serial.println("\nPublishing heartbeat...\n");
}

void loop() {
  static unsigned long lastPublish = 0;
  static int counter = 0;

  if (millis() - lastPublish >= 1000) {
    lastPublish = millis();

    sprintf(msg.data.data, "Heartbeat %d", counter++);
    msg.data.size = strlen(msg.data.data);

    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret == RCL_RET_OK) {
      Serial.printf("✓ Published: %s\n", msg.data.data);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    } else {
      Serial.printf("✗ Publish failed: %d\n", ret);
    }
  }
  delay(10);
}
