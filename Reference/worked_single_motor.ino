#include <driver/twai.h> 

#define MOTOR_ID 2
#define SPEED_FRAME_ID (0x200 + MOTOR_ID) 
#define CAN_TX_GPIO 5 
#define CAN_RX_GPIO 4

void setup() { 
  Serial.begin(115200); 
  delay(100); 
  // Initialize CAN 
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_GPIO, 
    (gpio_num_t)CAN_RX_GPIO,
    TWAI_MODE_NORMAL
    ); 
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config); 
  twai_start(); 
  
  // Enable motor 
  twai_message_t enable_msg; 
  enable_msg.identifier = SPEED_FRAME_ID; 
  enable_msg.extd = 0; 
  enable_msg.data_length_code = 8; 
  for (int i = 0; i < 7; i++) enable_msg.data[i] = 0xFF;
  enable_msg.data[7] = 0xFC;

  twai_transmit(&enable_msg, pdMS_TO_TICKS(100)); 
}

void loop() { 
  // Send speed command 
  twai_message_t speed_msg; 
  speed_msg.identifier = SPEED_FRAME_ID; 
  speed_msg.extd = 0; 
  speed_msg.data_length_code = 4; 
  float speed = 5.0f; 

  memcpy(speed_msg.data, &speed, 4); 
  twai_transmit(&speed_msg, pdMS_TO_TICKS(100)); 

  delay(100); // Send every 100ms 
}
