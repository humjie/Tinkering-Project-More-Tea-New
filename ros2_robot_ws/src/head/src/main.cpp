
/**
 * Combined Robot Control
 * 
 * This program controls both:
 * 1. Servo motors for head movement
 * 2. OLED displays for eyes expressions
 * 
 * Hardware:
 * - ESP32 Dev Board
 * - 2x Servo motors (X and Y directions)
 * - 2x SSD1309 OLED Displays (128x64)
 * 
 * ROS2 Communication:
 * - Subscribes to "/face_direction_x" and "/face_direction_y" for servo control
 * - Subscribes to "eye_expression" for eye expressions
 * - Publishes status messages
 */

#include <ESP32Servo.h>
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <U8g2lib.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>  // String message for servo control
#include <std_msgs/msg/int32.h>   // Int32 message for eye expressions

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Error checking macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//==============================================
// Servo Configuration
//==============================================
// Servo objects and pins
Servo myServoX;  // Create a Servo object for the 'x' motor
Servo myServoY;  // Create a Servo object for the 'y' motor

int servoPinX = 18;  // Pin to which the 'x' servo is connected
int servoPinY = 19;  // Pin to which the 'y' servo is connected

//==============================================
// OLED Display Configuration 
//==============================================
// Define I2C pins
#define SDA_LEFT 5   // Hardware I2C
#define SCL_LEFT 4   // Hardware I2C
#define SDA_RIGHT 21 // Software I2C
#define SCL_RIGHT 22 // Software I2C

// Initialize left display with Hardware I2C
U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2_left(U8G2_R0, U8X8_PIN_NONE);

// Initialize right display with Software I2C
U8G2_SSD1309_128X64_NONAME2_F_SW_I2C u8g2_right(U8G2_R0, SCL_RIGHT, SDA_RIGHT, U8X8_PIN_NONE);

// Animation control variables
int normal_eye_frame = 0;
const int total_normal_eye_frames = 8;
int current_expression = 0;
int normal_eye_y_offset = 0;
int frame_counter = 0;

//==============================================
// ROS entities
//==============================================
// Publisher
rcl_publisher_t status_publisher;

// Subscribers
rcl_subscription_t servo_x_subscription;
rcl_subscription_t servo_y_subscription;
rcl_subscription_t eyes_subscription;

// Timer
rcl_timer_t timer;

// ROS messages
std_msgs__msg__String msg_x;        // For receiving X direction commands
std_msgs__msg__String msg_y;        // For receiving Y direction commands
std_msgs__msg__String msg_status;   // For publishing status
std_msgs__msg__Int32 msg_eyes;      // For receiving eye expression commands

// ROS support objects
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Timer period
const int timer_timeout = 1000; // ms

//==============================================
// Eye expressions bitmap data
//==============================================
// Include all the bitmap data for eye expressions
#include "eye_bitmaps.h"

/*
 * Expression IDs:
 * 0 = normal eye (animated)
 * 1 = happy
 * 2 = sad
 * 3 = angry
 * 4 = confused
 * 5 = shocked
 * 6 = love
 * 7 = shy
*/

// Normal eye frames array
const unsigned char* normal_eye_frames[] = {
    normal_eye_1,
    //normal_eye_2,
    normal_eye_3,
    //normal_eye_4,
    normal_eye_5,
    //normal_eye_6,
    normal_eye_7,
    //normal_eye_8,
    //normal_eye_9
};

//==============================================
// Function declarations
//==============================================
void error_loop();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void servo_x_callback(const void * msgin);
void servo_y_callback(const void * msgin);
void eyes_callback(const void * msgin);
void displayLeftEyeExpression(int expression);
void displayRightEyeExpression(int expression);
void displayBothEyesExpression(int expression);

//==============================================
// Error handling function
//==============================================
void error_loop() {
  while(1) {
    Serial.println("Error detected, restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }
}

//==============================================
// Timer callback function - publishes status periodically
//==============================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;
  
  msg_status.data = current_expression;

  RCSOFTCHECK(rcl_publish(&status_publisher, &msg_status, NULL));
  Serial.print("Published status message: ");
  Serial.println(msg->data.data);
}

//==============================================
// Callback for X direction servo messages
//==============================================
void servo_x_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  Serial.print("X callback triggered with message: ");
  Serial.println(msg->data.data);

  if (strcmp(msg->data.data, "+") == 0) {  // Compare the message data with "+"
    // Increase servoX angle (make sure it doesn't exceed 180)
    int currentAngleX = myServoX.read();
    if (currentAngleX < 170) {
      Serial.print("Current X angle: ");
      Serial.println(currentAngleX);
      myServoX.write(currentAngleX + 10);
    }
  } 
  else if (strcmp(msg->data.data, "-") == 0) {  // Compare the message data with "-"
    // Decrease servoX angle
    int currentAngleX = myServoX.read();
    if (currentAngleX > 70) {
      Serial.print("Current X angle: ");
      Serial.println(currentAngleX);
      myServoX.write(currentAngleX - 10);
    }
  }
}

//==============================================
// Callback for Y direction servo messages
//==============================================
void servo_y_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  Serial.print("Y callback triggered with message: ");
  Serial.println(msg->data.data);

  if (strcmp(msg->data.data, "+") == 0) {  // Compare the message data with "+"
    // Increase servoY angle (make sure it doesn't exceed 180)
    int currentAngleY = myServoY.read();
    if (currentAngleY < 170) {
      Serial.print("Current Y angle: ");
      Serial.println(currentAngleY);
      myServoY.write(currentAngleY + 3);
    }
  } 
  else if (strcmp(msg->data.data, "-") == 0) {  // Compare the message data with "-"
    // Decrease servoY angle (make sure it doesn't go below 0)
    int currentAngleY = myServoY.read();
    if (currentAngleY > 50) {
      Serial.print("Current Y angle: ");
      Serial.println(currentAngleY);
      myServoY.write(currentAngleY - 3);
    }
  }
}

//==============================================
// Callback for eye expression messages
//==============================================
void eyes_callback(const void *msg_in) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msg_in;
    int expression = msg->data;
    
    Serial.print("Eye expression changed to: ");
    Serial.println(current_expression);
}

//==============================================
// Display eye expressions functions
//==============================================

/**
 * Display expression on the left eye
 * @param expression The expression ID to display
 */
void displayLeftEyeExpression(int expression) {
    u8g2_left.clearBuffer();
    u8g2_left.setFontMode(1);
    u8g2_left.setBitmapMode(1);

    switch (expression) {
        case 0: // normal
            u8g2_left.drawXBM(33, normal_eye_y_offset, 62, 61, normal_eye_frames[0]);
            break;
        case 1: // happy
            u8g2_left.drawXBM(30, 6 + normal_eye_y_offset, 68, 52, happy_left);
            break;
        case 2: // sad
            u8g2_left.drawXBM(42, 5 + normal_eye_y_offset, 44, 54, sad_left);
            break;
        case 3: // angry
            u8g2_left.drawXBM(32, 0 + normal_eye_y_offset, 64, 64, angry_left);
            break;        
        case 4: // confused
            u8g2_left.drawXBM(38, 7 + normal_eye_y_offset, 53, 51, confused_left);
            break;
        case 5: // shocked
            u8g2_left.drawXBM(40, 8 + normal_eye_y_offset, 48, 49, shocked_left);
            break;
        case 6: // love
            u8g2_left.drawXBM(28, 0 + normal_eye_y_offset, 73, 64, love);
            break;
        case 7: // shy
            u8g2_left.drawXBM(27, 6 + normal_eye_y_offset, 74, 52, shy_left);
            break;
    }
    u8g2_left.sendBuffer();
}

/**
 * Display expression on the right eye
 * @param expression The expression ID to display
 */
void displayRightEyeExpression(int expression) {
    u8g2_right.clearBuffer();
    u8g2_right.setFontMode(1);
    u8g2_right.setBitmapMode(1);

    switch (expression) {
        case 0: // normal
            u8g2_right.drawXBM(33, normal_eye_y_offset, 62, 61, normal_eye_frames[0]);
            break;
        case 1: // happy
            u8g2_right.drawXBM(30, 6 + normal_eye_y_offset, 68, 52, happy_right);
            break;
        case 2: // sad
            u8g2_right.drawXBM(42, 5 + normal_eye_y_offset, 44, 54, sad_right);
            break;
        case 3: // angry
            u8g2_right.drawXBM(32, 0 + normal_eye_y_offset, 64, 64, angry_right);
            break;
        case 4: // confused
            u8g2_right.drawXBM(38, 7 + normal_eye_y_offset, 53, 51, confused_right);
            break;
        case 5: // shocked
            u8g2_right.drawXBM(40, 8 + normal_eye_y_offset, 48, 49, shocked_right);
            break;
        case 6: // love
            u8g2_right.drawXBM(28, 0 + normal_eye_y_offset, 73, 64, love);
            break;
        case 7: // shy
            u8g2_right.drawXBM(27, 6 + normal_eye_y_offset, 74, 52, shy_right);
            break;
    }
    u8g2_right.sendBuffer();
}

/**
 * Display the same expression on both eyes
 * @param expression The expression ID to display
 */
void displayBothEyesExpression(int expression) {
    displayLeftEyeExpression(expression);
    displayRightEyeExpression(expression);
    current_expression = expression;
}

//==============================================
// Setup
//==============================================
void setup() {
  // Configure serial transport
  Serial.begin(115200);
  
  // Initialize servos
  myServoX.attach(servoPinX);
  myServoY.attach(servoPinY);
  
  // Set initial servo positions
  myServoX.write(100);
  myServoY.write(100);
  
  Serial.println("Servos initialized!");
  
  // Initialize I2C for OLED displays
  Wire.begin(SDA_LEFT, SCL_LEFT);
  randomSeed(analogRead(0));

  // Initialize displays
  u8g2_left.begin();
  u8g2_right.begin();
  
  // Show startup expression
  displayBothEyesExpression(0);
  
  Serial.println("OLED displays initialized!");
  
  // Setup micro-ROS
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize memory for messages
  msg_x.data.capacity = 100;
  msg_x.data.size = 0;
  msg_x.data.data = (char *) malloc(msg_x.data.capacity * sizeof(char));

  msg_y.data.capacity = 100;
  msg_y.data.size = 0;
  msg_y.data.data = (char *) malloc(msg_y.data.capacity * sizeof(char));
  
  msg_status.data.capacity = 100;
  msg_status.data.size = 0;
  msg_status.data.data = (char *) malloc(msg_status.data.capacity * sizeof(char));
  snprintf(msg_status.data.data, msg_status.data.capacity, "Combined robot controller running");
  msg_status.data.size = strlen(msg_status.data.data);

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "combined_robot_controller", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "combined_robot_status"));

  // Create subscriptions for servo control
  RCCHECK(rclc_subscription_init_default(
    &servo_x_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/face_direction_x"));
    
  RCCHECK(rclc_subscription_init_default(
    &servo_y_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/face_direction_y"));
    
  // Create subscription for eye expressions
  RCCHECK(rclc_subscription_init_default(
    &eyes_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "eye_expression"));

  // Create timer
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor with capacity for 4 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  
  // Add entities to executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_x_subscription, &msg_x, &servo_x_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_y_subscription, &msg_y, &servo_y_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &eyes_subscription, &msg_eyes, &eyes_callback, ON_NEW_DATA));
  
  Serial.println("MicroROS initialized successfully");
}

//==============================================
// Main loop
//==============================================
void loop() {
  // Process ROS messages
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
