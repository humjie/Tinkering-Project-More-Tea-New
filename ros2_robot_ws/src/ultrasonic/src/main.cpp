#include <micro_ros_platformio.h>
#include <stdio.h>
#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pins for Ultrasonic Sensor
const int trigPin = 25;
const int echoPin = 26;
#define SOUND_SPEED 0.034 // cm/us

// Distance Threshold
#define THRESHOLD_CM 10

// ROS 2 Variables
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// Micro-ROS Timer Callback
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // Measure distance
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    float distanceCm = duration * SOUND_SPEED / 2;

    Serial.print("Distance (cm): ");
    Serial.println(distanceCm);

    // Set message data
    msg.data = (distanceCm < THRESHOLD_CM) ? 1 : 0;

    // Publish the message
    rcl_publish(&publisher, &msg, NULL);
  }
}

void setup() {
  // Serial for debugging and micro-ROS transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  Serial.println("Ultrasonic starting...");

  // Ultrasonic pin setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // micro-ROS setup
  allocator = rcl_get_default_allocator();

  // Create init options
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create node
  rclc_node_init_default(&node, "ultrasonic_node", "", &support);

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "ultrasonic_status");

  // Create timer to run at 1Hz
  const unsigned int timer_timeout = 1000; // ms
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback);

  // Create executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
