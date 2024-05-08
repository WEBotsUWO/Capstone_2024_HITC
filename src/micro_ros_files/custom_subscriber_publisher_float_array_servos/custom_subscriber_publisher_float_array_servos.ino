#include <micro_ros_arduino.h>

#include <ESP32Servo.h>

#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>


rcl_subscription_t subscriber_renamed;
rcl_publisher_t publisher_renamed;
std_msgs__msg__Float32MultiArray laser_joints;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

Servo servo1;
Servo servo2;
int servo1Pin = 12;
int servo2Pin = 14;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Subscriber Callback
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  digitalWrite(LED_PIN, (msg->data.size > 0) ? LOW : HIGH); 
//   
  float laser_joints[2] = {msg->data.data[0],msg->data.data[1]} ;
//  memcpy(test_msg.data.data,initialization,sizeof(initialization));
//  test_msg.data.size = 2;

    servo1.write(laser_joints[0]);
    servo2.write(laser_joints[1]);
//  RCSOFTCHECK(rcl_publish(&publisher_renamed, &test_msg, NULL));
}


void initializeFloatMsg(std_msgs__msg__Float32MultiArray (&msg), int givenCapacity)
{
  
  msg.data.capacity = givenCapacity; 
  msg.data.size = 0;
  msg.data.data = (float*) malloc(msg.data.capacity * sizeof(float));
  
  msg.layout.dim.capacity = givenCapacity;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  
  for(size_t i = 0; i < msg.layout.dim.capacity; i++){
    msg.layout.dim.data[i].label.capacity = givenCapacity;
    msg.layout.dim.data[i].label.size = 0;
    msg.layout.dim.data[i].label.data = (char*) malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  servo1.attach(servo1Pin); 
  servo2.attach(servo2Pin);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "student_trajectory"));  
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_node_publisher"));


  initializeFloatMsg(laser_joints, 2);
  initializeFloatMsg(msg, 2);

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_renamed, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(1)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(1)));
}
