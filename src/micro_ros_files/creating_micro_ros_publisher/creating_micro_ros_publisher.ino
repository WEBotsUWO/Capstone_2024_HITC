#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;    // variable declarations
//std_msgs__msg__Int32MultiArray * msg;     // message of array type Int32
std_msgs__msg__Int32 msg; 
rclc_executor_t executor;     // still don't know what an executor is
rclc_support_t support;       //Before a node is created, a rclc_support_t object needs to be created and initialized The rclc_support_init method will handle micro-ROS initial configuration (memory initialization, transport configuration, …)
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} // macro for function
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){    // function to handle errors
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // just toggles the LED
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) // timer callback function for timer
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//    msg->data.data[0]++;
//    msg->data.data[1]++;
      
    

      
  }
}

void setup() {
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher")); // micro_ros_arduino_node_publisher is the topic name.

  // create timer,
  const unsigned int timer_timeout = 1000; // every one second the timer calls the timer_callback function.
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // this allows for determinitic excution of tasks.
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

//    msg.std_msgs__msg__Int32MultiArray__Sequence__init; // creating and initializing the array type message.
      int32_t array_initilizer[2] = {1, 0};
      rosidl_runtime_c__int32__Sequence initial_data;
      initial_data.size = 2;
      initial_data.data[0] = 1;
      initial_data.data[3] = 2;
//    msg->data.data = array_initilizer;
      msg.data = 1;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
//  Serial.println("msg contains the value: ");
//  Serial.print(*(msg->data).data);
}
