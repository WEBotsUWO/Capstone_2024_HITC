#include <micro_ros_arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

rcl_publisher_t publisher_renamed;
std_msgs__msg__Float32MultiArray test_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    RCSOFTCHECK(rcl_publish(&publisher_renamed, &test_msg, NULL));
//    test_msg.data+=0.1;
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  /*
   * 
   * typedef struct std_msgs__msg__Float32MultiArray
{

  std_msgs__msg__MultiArrayLayout layout; // need to initialize this line
 
  rosidl_runtime_c__float__Sequence data; // need to also initialize this line
} std_msgs__msg__Float32MultiArray;
   * 
   * 
   * from primitive_sequences.h
   * #define ROSIDL_RUNTIME_C__PRIMITIVE_SEQUENCE(STRUCT_NAME, TYPE_NAME) \
    typedef struct rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence \
    { \
      TYPE_NAME * data;  // 
      size_t size; 
      size_t capacity; 
    } rosidl_runtime_c__ ## STRUCT_NAME ## __Sequence;

   */
  test_msg.data.capacity = 1;
  test_msg.data.size = 0;
  test_msg.data.data = (float*) malloc(test_msg.data.capacity * sizeof(float));

  test_msg.layout.dim.capacity = 100;
  test_msg.layout.dim.size = 0; // currently nothing in the array
  test_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *) malloc(test_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for(int i = 0; i < test_msg.layout.dim.capacity; i++){
    test_msg.layout.dim.data[i].label.capacity = 20;
    test_msg.layout.dim.data[i].label.size = 0;
    test_msg.layout.dim.data[i].label.data = (char*) malloc(test_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }
  float initialization[2] = {1.01, 2.01};
  memcpy(test_msg.data.data,initialization,sizeof(initialization));
  test_msg.data.size = 2;

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
