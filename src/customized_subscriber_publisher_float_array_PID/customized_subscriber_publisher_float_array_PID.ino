#include <micro_ros_arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h> // no longer needed.

#include <Wire.h> //This is for i2C
#include <AS5600.h>
#include <AccelStepper.h>

#define SDA 21
#define SCL 22


#define stepPin 33 
#define dirPin 25


#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}



rcl_subscription_t subscriber_renamed;
rcl_publisher_t publisher_renamed;
std_msgs__msg__Float32MultiArray test_msg;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


AccelStepper stepper1(1, stepPin, dirPin);

//PID parameters - tuned by the user
float proportional = 1.35; //k_pp
float integral = 0.00005; //k_i 
float derivative = 0.01; //k_d 
float controlSignal = 0; //u 
//-----------------------------------
//-----------------------------------


void error_loop(){ // debugging error loop, not control systems loop
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
  float initialization[2] = {msg->data.data[0],msg->data.data[1]} ;
  memcpy(test_msg.data.data,initialization,sizeof(initialization));
  test_msg.data.size = 2;

// After receiving the trajectory from the student the motor must follow the current joint angle.
// after the motor has moved the encoder readings will be sent back to the simulation 

  //PID-related
float givenPosition = 0; // initial target is zero ( this will be updated from the students trajectory)
float previousTime = 0; //for calculating delta t
float previousError = 0; //for calculating the derivative (edot)
float errorIntegral = 0; //integral error
float currentTime = 0; //time in the moment of calculation
float deltaTime = 0; //time difference
float errorValue = 0; //error
float edot = 0; //derivative (de/dt)
float motorPosition = 0;

  // get new target from student's trajectory.
  joint1Target = msg->data.data[0];
  joint2Target = msg->data.data[1]; // needs to be sent to other microcontroller.

  // get encoder position before controller
  motorPosition = as5600.readAngle()*0.08789; // *(360/4096);
//  motorPosition2 = as5600.readAngle()*0.08789; // *(360/4096); for second encoder
  
  calculatePID();
  driveMotor(); //changes to use Accel stepper library

  stepper1.runSpeed(); // from accel stepper library.



// assign value t float array msg 
  RCSOFTCHECK(rcl_publish(&publisher_renamed, &test_msg, NULL));
}



void setup() {
  set_microros_transports();

  Wire.begin();           //start i2C
  Wire.setClock(800000L); // faster clock for the encoder (so that we can read from it faster).
  Wire.setPins(SDA, SCL);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  //start as5600 encoder 
  as5600.begin();  //  start as5600 encoder.
  as5600.setDirection(AS5600_CLOCK_WISE);
//  Serial.print("Connect device 1: ");
//  Serial.println(as5600.isConnected() ? "true" : "false");
//  delay(1000);
  stepper1.setMaxSpeed(200); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(100); //100 steps per second square.

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
    "trajectory_for_esp32"));  
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_node_publisher"));


  // Initialize float Array
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

  // For the subscriber message we still need to initialize it before we hand it over to the executor. We don't need to assigna a value but it must be initialized.

  msg.data.capacity = 3; 
  msg.data.size = 0;
  msg.data.data = (float*) malloc(msg.data.capacity * sizeof(float));
  
  msg.layout.dim.capacity = 3;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  
  for(size_t i = 0; i < msg.layout.dim.capacity; i++){
    msg.layout.dim.data[i].label.capacity = 3;
    msg.layout.dim.data[i].label.size = 0;
    msg.layout.dim.data[i].label.data = (char*) malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_renamed, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}


//void checkCurrTarget()
//{
// givenPosition = 80.0; // target will just be 30 degreees for now.
//} 
//
//void checkEncoder()
//{
//  //We need to read the other pin of the encoder which will be either 1 or 0 depending on the direction
//  motorPosition = as5600.readAngle()*0.08789; // *(360/4096);
//}


void calculatePID(float motorPosition, float givenPosition, unsigned long currentTime, unsigned long previousTime)
{
  //Determining the elapsed time
//  currentTime = micros(); //current time
  deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  //---
  errorValue = motorPosition - givenPosition; //Current position - target position (or setpoint)

  edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term

  errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

  previousError = errorValue; //save the error for the next iteration to get the difference (for edot)
}

void driveMotor()
{
//  stepper1.enableOutputs(); // not sure what this is for.
  if(fabs(controlSignal) > 5) // must be atleast within 5 degrees per step // but the magnet disk needs resolution of 1 degree... to be fixed!
  {
    stepper1.setSpeed(controlSignal*20);
  }
  else
  {
    stepper1.setSpeed(0);
    stepper1.stop();
  }
}
