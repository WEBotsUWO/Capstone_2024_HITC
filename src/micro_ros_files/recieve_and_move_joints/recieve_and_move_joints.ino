#include <Wire.h>
#include <AS5600.h>


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>


rcl_subscription_t subscriber_renamed;
rcl_publisher_t current_encoder_readings;
rcl_publisher_t received_joint_angles;


std_msgs__msg__Float32MultiArray laser_angles;
std_msgs__msg__Float32MultiArray joint_angles;
std_msgs__msg__Float32MultiArray sensor_readings;
std_msgs__msg__Float32MultiArray msg;

rclc_executor_t execute_current_encoder_readings;
rclc_executor_t execute_received_joint_angles;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
AS5600 as5600(&Wire);
#define SDA 21
#define SCL 22

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#include <AccelStepper.h>

#define stepPin 33
#define dirPin 25


AccelStepper stepper1(1,stepPin, dirPin);


//PID parameters - tuned by the user
float proportional = 1.35; //k_pp
float integral = 0.00005; //k_i 
float derivative = 0.01; //k_d 
float controlSignal = 0; //u 
//-----------------------------------
//-----------------------------------
//PID-related
float pT[2] = {0,0}; // previousTime, for calculating delta t
float pE[2] = {0,0}; //previousError for calculating the derivative (edot)
float eI[2] = {0,0}; // errorIntegral
float cT[2] = {0,0}; //currentTime, time in the moment of calculation
float dT[2] = {0,0}; //deltaTime, time difference
float eV[2] = {0,0}; //errorValue, error
float ed[2] = {0,0}; // edot, derivative (de/dt)
volatile float encoderAngles[2] = {0.0, 0.0};
volatile float globalAngles[2] = {0.0, 0.0};

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
//    move_motors(globalAngles, encoderAngles);
//    RCSOFTCHECK(rcl_publish(&current_encoder_readings, &received_joint_angles, NULL));
  }
}

// Subscriber Callback
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  digitalWrite(LED_PIN, (msg->data.size > 0) ? LOW : HIGH); 
//   
  float angles_radian[2] = {msg->data.data[0],msg->data.data[1]} ; // received angles as radian from main program , convert to degrees.
  float angles_degree[2] = {msg->data.data[0]*57.2958,msg->data.data[1]*57.2958} ;
  // update target for motors.
  globalAngles[0] = angles_degree[0];
  globalAngles[1] = angles_degree[1];

  //lets publish the received angles (for confirmation)
  memcpy(joint_angles.data.data,angles_degree,sizeof(angles_degree));
  joint_angles.data.size = 2;  
  RCSOFTCHECK(rcl_publish(&received_joint_angles, &joint_angles, NULL)); 
  
  move_motors(globalAngles, encoderAngles);
  // let's publish encoder readings.
//  std_msgs__msg__Float32MultiArray sensor_readings;
  float encoderAnglesRadian[2] = {encoderAngles[0]*0.0174533, encoderAngles[1]*0.0174533} ;
  memcpy(sensor_readings.data.data,encoderAnglesRadian,sizeof(encoderAnglesRadian));
  sensor_readings.data.size = 2;
  RCSOFTCHECK(rcl_publish(&current_encoder_readings, &sensor_readings, NULL));
}

void move_motors(volatile float angles[2], volatile float (&currPosition)[2])
{
  float j1Target = angles[0];  // set up current target.
  
  float j1CurrPosition = as5600.readAngle()*0.08789; // current encoder reading
  
  float control1 = calculatePID(j1CurrPosition,j1Target,1); // PID algorithm j for joint
  
  driveMotor(stepper1, control1); //move motor using Accel stepper library
  
  //---------------------Code for second joint (via canbus)-------------------------//

  float j2Target = angles[1];  // set up current target.
  
  float j2CurrPosition = 0.0;
  
//  float control2 = calculatePID(j1CurrPosition,j1Target,1); // PID algorithm j for joint
//  
//  driveMotor(stepper2, control2); //move motor using Accel stepper library
  
  //-------------------------------------------------------------------------------//
  currPosition[0] = j1CurrPosition;
  currPosition[1] = j2CurrPosition;
}

float calculatePID(float currPosition, float Target, int jointNum)
{
  cT[jointNum-1] = micros();//current time
  dT[jointNum-1] = (cT[jointNum-1] - pT[jointNum-1]) / 1000000.0; //time difference in seconds
  pT[jointNum-1] = cT[jointNum-1]; //save the current time for the next iteration to get the time difference
  eV[jointNum-1] = currPosition - Target; //Current position - target position (or setpoint)
  ed[jointNum-1] = (eV[jointNum-1] - pE[jointNum-1]) / dT[jointNum-1]; //edot = de/dt - derivative term
  eI[jointNum-1] = eI[jointNum-1] + (eV[jointNum-1] * dT[jointNum-1]); //integral term - Newton-Leibniz, notice, this is a running sum!6
  controlSignal = (proportional * eV[jointNum-1]) + (derivative * ed[jointNum-1]) + (integral * eI[jointNum-1]); //final sum, proportional term also calculated here
  pE[jointNum-1] = eV[jointNum-1]; //save the error for the next iteration to get the difference (for edot)
 
  return controlSignal;
}

void driveMotor(AccelStepper stepper, float controlSignal)
{
  if(fabs(controlSignal) > 5) // currently set to within +/- 5 degrees per step
  {
    stepper.setSpeed(controlSignal*20);
  }
  else
  {
    stepper.setSpeed(0);
    stepper.stop();
  }

   stepper.runSpeed();
}

void initializeFloatMsgArray(std_msgs__msg__Float32MultiArray (&msg), int givenCapacity)
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

  Wire.begin();           //start i2C
  Wire.setClock(800000L); // faster clock for the encoder (so that we can read from it faster).
  Wire.setPins(SDA, SCL);
   
  as5600.begin(); 
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");


  
  stepper1.setMaxSpeed(200); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(100); //100 steps per second square.
  
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
  
  // create publisher for current encoder readings
  RCCHECK(rclc_publisher_init_default(
    &current_encoder_readings,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_readings"));

    // create for received joint angles
  RCCHECK(rclc_publisher_init_default(
    &received_joint_angles,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "esp_echo_received_angles"));

  // create timer, (control loop)
  const unsigned int timer_timeout = 10; // every 10 milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    control_timer_callback));


  initializeFloatMsgArray(joint_angles, 2);
  initializeFloatMsgArray(msg, 2);
  initializeFloatMsgArray(sensor_readings, 2);

  // executor for subscribers
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_renamed, &msg, &subscription_callback, ON_NEW_DATA));

  // executor for publishers
  RCCHECK(rclc_executor_init(&execute_current_encoder_readings, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&execute_received_joint_angles, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&execute_received_joint_angles, &timer));

}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&execute_current_encoder_readings, RCL_MS_TO_NS(10)));
  RCSOFTCHECK(rclc_executor_spin_some(&execute_received_joint_angles, RCL_MS_TO_NS(10)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10)));
}
