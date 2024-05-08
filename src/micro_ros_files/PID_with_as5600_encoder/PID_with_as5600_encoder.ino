#include <Wire.h> //This is for i2C

//Input and output pins
// AS5600 encoder

#include <AS5600.h>

AS5600 as5600(&Wire);
#define SDA 21
#define SCL 22


#define stepPin 33 
#define dirPin 25
#include <AccelStepper.h>
AccelStepper stepper1(1, stepPin, dirPin);


//PID parameters - tuned by the user
float proportional = 1.35; //k_pp
float integral = 0.00005; //k_i 
float derivative = 0.01; //k_d 
float controlSignal = 0; //u 
//-----------------------------------
//-----------------------------------
//PID-related
float targetPosition = 0; // initial target is zero ( this will be updated from the students trajectory)
float previousTime = 0; //for calculating delta t
float previousError = 0; //for calculating the derivative (edot)
float errorIntegral = 0; //integral error
float currentTime = 0; //time in the moment of calculation
float deltaTime = 0; //time difference
float errorValue = 0; //error
float edot = 0; //derivative (de/dt)
float motorPosition = 0;


void setup()
{
  Serial.begin(115200);
  Wire.begin();           //start i2C
  Wire.setClock(800000L); // faster clock for the encoder (so that we can read from it faster).
  Wire.setPins(SDA, SCL);

  
  //Motor encoder-related
  as5600.begin();  //  start as5600 encoder.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");
  delay(1000);

  stepper1.setMaxSpeed(200); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(100); //100 steps per second square.
//  stepper1.moveTo(200);

}

void loop()
{
  Serial.print("Current Target: ");
  checkCurrTarget(); // just for setting the target

  Serial.print("Current Pos: ");
  checkEncoder();

  Serial.print(" Control signal: ");
  calculatePID();
  Serial.println();

  driveMotor(); //changes to use Accel stepper library

  stepper1.runSpeed(); // from accel stepper library.

  //printValues();



//  refreshDisplay(); not needed
}
void checkCurrTarget()
{
 targetPosition = 160.0; // target will just be 30 degreees for now.
 Serial.print(targetPosition);
 Serial.print(" ");
} 

void checkEncoder()
{
  //We need to read the other pin of the encoder which will be either 1 or 0 depending on the direction
  motorPosition = as5600.readAngle()*0.08789; // *(360/4096);
  Serial.print(motorPosition);
}


void calculatePID()
{
  //Determining the elapsed time
  currentTime = micros(); //current time
  deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  //---
  errorValue = motorPosition - targetPosition; //Current position - target position (or setpoint)

  edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term

  errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!

  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here

  previousError = errorValue; //save the error for the next iteration to get the difference (for edot)

  Serial.print(controlSignal);

//  Serial.print(currentTime);
}

void driveMotor()
{
//  //Determine speed and direction based on the value of the control signal
//  //direction
//  if (controlSignal < 0) //negative value: CCW
//  {
//    motorDirection = -1;
//  }
//  else if (controlSignal > 0) //positive: CW
//  {
//    motorDirection = 1;
//  }
//  else //0: STOP - this might be a bad practice when you overshoot the setpoint
//  {
//    motorDirection = 0;
//  }

//  stepper1.enableOutputs(); // not sure what this is for.
  if(fabs(controlSignal) > 5) // 1.8 degrees per step
  {
    stepper1.setSpeed(controlSignal*20);
  }
  else
  {
    stepper1.setSpeed(0);
    stepper1.stop();
  }
  
//  
//  //---------------------------------------------------------------------------
//  //Speed
//  PWMValue = (int)fabs(controlSignal); //PWM values cannot be negative and have to be integers
//  if (PWMValue > 255) //fabs() = floating point absolute value
//  {
//    PWMValue = 255; //capping the PWM signal - 8 bit
//  }
//
//  if (PWMValue < 30 && errorValue != 0)
//  {
//    PWMValue = 30;
//  }
//  //A little explanation for the "bottom capping":
//  //Under a certain PWM value, there won't be enough current flowing through the coils of the motor
//  //Therefore, despite the fact that the PWM value is set to the "correct" value, the motor will not move
//  //The above value is an empirical value, it depends on the motors perhaps, but 30 seems to work well in my case
//
//  //we set the direction - this is a user-defined value, adjusted for TB6612FNG driver
//  if (motorDirection == -1) //-1 == CCW
//  {
//    digitalWrite(directionPin1, LOW);
//    digitalWrite(directionPin2, HIGH);
//  }
//  else if (motorDirection == 1) // == 1, CW
//  {
//    digitalWrite(directionPin1, HIGH);
//    digitalWrite(directionPin2, LOW);
//  }
//  else // == 0, stop/break
//  {
//    digitalWrite(directionPin1, LOW);
//    digitalWrite(directionPin2, LOW);
//    digitalWrite(standByPin, LOW);
//    PWMValue = 0;
//    //In this block we also shut down the motor and set the PWM to zero
//  }
//  //----------------------------------------------------
//  //Then we set the motor speed
//  analogWrite(PWMPin, PWMValue);
//
//  //Optional printing on the terminal to check what's up
//  /*
//    Serial.print(errorValue);
//    Serial.print(" ");
//    Serial.print(PWMValue);
//    Serial.print(" ");
//    Serial.print(targetPosition);
//    Serial.print(" ");
//    Serial.print(motorPosition);
//    Serial.println();
//  */

}


void printValues()
{
  //Serial.print("Position: ");
  Serial.println(motorPosition);
}
