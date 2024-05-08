#include <Wire.h> //This is for i2C
#include <AS5600.h>

AS5600 as5600(&Wire);
#define SDA 21
#define SCL 22

AS5600 as5600_2(&Wire1);
#define SDA_2 18
#define SCL_2 19


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
float pT[2] = {0, 0}; // previousTime, for calculating delta t
float pE[2] = {0, 0}; //previousError for calculating the derivative (edot)
float eI[2] = {0,0}; // errorIntegral
float cT[2] = {0,0}; //currentTime, time in the moment of calculation
float dT[2] = {0,0}; //deltaTime, time difference
float eV[2] = {0,0}; //errorValue, error
float ed[2] = {0,0}; // edot, derivative (de/dt)

float globalAngles[2] = {0.0, 0.0};

void setup()
{
  Serial.begin(115200);

//--------------------Joint 1----------------------------------------------------//  
  Wire.begin();           //start i2C
  Wire.setClock(800000L); // faster clock for the encoder (so that we can read from it faster).
  Wire.setPins(SDA, SCL);

  Wire1.begin();
  Wire1.setClock(800000L);
  Wire1.setPins(SDA_2, SCL_2);
  
  //Motor encoder-related ( for one joint)
  as5600.begin();  //  start as5600 encoder.
  as5600.setDirection(AS5600_CLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");
  delay(1000);

//  as5600_2.begin();  //  set direction pin.
//  as5600_2.setDirection(AS5600_COUNTERCLOCK_WISE);
//  Serial.println("Connect device 2: ");
//  Serial.println(as5600_2.isConnected() ? "true" : "false");
//  delay(1000);

  stepper1.setMaxSpeed(32000); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(6000); //100 steps per second square.

//------------------------------------------------------------------------------//

//--------------------Joint 2 via can bus---------------------------------------//  


 
//------------------------------------------------------------------------------//
}

void loop()
{
  float angles[2] = {200.0, 0};
  move_motors(angles);
}

void move_motors(float angles[2])
{
  // set up current target.
  float j1Target = angles[0];
//  float j2Target = angles[1];

Serial.print("Target : ");
Serial.print(j1Target);

  // get encoder reading for both joints
  float j1CurrPosition = as5600.readAngle()*0.08789;
//  float j2CurrPosition = as5600.readAngle()*0.08789; // to be changed

Serial.print(" Position : ");
Serial.print(j1CurrPosition);
  // PID algorithm j1
  float control1 = calculatePID(j1CurrPosition,j1Target,1);
  // PID algorithm j2
//  float control2 = calculatePID(j2CurrPosition, j2Target)

  Serial.println();
  driveMotor(stepper1, control1); //changes to use Accel stepper library
//  driveMotor(stepper2, control2);
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
  
  Serial.print(" Control Signal: ");
  Serial.print(controlSignal);
  return controlSignal;
}
  
void driveMotor(AccelStepper stepper, float controlSignal)
{
  if(fabs(controlSignal) > 5) // currently set to within +/- 5 degrees per step
  {
    stepper.setSpeed(controlSignal*20);
//    stepper.setAcceleration(3000); // 1/32 steps
  }
  else
  {
    stepper.setSpeed(0);
    stepper.stop();
  }

   stepper.runSpeed();
}
