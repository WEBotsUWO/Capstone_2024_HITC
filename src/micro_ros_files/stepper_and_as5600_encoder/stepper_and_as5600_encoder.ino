#include <Wire.h>
#include <AS5600.h>

#include <AccelStepper.h>


AS5600 as5600(&Wire);
#define SDA 21
#define SCL 22


#define stepPin 33
#define dirPin 25


AccelStepper stepper1(1,stepPin, dirPin);

void setup() {
  // put your setup code here, to run once:
  stepper1.setMaxSpeed(200); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(100); //100 steps per second square.
  stepper1.moveTo(200);


  Serial.begin(115200);
  Wire.setPins(SDA, SCL);
  Wire.begin();


  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");
  delay(1000);
  
 
}

void loop() {
  // put your main code here, to run repeatedly:
  stepper1.run();
  Serial.print("Sensor 1: ");
  Serial.println(as5600.readAngle());
  delay(10);

}
