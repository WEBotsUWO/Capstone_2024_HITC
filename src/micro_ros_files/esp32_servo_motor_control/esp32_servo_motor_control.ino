// using ESP32Servo library by Kevin Harrington and John k. Bennett

#include <ESP32Servo.h>

Servo servo1;
Servo servo2;
int servo1Pin = 12;
int servo2Pin = 14;

void setup() {
  // put your setup code here, to run once:
  servo1.attach(servo1Pin); 
  servo2.attach(servo2Pin);
   
//  servo1.write(0); // angle between zero and 180 degrees
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    int angle = Serial.parseInt();
    servo1.write(angle);
    servo2.write(angle);
}
delay(20);
}
