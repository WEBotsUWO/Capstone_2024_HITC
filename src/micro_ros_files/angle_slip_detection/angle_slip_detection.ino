#include <Wire.h>
#include <AS5600.h>

AS5600 as5600(&Wire1);
AS5600 as5600_2(&Wire);
#define SDA 21
#define SCL 22
#define SDA_2 18
#define SCL_2 19

float home1 = 0;
float home2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA, SCL);
  Wire1.setPins(SDA_2, SCL_2);
  Wire.begin();
  Wire1.begin();

  as5600.begin(); 
  as5600.setDirection(AS5600_CLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");
  delay(1000);
  
  as5600_2.begin();  
  as5600_2.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.println("Connect device 2: ");
  Serial.println(as5600_2.isConnected() ? "true" : "false");
  delay(1000);

  home1 = 0.0; //as5600.readAngle();
  home2 = 0.0; //as5600_2.readAngle();
  

}

void loop() {
 
  float angle1 = as5600.readAngle()-home1;
  float angle2 = as5600_2.readAngle() - home2;
  Serial.print("Sensor 1: "); 
  Serial.print(angle1*AS5600_RAW_TO_DEGREES);
  Serial.print(", Sensor 2: ");
  Serial.print(angle2*AS5600_RAW_TO_DEGREES);


  Serial.print(" Sensor 1 offset: "); 
  Serial.print(as5600.getOffset());
  Serial.print(", Sensor 2 offset: ");
  Serial.print(as5600_2.getOffset());
  float diff = (angle1 - angle2)*AS5600_RAW_TO_DEGREES;
  Serial.print(", Difference : ");
  Serial.println(diff);

  if(fabs(diff) > 20){
    Serial.println("Slip detected!!!");
  }
  delay(10);

}
