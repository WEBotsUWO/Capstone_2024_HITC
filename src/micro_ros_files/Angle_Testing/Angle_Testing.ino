#include <Wire.h>
#include <AS5600.h>

AS5600 as5600(&Wire);
AS5600 as5600_2(&Wire1);
#define SDA 21
#define SCL 22
#define SDA_2 18
#define SCL_2 19
//#define RXD2 16
//#define TXD2 17
//
//int angle_in = 0;
//int angle_in2 = 0;

void setup() {
  Serial.begin(115200);
//  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Wire.setPins(SDA, SCL);
  Wire1.setPins(SDA_2, SCL_2);
  Wire.begin();
  Wire1.begin();
//
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

}

void loop() {
  Serial.print("Sensor 1: ");
  Serial.println(as5600.readAngle());
  Serial.print("\n");
//  Serial.println(as5600.readAngle());
  Serial.print(", Sensor 2: ");
  Serial.println(as5600_2.readAngle());
  delay(10);

}
