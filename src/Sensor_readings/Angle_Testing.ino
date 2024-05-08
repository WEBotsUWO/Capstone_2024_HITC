#include <Wire.h>
#include <AS5600.h>

AS5600 as5600(&Wire1);
AS5600 as5600_2(&Wire);
#define SDA 21
#define SCL 22
#define SDA_2 18
#define SCL_2 19

int angle_in = 0;
int angle_in2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA, SCL);
  Wire1.setPins(SDA_2, SCL_2);
  Wire.begin();
  Wire1.begin();

  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");
  delay(1000);
  
  as5600_2.begin();  //  set direction pin.
  as5600_2.setDirection(AS5600_COUNTERCLOCK_WISE);
  Serial.println("Connect device 2: ");
  Serial.println(as5600_2.isConnected() ? "true" : "false");
  delay(1000);

}

void loop() {
  Serial.print("Sensor 1: ");
  Serial.print(as5600.readAngle());
  Serial.print(", Sensor 2: ");
  Serial.println(as5600_2.readAngle());
  delay(10);

}
