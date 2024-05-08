#include <Wire.h>

#define SDA 21
#define SCL 22
#define SDA_2 18
#define SCL_2 19


//Magnetic sensor things
int magnetStatus[2] = {0,0}; //value of the status register (MD, ML, MH)

int lowbyte[2] = {0,0}; //raw angle 7:0 registers
word highbyte[2] = {0,0}; //raw angle 7:0 and 11:8
int rawAngle[2] = {0,0}; //final raw angle , combines the lowbyte and highbyte registers
float degAngle[2] = {0,0}; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber[2] = {0,0};
int previousquadrantNumber[2] = {0,0}; //quadrant IDs, we will be tracking which quadrant the encoder is in.
float numberofTurns[2] = {0,0}; //number of turns
float correctedAngle[2] = {0,0}; //tared angle - based on the startup value
float startAngle[2] = {0,0}; //starting angle  , for aligning the value of the encoder's angle
float totalAngle[2] = {0,0}; //total absolute angular displacement absolute angle irrespective of the quadrant we're in.
float previoustotalAngle[2] = {0,0}; //for the display printing
float sending_timer;


void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA, SCL);
  Wire1.setPins(SDA_2, SCL_2);
  Wire.begin();
  Wire1.begin();
//
//  Wire.setClock(800000L); //fast clock
//  Wire1.setClock(800000L); //fast clock

  checkMagnetPresence(1); // check sensor 1, check the magnet (blocks until magnet is found)
  checkMagnetPresence(2); // check sensor 2
  
  ReadRawAngle(1); //make a reading so the degAngle gets updated
  ReadRawAngle(2);
  
  startAngle[0] = degAngle[0]; //update startAngle with degAngle - for taring
  startAngle[1] = degAngle[1];

  Serial.print(degAngle[0]);
  Serial.print(", ");
  Serial.println(degAngle[1]);

  //------------------------------------------------------------------------------  

  sending_timer = millis();
}

void loop() {
  ReadRawAngle(1); 
  ReadRawAngle(2);
  correctAngle(1);
  correctAngle(2);

  checkQuadrant(1);
  checkQuadrant(2);
  refreshDisplay();
 }



void checkMagnetPresence(int sensor_index)
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus[sensor_index-1] & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1  (0xb100000)
  {
    magnetStatus[sensor_index-1] = 0; //reset reading

    if(sensor_index-1 == 0){
      Wire.beginTransmission(0x36); //connect to the sensor
      Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
      Wire.endTransmission(); //end transmission
      Wire.requestFrom(0x36, 1); //request from the sensor
  
      while(Wire.available() == 0); //wait until it becomes available 
      magnetStatus[sensor_index-1] = Wire.read(); //Reading the data after the request
    }

    if(sensor_index-1 == 1){
      Wire1.beginTransmission(0x36); //connect to the sensor
      Wire1.write(0x0B); //figure 21 - register map: Status: MD ML MH
      Wire1.endTransmission(); //end transmission
      Wire1.requestFrom(0x36, 1); //request from the sensor
  
      while(Wire1.available() == 0); //wait until it becomes available 
      magnetStatus[sensor_index-1] = Wire1.read(); //Reading the data after the request
    }    
  }       
}

void correctAngle(int sensor_index)
{
  //recalculate angle
  correctedAngle[sensor_index-1] = degAngle[sensor_index-1] - startAngle[sensor_index-1]; //this tares the position

  if(correctedAngle[sensor_index-1] < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle[sensor_index-1] = correctedAngle[sensor_index-1] + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  //Serial.print("Corrected angle: ");
  //Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}


void ReadRawAngle(int sensor_index)
{ 
  if(sensor_index - 1 == 0){
      //7:0 - bits
    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor
    
    while(Wire.available() == 0); //wait until it becomes available 
    lowbyte[sensor_index-1] = Wire.read(); //Reading the data after the request
   
    //11:8 - 4 bits
    Wire.beginTransmission(0x36);
    Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    
    while(Wire.available() == 0);  
    highbyte[sensor_index -1] = Wire.read();
    
    //4 bits have to be shifted to its proper place as we want to build a 12-bit number
    highbyte[sensor_index -1] = highbyte[sensor_index -1] << 8; //shifting to left
    //What is happening here is the following: The variable is being shifted by 8 bits to the left:
    //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
    //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
    
    //Finally, we combine (bitwise OR) the two numbers:
    //High: 00001111|00000000
    //Low:  00000000|00001111
    //      -----------------
    //H|L:  00001111|00001111
    rawAngle[sensor_index -1] = highbyte[sensor_index -1] | lowbyte[sensor_index -1]; //int is 16 bits (as well as the word)
  } 
  else if (sensor_index - 1 == 1){
      //7:0 - bits
    Wire1.beginTransmission(0x36); //connect to the sensor
    Wire1.write(0x0D); //figure 21 - register map: Raw angle (7:0)
    Wire1.endTransmission(); //end transmission
    Wire1.requestFrom(0x36, 1); //request from the sensor
    
    while(Wire1.available() == 0); //wait until it becomes available 
    lowbyte[sensor_index-1] = Wire1.read(); //Reading the data after the request
   
    //11:8 - 4 bits
    Wire1.beginTransmission(0x36);
    Wire1.write(0x0C); //figure 21 - register map: Raw angle (11:8)
    Wire1.endTransmission();
    Wire1.requestFrom(0x36, 1);
    
    while(Wire1.available() == 0);  
    highbyte[sensor_index -1] = Wire1.read();
    
    //4 bits have to be shifted to its proper place as we want to build a 12-bit number
    highbyte[sensor_index -1] = highbyte[sensor_index -1] << 8; //shifting to left
    //What is happening here is the following: The variable is being shifted by 8 bits to the left:
    //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
    //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
    
    //Finally, we combine (bitwise OR) the two numbers:
    //High: 00001111|00000000
    //Low:  00000000|00001111
    //      -----------------
    //H|L:  00001111|00001111
    rawAngle[sensor_index -1] = highbyte[sensor_index -1] | lowbyte[sensor_index -1]; //int is 16 bits (as well as the word)
  }
  

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle[sensor_index-1] = rawAngle[sensor_index -1] * 0.087890625; 
  
  //Serial.print("Deg angle: ");
  //Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle
  
}



void refreshDisplay()
{
  if (millis() - sending_timer > 100) //chech if we will update at every 100 ms
  { 
//    if(totalAngle != previoustotalAngle) //if there's a change in the position*
//    {
        Serial.print(totalAngle[0]); //print the new absolute position
        Serial.print(", "); 
        Serial.print(totalAngle[1]); //print the new absolute position
        Serial.print(", "); 
        Serial.println(abs(totalAngle[0]-totalAngle[1]));
        sending_timer = millis(); //reset timer   
        previoustotalAngle[0] = totalAngle[0]; //update the previous value
        previoustotalAngle[1] = totalAngle[1];
//    }
  }
  else
  {
    //skip
  }
  //*idea: you can define a certain tolerance for the angle so the screen will not flicker
  //when there is a 0.08 change in the angle (sometimes the sensor reads uncertain values)
}


void checkQuadrant(int sensor_index)
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */
  //Quadrant 1
  if(correctedAngle[sensor_index -1] >= 0 && correctedAngle[sensor_index-1] <=90)
  {
    quadrantNumber[sensor_index-1] = 1;
  }

  //Quadrant 2
  if(correctedAngle[sensor_index-1] > 90 && correctedAngle[sensor_index-1] <=180)
  {
    quadrantNumber[sensor_index-1] = 2;
  }

  //Quadrant 3
  if(correctedAngle[sensor_index-1] > 180 && correctedAngle[sensor_index-1] <=270)
  {
    quadrantNumber[sensor_index-1] = 3;
  }

  //Quadrant 4
  if(correctedAngle[sensor_index-1] > 270 && correctedAngle[sensor_index-1] <360)
  {
    quadrantNumber[sensor_index-1] = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber[sensor_index-1] != previousquadrantNumber[sensor_index-1]) //if we changed quadrant
  {
    if(quadrantNumber[sensor_index-1] == 1 && previousquadrantNumber[sensor_index-1] == 4)
    {
      numberofTurns[sensor_index-1]++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber[sensor_index-1] == 4 && previousquadrantNumber[sensor_index-1] == 1)
    {
      numberofTurns[sensor_index-1]--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber[sensor_index-1] = quadrantNumber[sensor_index-1];  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle[sensor_index-1] = (numberofTurns[sensor_index-1]*360) + correctedAngle[sensor_index-1]; //number of turns (+/-) plus the actual angle within the 0-360 range
  //Serial.print("Total angle: ");
  //Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits
}
