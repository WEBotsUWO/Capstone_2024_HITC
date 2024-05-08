const int stepPin = 33;           
const int dirPin = 25;

void setup() {
  // put your setup code here, to run once:
  pinMode (stepPin, OUTPUT);
  pinMode (dirPin, OUTPUT);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dirPin, LOW); // enables motor to move in one direction
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(5500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(5500);

    Serial.println("hello2");
  
}
