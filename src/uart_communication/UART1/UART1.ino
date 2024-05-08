#define RXD2 16
#define TXD2 17

float angle = 8.2;
float angle2 = 0.0;
int complianceMode = 0;
int slipDetect = 0;
char buf[80];


//Reading Messages Function
int readline(int readch, char *buffer, int len) {
//    Serial.println(readch);
    static int pos = 0;
    int rpos;
    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.print("Angle 1: ");
  Serial.println(angle);
  Serial.print("Angle 2: ");
  Serial.println(angle2);
}

void loop() { //Choose Serial1 or Serial2 as required
  if (readline(Serial2.read(), buf, 80) > 0) {
//        Serial.println(buf);
        if (buf == "Slipped"){
          complianceMode = 1;
          Serial.println("Joint 2 triggered compliance mode");
        }
//        else if (buf == "Cleared"){
//          complianceMode = 0;
//        }
        else{
          angle2 = atof(buf);
          Serial.println(angle2);
        }
  }

  //Structure for slip signal
  // if(slipDetected){
  //   Serial2.println("Slipped");
  // }
  // if(slipCleared){
  //   Serial2.println("Cleared");
  // }

  Serial2.print(angle);
  Serial2.print("\n");
}
