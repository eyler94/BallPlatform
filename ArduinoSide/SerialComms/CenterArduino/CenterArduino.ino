#include <Servo.h>

Servo Pitch;
Servo Roll;

float theta = 1575;
float phi = 1415;

void setup() {
  Serial.begin(19200); // opens serial port, sets data rate to 9600 bps
//  pinMode(LED_BUILTIN, OUTPUT); 
  Pitch.attach(3);
  Roll.attach(5);
  delay(1000);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    String incomingstring = Serial.readStringUntil('\n');
//    incomingstring.remove(0,1);
//    incomingstring.remove(incomingstring.length()-1,1);
    Serial.println(incomingstring);
    theta = getValue(incomingstring,',',0).toFloat();
//    Serial.print(theta);
    phi = getValue(incomingstring,',',1).toFloat();
  }
  Pitch.writeMicroseconds(theta);
  Roll.writeMicroseconds(phi);
}


String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
