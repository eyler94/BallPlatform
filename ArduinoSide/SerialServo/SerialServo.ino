#include <Servo.h>

String incomingstring = ""; // for incoming serial data
String Theta = "90";
String Phi = "90";

Servo Pitch;
Servo Roll;

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
  Pitch.attach(3);
  Roll.attach(5);  
  Pitch.write(90);
  Roll.write(90);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingstring = Serial.readString();
    Theta = incomingstring.substring(0,4);
    Phi = incomingstring.substring(4);
    Pitch.writeMicroseconds(Theta.toInt());
    Roll.writeMicroseconds(Phi.toInt());
  }
}
