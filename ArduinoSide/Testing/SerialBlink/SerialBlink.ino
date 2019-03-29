String incomingstring = ""; // for incoming serial data
int Delay_time = 1000;

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingstring = Serial.readString();
    delay(incomingstring.toInt());
  }
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(incomingstring.toInt());                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(incomingstring.toInt());    
}
