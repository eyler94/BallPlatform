void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int n = 0; n < 1024; n++){
    Serial.println(n, DEC);
    delay(50);
  }
}
