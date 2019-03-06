void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int n = 0; n < 1024; n++){
    Serial.println(n, DEC);
    delay(10);
  }
}
