#define BAUD 9600
#define READY Serial.println(">")


void setup() {
  Serial.begin(BAUD);
  while (Serial.available() == 0) {
    delay(10);
  }
  READY;
}

void loop() {
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print("\n");
  delay(1000);
}
