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
  static int lvl = LOW;
  char c;
  if (Serial.available() > 0) {
    digitalWrite(13, lvl);
    lvl = !lvl;
    c = Serial.read();
    Serial.print("-> ");
    Serial.println(c);
    Serial.print("Time: ");
    Serial.println(millis());
    //Serial1.print("\n");
  }
  delay(1000);
}
