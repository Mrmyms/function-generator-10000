#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(0, INPUT_PULLUP); // D0 (PB15)
  pinMode(1, INPUT_PULLUP); // D1 (PB14)
}

void loop() {
  int d0 = digitalRead(0);
  int d1 = digitalRead(1);
  Serial.printf("Lectura directa: D0=%d, D1=%d\n", d0, d1);
  delay(250);
}
