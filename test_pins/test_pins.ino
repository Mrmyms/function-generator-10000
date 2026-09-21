#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  for (int i = 0; i <= 15; i++) {
    pinMode(i, INPUT_PULLUP);
  }
  Serial.println("\n--- All Pins D0 to D15 Scanner Running ---");
}

void loop() {
  // Monitorear D0 hasta D15 por 500ms
  int transitions[16] = {0};
  int lastState[16];
  for (int i = 0; i <= 15; i++) lastState[i] = digitalRead(i);

  uint32_t t0 = millis();
  while (millis() - t0 < 600) {
    for (int i = 0; i <= 15; i++) {
      int s = digitalRead(i);
      if (s != lastState[i]) {
        transitions[i]++;
        lastState[i] = s;
      }
    }
  }

  // Imprimir reporte de cambios
  bool anyPulse = false;
  for (int i = 0; i <= 15; i++) {
    if (transitions[i] > 2) {
      Serial.printf(">>> PULSOS DETECTADOS EN PIN D%d! (Transiciones: %d) <<<\n", i, transitions[i]);
      anyPulse = true;
    }
  }

  if (!anyPulse) {
    Serial.print("Sin pulsos. Estados: ");
    for (int i = 0; i <= 15; i++) {
      Serial.printf("D%d=%d ", i, lastState[i]);
    }
    Serial.println();
  }
}
