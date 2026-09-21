#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN PA5
#endif

// Pines a escanear en la Nucleo
const int pinsToScan[] = { 0, 1, 2, 50 }; // D0 (PB15), D1 (PB14), D2 (PA10), D50 (PA9)
const char* pinNames[] = { "Pin D0 (PB15)", "Pin D1 (PB14)", "Pin D2 (PA10)", "Pin PA9" };
const int numPins = 4;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < numPins; i++) {
    pinMode(pinsToScan[i], INPUT_PULLUP);
  }

  Serial.println("\n==================================================");
  Serial.println("   STM32 Pin Activity Scanner");
  Serial.println("   Detectando cual pin recibe pulsos del ESP32...");
  Serial.println("==================================================");
}

void loop() {
  int transitions[numPins] = {0};
  int lastState[numPins];

  for (int i = 0; i < numPins; i++) {
    lastState[i] = digitalRead(pinsToScan[i]);
  }

  // Muestrear durante 1 segundo (el ESP32 envia 1 ping cada segundo)
  uint32_t tStart = millis();
  while (millis() - tStart < 1200) {
    for (int i = 0; i < numPins; i++) {
      int s = digitalRead(pinsToScan[i]);
      if (s != lastState[i]) {
        transitions[i]++;
        lastState[i] = s;
      }
    }
  }

  // Reportar actividad
  Serial.println("\n--- Reporte de Actividad (en 1.2s) ---");
  bool anyActive = false;
  for (int i = 0; i < numPins; i++) {
    int cur = digitalRead(pinsToScan[i]);
    Serial.printf("  %s: Estado=%s | Transiciones=%d\n", 
      pinNames[i], 
      cur ? "HIGH (3.3V)" : "LOW (0V)", 
      transitions[i]);
    if (transitions[i] > 5) {
      anyActive = true;
      Serial.printf("  >>> ¡SENAL DETECTADA EN %s! <<<\n", pinNames[i]);
    }
  }

  if (anyActive) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    Serial.println("  [!] Ningun pin recibio datos. Revisa si el ESP32 esta encendido y conectado a TX2 (GPIO 17).");
  }
}
