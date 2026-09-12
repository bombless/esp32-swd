#include <Arduino.h>

// STM32F103C8T6 Blue Pill onboard LED is connected to PC13.
// The LED is active-low: LOW = on, HIGH = off.

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
}

void loop() {
  digitalWrite(PC13, LOW);
  delay(500);

  digitalWrite(PC13, HIGH);
  delay(500);
}
