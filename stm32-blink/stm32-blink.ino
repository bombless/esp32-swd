#include <Arduino.h>

// Test the two suspected onboard LED pins on this STM32 board.
// PA8 and PB2 will blink together.

void setup() {
  pinMode(PA8, OUTPUT);
  pinMode(PB2, OUTPUT);
}

void loop() {
  digitalWrite(PA8, HIGH);
  digitalWrite(PB2, HIGH);
  delay(500);

  digitalWrite(PA8, LOW);
  digitalWrite(PB2, LOW);
  delay(500);
}
