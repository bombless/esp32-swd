#include <Arduino.h>

// Simple STM32F103C8T6 Blue Pill LED blink test.
// The onboard LED is on PC13 and is active-low.

#ifndef LED_BUILTIN
#define LED_BUILTIN PC13
#endif

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);   // LED on
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);  // LED off
  delay(500);
}
