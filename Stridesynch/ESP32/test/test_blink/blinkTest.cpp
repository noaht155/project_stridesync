#include <Arduino.h>
void setup() {
    pinMode(2, OUTPUT); // Built in LED on most ESP32 devkits
}

void loop() {
    digitalWrite(2, HIGH);
    delay(100);
    digitalWrite(2, LOW);
    delay(100);
}