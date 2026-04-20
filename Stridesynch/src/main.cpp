#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define TCA_ADDRESS 0x70

void tcaSelect(uint8_t channel) {
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void tcaCloseAll() {
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(0x00);    // Writing 0 closes all channels
    Wire.endTransmission();
}

void i2cScan(String label) {
    Serial.println("--- " + label + " ---");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Device found at 0x");
            Serial.println(addr, HEX);
        }
    }
    Serial.println("Scan complete");
}

void setup() {
    delay(1000);
    Serial.begin(115200);
    
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);  // Slow down to standard mode

    // Scan before selecting any channel
    tcaCloseAll();
    i2cScan("Before channel select");

    // Select channel 0 and scan again
    tcaSelect(0);
    i2cScan("After channel 0 select");
}

void loop() {}