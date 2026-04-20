#include <Arduino.h>
#include <Wire.h>

// ============================================================
// MODE SELECT
// Change this value to switch between test modes:
// 1 = Single IMU test (channel 0)
// 2 = FSR test
// 3 = All 3 IMUs test
// 4 = WiFi streaming
// 5 = SD card logging
// 6 = Haptic feedback test
// 7 = Gait event detection
// 8 = Bilateral synchronisation
// 9 = Full functionality
// ============================================================
#define MODE_SELECT 1

// ============================================================
// HARDWARE CONSTANTS
// I2C addresses
// PCA9548A default address when A0/A1/A2 all tied to GND
// MPU6050 default address when AD0 tied to GND
// ============================================================
#define TCA_ADDRESS 0x70
#define MPU_ADDRESS 0x68

// ESP32 I2C pins
// GPIO 21 and 22 are the default hardware I2C pins on ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// FSR analog input pins
// Must use ADC1 pins only — ADC2 conflicts with WiFi
// GPIO 32, 33, 34, 35 are all ADC1 safe
#define FSR_HEEL_PIN 32
#define FSR_FORE_PIN 33

// Haptic motor pin
#define HAPTIC_PIN 25

// ============================================================
// MULTIPLEXER CHANNEL CLOSE ALL
// Writing 0x00 to the PCA9548A closes all channels
// Called on boot to ensure clean state before any channel
// is selected — prevents IMU appearing on bus before intended
// ============================================================
void tcaCloseAll() {
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(0x00);       // 0x00 closes all 8 channels
    Wire.endTransmission();
}

// ============================================================
// MULTIPLEXER CHANNEL SELECT
// The PCA9548A has 8 channels (0-7)
// Writing (1 << channel) to the multiplexer opens that channel
// For example:
//   channel 0: writes 0b00000001
//   channel 1: writes 0b00000010
//   channel 2: writes 0b00000100
// Only one channel is open at a time
// Always call tcaCloseAll() on boot before first tcaSelect()
// ============================================================
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;        // Safety check — only 8 channels exist
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(1 << channel);       // Bit shift opens the selected channel
    Wire.endTransmission();
}

// ============================================================
// RAW IMU DATA READ
// MPU6050 stores each axis as a 16-bit signed integer
// split across two consecutive 8-bit registers
// High byte comes first, low byte second
// We read both bytes and combine them:
//   (highByte << 8) | lowByte = 16-bit value
// The 'false' in endTransmission keeps the I2C bus active
// so we can immediately request the data bytes
// ============================================================
int16_t readRawData(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(reg);                // Tell MPU which register to read from
    Wire.endTransmission(false);    // Keep bus active (repeated start)
    Wire.requestFrom(MPU_ADDRESS, 2); // Request 2 bytes
    return (Wire.read() << 8) | Wire.read(); // Combine high and low bytes
}

// ============================================================
// IMU INITIALISATION
// MPU6050 starts in sleep mode by default
// Register 0x6B is the power management register
// Writing 0x00 to it wakes the device up
// Must be called after selecting the correct multiplexer channel
// ============================================================
void initIMU() {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);   // Power management register address
    Wire.write(0x00);   // Write 0 to wake up — clears sleep bit
    Wire.endTransmission();
}

// ============================================================
// READ AND PRINT ONE IMU
// Reads all 6 axes from whichever IMU is currently selected
// via the multiplexer, converts to physical units, prints
//
// Accelerometer conversion:
//   Default range is ±2g
//   LSB sensitivity is 16384 counts per g
//   Raw value / 16384.0 = acceleration in g
//
// Gyroscope conversion:
//   Default range is ±250 degrees per second
//   LSB sensitivity is 131 counts per degree/second
//   Raw value / 131.0 = angular velocity in degrees/second
// ============================================================
void readAndPrintIMU(uint8_t channel) {
    tcaSelect(channel);     // Open the correct multiplexer channel

    // Accelerometer register map:
    // 0x3B = ACCEL_XOUT_H (X high byte)
    // 0x3C = ACCEL_XOUT_L (X low byte)
    // 0x3D = ACCEL_YOUT_H
    // 0x3E = ACCEL_YOUT_L
    // 0x3F = ACCEL_ZOUT_H
    // 0x40 = ACCEL_ZOUT_L
    int16_t ax = readRawData(0x3B);
    int16_t ay = readRawData(0x3D);
    int16_t az = readRawData(0x3F);

    // Gyroscope register map:
    // 0x43 = GYRO_XOUT_H
    // 0x44 = GYRO_XOUT_L
    // 0x45 = GYRO_YOUT_H
    // 0x46 = GYRO_YOUT_L
    // 0x47 = GYRO_ZOUT_H
    // 0x48 = GYRO_ZOUT_L
    int16_t gx = readRawData(0x43);
    int16_t gy = readRawData(0x45);
    int16_t gz = readRawData(0x47);

    // Convert raw values to physical units
    float ax_g  = ax / 16384.0;
    float ay_g  = ay / 16384.0;
    float az_g  = az / 16384.0;
    float gx_ds = gx / 131.0;
    float gy_ds = gy / 131.0;
    float gz_ds = gz / 131.0;

    // Print results
    Serial.print("CH"); Serial.print(channel);
    Serial.print(" | Accel (g): ");
    Serial.print(ax_g, 3); Serial.print(", ");
    Serial.print(ay_g, 3); Serial.print(", ");
    Serial.print(az_g, 3);
    Serial.print(" | Gyro (deg/s): ");
    Serial.print(gx_ds, 3); Serial.print(", ");
    Serial.print(gy_ds, 3); Serial.print(", ");
    Serial.println(gz_ds, 3);
}

// ============================================================
// FSR READ AND PRINT
// FSRs are analog resistive sensors wired in a voltage divider
// with a fixed resistor (10kΩ) to GND
// ESP32 ADC reads 0-4095 (12-bit) mapping to 0-3.3V
// Higher pressure = lower FSR resistance = higher voltage
// = higher ADC value
// ============================================================
void readAndPrintFSR() {
    int heel = analogRead(FSR_HEEL_PIN);
    int fore = analogRead(FSR_FORE_PIN);

    // Convert raw ADC to voltage for more meaningful output
    float heel_v = heel * (3.3 / 4095.0);
    float fore_v = fore * (3.3 / 4095.0);

    Serial.print("FSR Heel: ");
    Serial.print(heel);
    Serial.print(" (");
    Serial.print(heel_v, 2);
    Serial.print("V) | FSR Fore: ");
    Serial.print(fore);
    Serial.print(" (");
    Serial.print(fore_v, 2);
    Serial.println("V)");
}

// ============================================================
// HAPTIC TEST
// Simple pattern to verify vibration motor is working
// Motor is driven by a digital GPIO pin through a transistor
// digitalWrite HIGH activates the motor
// ============================================================
void hapticTest() {
    Serial.println("Haptic: 3 short pulses");
    for (int i = 0; i < 3; i++) {
        digitalWrite(HAPTIC_PIN, HIGH);
        delay(100);
        digitalWrite(HAPTIC_PIN, LOW);
        delay(100);
    }
    delay(1000);
}

// ============================================================
// SETUP
// Runs once on boot
// Initialises serial, I2C, and any hardware needed
// for the selected mode
// ============================================================
void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.print("StrideSync booting | Mode: ");
    Serial.println(MODE_SELECT);

    // I2C initialisation — required for all IMU modes
    if (MODE_SELECT == 1 || MODE_SELECT == 3) {
        pinMode(SDA_PIN, INPUT_PULLUP);     // Enable internal pull-ups
        pinMode(SCL_PIN, INPUT_PULLUP);     // on both I2C lines
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(100000);              // 100kHz standard mode
                                            // proven stable on this hardware
                                            // upgrade to 400kHz after all
                                            // 3 IMUs verified working

        tcaCloseAll();                      // Close all channels on boot
                                            // prevents IMU appearing on bus
                                            // before channel is selected

        // Initialise IMU on channel 0
        tcaSelect(0);
        initIMU();
        Serial.println("IMU on channel 0 initialised");

        // For mode 3 initialise all 3 IMUs
        if (MODE_SELECT == 3) {
            tcaSelect(1);
            initIMU();
            Serial.println("IMU on channel 1 initialised");
            tcaSelect(2);
            initIMU();
            Serial.println("IMU on channel 2 initialised");
        }
    }

    // FSR mode — analog pins need no special init on ESP32
    if (MODE_SELECT == 2) {
        Serial.println("FSR test mode — sensors on pins 32 and 33");
    }

    // Haptic mode
    if (MODE_SELECT == 6) {
        pinMode(HAPTIC_PIN, OUTPUT);
        Serial.println("Haptic test mode");
    }
}

// ============================================================
// LOOP
// Runs repeatedly after setup
// Routes to the correct test based on MODE_SELECT
// ============================================================
void loop() {
    switch (MODE_SELECT) {

        // ----------------------------------------
        // MODE 1: Single IMU on channel 0
        // Expected: Z accel ~1.0g flat, gyro ~0 stationary
        // ----------------------------------------
        case 1:
            readAndPrintIMU(0);
            delay(100);
            break;

        // ----------------------------------------
        // MODE 2: FSR test
        // Press heel and forefoot sensors
        // Expected: values rise with pressure
        // ----------------------------------------
        case 2:
            readAndPrintFSR();
            delay(100);
            break;

        // ----------------------------------------
        // MODE 3: All 3 IMUs via multiplexer
        // Wire IMUs to channels 0, 1, 2
        // Expected: all 3 showing independent data
        // ----------------------------------------
        case 3:
            readAndPrintIMU(0);
            readAndPrintIMU(1);
            readAndPrintIMU(2);
            Serial.println("---");
            delay(100);
            break;

        // ----------------------------------------
        // MODE 4-9: Placeholders for future modes
        // ----------------------------------------
        case 4:
            Serial.println("Mode 4 (WiFi) not yet implemented");
            delay(1000);
            break;

        case 5:
            Serial.println("Mode 5 (SD card) not yet implemented");
            delay(1000);
            break;

        case 6:
            hapticTest();
            break;

        case 7:
            Serial.println("Mode 7 (Gait detection) not yet implemented");
            delay(1000);
            break;

        case 8:
            Serial.println("Mode 8 (Bilateral sync) not yet implemented");
            delay(1000);
            break;

        case 9:
            Serial.println("Mode 9 (Full) not yet implemented");
            delay(1000);
            break;

        default:
            Serial.println("Invalid mode — set MODE_SELECT to 1-9");
            delay(1000);
            break;
    }
}