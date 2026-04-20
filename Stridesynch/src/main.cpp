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
#define MODE_SELECT 3

// ============================================================
// HARDWARE CONSTANTS
// I2C addresses
// PCA9548A default address when A0/A1/A2 all tied to GND
// MPU6050 default address when AD0 tied to GND
// ============================================================
#define TCA_ADDRESS 0x70
#define MPU_ADDRESS 0x68

// ESP32 I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// FSR analog input pins
// Must use ADC1 pins only — ADC2 conflicts with WiFi
#define FSR_HEEL_PIN 32
#define FSR_FORE_PIN 33

// Haptic motor pin
#define HAPTIC_PIN 25

// ============================================================
// CALIBRATION CONSTANTS
// Number of samples to average during calibration
// More samples = more accurate bias estimate
// 1000 samples at 100Hz takes ~10 seconds
// IMU must be completely stationary during this time
// ============================================================
#define CALIB_SAMPLES 1000

// ============================================================
// CALIBRATION STORAGE
// One set of offsets per IMU channel (0, 1, 2)
// Accelerometer offsets in raw LSB units
// Gyroscope offsets in raw LSB units
// Converted to physical units during readAndPrintIMU()
//
// accelOffset[channel][axis] — axis: 0=X, 1=Y, 2=Z
// gyroOffset[channel][axis]
//
// Z accelerometer target is 16384 (1.0g) not 0
// because gravity is always present on Z when flat
// X and Y accelerometer targets are 0
// All gyro targets are 0 — sensor should read zero when still
// ============================================================
float accelOffset[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float gyroOffset[3][3]  = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// ============================================================
// MULTIPLEXER CHANNEL CLOSE ALL
// Writing 0x00 to the PCA9548A closes all channels
// Called on boot to ensure clean state
// ============================================================
void tcaCloseAll() {
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
}

// ============================================================
// MULTIPLEXER CHANNEL SELECT
// Writing (1 << channel) opens that channel exclusively
// ============================================================
void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// ============================================================
// RAW IMU DATA READ
// Reads 2 bytes from consecutive registers and combines
// into a signed 16-bit integer
// ============================================================
int16_t readRawData(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 2);
    return (Wire.read() << 8) | Wire.read();
}

// ============================================================
// IMU INITIALISATION
// Wakes MPU6050 from default sleep mode
// Must be called after selecting correct multiplexer channel
// ============================================================
void initIMU() {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

// ============================================================
// IMU CALIBRATION
// Places IMU flat and stationary, averages CALIB_SAMPLES
// readings to calculate bias offsets for each axis
//
// How it works:
// - Reads raw accelerometer and gyro values repeatedly
// - Accumulates sum of all readings
// - Divides by sample count to get average bias
// - For accel Z: subtracts 16384 (expected 1g in LSB units)
//   because gravity is real and should not be zeroed out
// - Stores offsets in accelOffset and gyroOffset arrays
// - These offsets are subtracted from every subsequent reading
//
// Haptic feedback signals calibration stages:
// - 3 short buzzes: calibration starting, place IMU flat
// - Slow pulses: calibrating, hold still
// - 2 long buzzes: calibration complete
// ============================================================
void calibrateIMU(uint8_t channel) {
    tcaSelect(channel);

    Serial.print("Calibrating IMU on channel ");
    Serial.print(channel);
    Serial.println(" — hold completely still...");

    // Haptic signal: calibration starting
    if (MODE_SELECT == 1 || MODE_SELECT == 3) {
        pinMode(HAPTIC_PIN, OUTPUT);
        for (int i = 0; i < 3; i++) {
            digitalWrite(HAPTIC_PIN, HIGH);
            delay(100);
            digitalWrite(HAPTIC_PIN, LOW);
            delay(100);
        }
    }

    delay(2000);    // Give user time to place IMU flat and still

    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        tcaSelect(channel);     // Reselect channel each iteration
                                // in case of I2C bus noise

        ax_sum += readRawData(0x3B);
        ay_sum += readRawData(0x3D);
        az_sum += readRawData(0x3F);
        gx_sum += readRawData(0x43);
        gy_sum += readRawData(0x45);
        gz_sum += readRawData(0x47);

        // Haptic pulse every 200 samples to show progress
        // 5 pulses total across 1000 samples
        if (i % 200 == 0) {
            digitalWrite(HAPTIC_PIN, HIGH);
            delay(50);
            digitalWrite(HAPTIC_PIN, LOW);
        }

        delay(5);   // ~200Hz sampling during calibration
                    // faster than runtime to get clean average quickly
    }

    // Calculate average offsets
    // Cast to float before dividing for precision
    accelOffset[channel][0] = (float)ax_sum / CALIB_SAMPLES;
    accelOffset[channel][1] = (float)ay_sum / CALIB_SAMPLES;
    accelOffset[channel][2] = (float)az_sum / CALIB_SAMPLES - 16384.0;
    // Subtract 16384 from Z accel offset — this is 1g in LSB units
    // We expect Z to read 16384 when flat so we correct toward that
    // not toward zero

    gyroOffset[channel][0] = (float)gx_sum / CALIB_SAMPLES;
    gyroOffset[channel][1] = (float)gy_sum / CALIB_SAMPLES;
    gyroOffset[channel][2] = (float)gz_sum / CALIB_SAMPLES;

    // Print calculated offsets for verification
    Serial.print("CH"); Serial.print(channel);
    Serial.print(" Accel offsets (raw): ");
    Serial.print(accelOffset[channel][0], 1); Serial.print(", ");
    Serial.print(accelOffset[channel][1], 1); Serial.print(", ");
    Serial.println(accelOffset[channel][2], 1);

    Serial.print("CH"); Serial.print(channel);
    Serial.print(" Gyro offsets (raw): ");
    Serial.print(gyroOffset[channel][0], 1); Serial.print(", ");
    Serial.print(gyroOffset[channel][1], 1); Serial.print(", ");
    Serial.println(gyroOffset[channel][2], 1);

    // Haptic signal: calibration complete
    digitalWrite(HAPTIC_PIN, HIGH);
    delay(300);
    digitalWrite(HAPTIC_PIN, LOW);
    delay(150);
    digitalWrite(HAPTIC_PIN, HIGH);
    delay(300);
    digitalWrite(HAPTIC_PIN, LOW);

    Serial.println("Calibration complete");
}

// ============================================================
// READ AND PRINT ONE IMU
// Reads all 6 axes, subtracts calibration offsets,
// converts to physical units, prints to serial
//
// Accelerometer: raw value - offset, divided by 16384.0 = g
// Gyroscope: raw value - offset, divided by 131.0 = deg/s
// ============================================================
void readAndPrintIMU(uint8_t channel) {
    tcaSelect(channel);

    int16_t ax_raw = readRawData(0x3B);
    int16_t ay_raw = readRawData(0x3D);
    int16_t az_raw = readRawData(0x3F);
    int16_t gx_raw = readRawData(0x43);
    int16_t gy_raw = readRawData(0x45);
    int16_t gz_raw = readRawData(0x47);

    // Subtract calibration offsets before converting
    float ax_g  = (ax_raw - accelOffset[channel][0]) / 16384.0;
    float ay_g  = (ay_raw - accelOffset[channel][1]) / 16384.0;
    float az_g  = (az_raw - accelOffset[channel][2]) / 16384.0;
    float gx_ds = (gx_raw - gyroOffset[channel][0])  / 131.0;
    float gy_ds = (gy_raw - gyroOffset[channel][1])  / 131.0;
    float gz_ds = (gz_raw - gyroOffset[channel][2])  / 131.0;

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
// FSRs wired in voltage divider with 10kΩ to GND
// ESP32 ADC: 12-bit, 0-4095 maps to 0-3.3V
// Higher pressure = lower resistance = higher voltage
// ============================================================
void readAndPrintFSR() {
    int heel = analogRead(FSR_HEEL_PIN);
    int fore = analogRead(FSR_FORE_PIN);

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
// 3 short pulses to verify motor wiring
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
// ============================================================
void setup() {
    delay(1000);
    Serial.begin(115200);
    Serial.print("StrideSync booting | Mode: ");
    Serial.println(MODE_SELECT);

    pinMode(HAPTIC_PIN, OUTPUT);    // Always init haptic pin
                                    // used during calibration
                                    // regardless of mode

    if (MODE_SELECT == 1 || MODE_SELECT == 3) {
        pinMode(SDA_PIN, INPUT_PULLUP);
        pinMode(SCL_PIN, INPUT_PULLUP);
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(100000);      // 100kHz — stable on this hardware
                                    // upgrade to 400kHz after all IMUs verified

        tcaCloseAll();              // Close all channels on boot

        // Calibrate and initialise IMU on channel 0
        tcaSelect(0);
        initIMU();
        calibrateIMU(0);
        Serial.println("IMU on channel 0 ready");

        if (MODE_SELECT == 3) {
            tcaSelect(1);
            initIMU();
            calibrateIMU(1);
            Serial.println("IMU on channel 1 ready");

            tcaSelect(2);
            initIMU();
            calibrateIMU(2);
            Serial.println("IMU on channel 2 ready");
        }
    }

    if (MODE_SELECT == 2) {
        Serial.println("FSR test mode — sensors on pins 32 and 33");
    }

    if (MODE_SELECT == 6) {
        Serial.println("Haptic test mode");
    }
}

// ============================================================
// LOOP
// ============================================================
void loop() {
    switch (MODE_SELECT) {

        // ----------------------------------------
        // MODE 1: Single IMU on channel 0
        // Expected after calibration:
        // Z accel = ~1.0g, X/Y accel = ~0.0g
        // All gyro axes = ~0.0 deg/s stationary
        // ----------------------------------------
        case 1:
            readAndPrintIMU(0);
            delay(100);
            break;

        // ----------------------------------------
        // MODE 2: FSR test
        // Press sensors, values should rise
        // ----------------------------------------
        case 2:
            readAndPrintFSR();
            delay(100);
            break;

        // ----------------------------------------
        // MODE 3: All 3 IMUs
        // Wire to channels 0, 1, 2
        // Each calibrated independently on boot
        // ----------------------------------------
        case 3:
            readAndPrintIMU(0);
            readAndPrintIMU(1);
            readAndPrintIMU(2);
            Serial.println("---");
            delay(100);
            break;

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