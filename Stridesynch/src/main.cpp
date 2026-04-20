#include <Arduino.h>
#include <Wire.h>

// ============================================================
// MODE SELECT
// 1 = IMU auto-detect and test
// 2 = FSR test
// 3 = All detected IMUs streaming
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
// ============================================================
#define TCA_ADDRESS  0x70
#define MPU_ADDRESS  0x68
#define SDA_PIN      21
#define SCL_PIN      22
#define FSR_HEEL_PIN 32
#define FSR_FORE_PIN 33
#define HAPTIC_PIN   25
#define CALIB_SAMPLES 1000

// ============================================================
// IMU CHANNEL MAPPING
// Physical meaning of each multiplexer channel
// Channels 0-2: Left leg (foot, shank, thigh)
// Channels 3-5: Right leg (foot, shank, thigh)
// Channels 6-7: Unused
// Update these labels to match your physical wiring
// ============================================================
const char* CHANNEL_LABELS[] = {
    "L-Foot",   // Channel 0
    "L-Shank",  // Channel 1
    "L-Thigh",  // Channel 2
    "R-Foot",   // Channel 3
    "R-Shank",  // Channel 4
    "R-Thigh",  // Channel 5
    "Unused",   // Channel 6
    "Unused"    // Channel 7
};

// ============================================================
// IMU STATE TRACKING
// imuConnected[] — true if IMU detected on that channel
// imuCount — total number of detected IMUs
// accelOffset / gyroOffset — calibration offsets per channel
// ============================================================
bool  imuConnected[8]    = {false};
uint8_t imuCount         = 0;
float accelOffset[8][3]  = {{0}};
float gyroOffset[8][3]   = {{0}};

// ============================================================
// MULTIPLEXER FUNCTIONS
// ============================================================
void tcaCloseAll() {
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
}

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// ============================================================
// RAW IMU DATA READ
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
// ============================================================
void initIMU() {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

// ============================================================
// IMU DETECTION
// Attempts to communicate with MPU at 0x68 on each channel
// A successful transmission with error code 0 means IMU present
// Populates imuConnected[] and imuCount
// Prints a summary of what was found and where
// ============================================================
void detectIMUs() {
    Serial.println("--- Scanning for IMUs ---");
    imuCount = 0;

    for (uint8_t ch = 0; ch < 8; ch++) {
        tcaSelect(ch);
        Wire.beginTransmission(MPU_ADDRESS);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            imuConnected[ch] = true;
            imuCount++;
            Serial.print("IMU found | Channel ");
            Serial.print(ch);
            Serial.print(" | ");
            Serial.println(CHANNEL_LABELS[ch]);
        } else {
            imuConnected[ch] = false;
        }
    }

    tcaCloseAll();

    Serial.print("Total IMUs detected: ");
    Serial.println(imuCount);

    if (imuCount == 0) {
        Serial.println("ERROR: No IMUs found — check wiring");
    } else if (imuCount < 6) {
        Serial.print("WARNING: Expected 6 IMUs, found ");
        Serial.println(imuCount);
    } else {
        Serial.println("All 6 IMUs detected");
    }

    Serial.println("-------------------------");
}

// ============================================================
// IMU CALIBRATION
// Calibrates only detected IMUs — skips empty channels
// Places haptic feedback at start, during, and end
// Averages CALIB_SAMPLES readings per IMU
// Subtracts 16384 from accel Z offset (expected 1g)
// ============================================================
void calibrateIMU(uint8_t channel) {
    if (!imuConnected[channel]) return;     // Skip if no IMU on this channel

    tcaSelect(channel);

    Serial.print("Calibrating ");
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.println(" — hold still...");

    // 3 short buzzes: calibration starting
    for (int i = 0; i < 3; i++) {
        digitalWrite(HAPTIC_PIN, HIGH); delay(100);
        digitalWrite(HAPTIC_PIN, LOW);  delay(100);
    }

    delay(2000);    // Time to place IMU flat and still

    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        tcaSelect(channel);
        ax_sum += readRawData(0x3B);
        ay_sum += readRawData(0x3D);
        az_sum += readRawData(0x3F);
        gx_sum += readRawData(0x43);
        gy_sum += readRawData(0x45);
        gz_sum += readRawData(0x47);

        // Haptic pulse every 200 samples — 5 pulses total
        if (i % 200 == 0) {
            digitalWrite(HAPTIC_PIN, HIGH); delay(30);
            digitalWrite(HAPTIC_PIN, LOW);
        }

        delay(5);
    }

    // Store offsets
    accelOffset[channel][0] = (float)ax_sum / CALIB_SAMPLES;
    accelOffset[channel][1] = (float)ay_sum / CALIB_SAMPLES;
    accelOffset[channel][2] = (float)az_sum / CALIB_SAMPLES - 16384.0;
    gyroOffset[channel][0]  = (float)gx_sum / CALIB_SAMPLES;
    gyroOffset[channel][1]  = (float)gy_sum / CALIB_SAMPLES;
    gyroOffset[channel][2]  = (float)gz_sum / CALIB_SAMPLES;

    // Print offsets for verification
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(" Accel offsets: ");
    Serial.print(accelOffset[channel][0], 1); Serial.print(", ");
    Serial.print(accelOffset[channel][1], 1); Serial.print(", ");
    Serial.println(accelOffset[channel][2], 1);

    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(" Gyro offsets:  ");
    Serial.print(gyroOffset[channel][0], 1); Serial.print(", ");
    Serial.print(gyroOffset[channel][1], 1); Serial.print(", ");
    Serial.println(gyroOffset[channel][2], 1);

    // 2 long buzzes: calibration complete
    digitalWrite(HAPTIC_PIN, HIGH); delay(300);
    digitalWrite(HAPTIC_PIN, LOW);  delay(150);
    digitalWrite(HAPTIC_PIN, HIGH); delay(300);
    digitalWrite(HAPTIC_PIN, LOW);

    Serial.print(CHANNEL_LABELS[channel]);
    Serial.println(" calibration complete");
    Serial.println();
}

// ============================================================
// CALIBRATE ALL DETECTED IMUs
// Iterates through all 8 channels
// Skips channels with no IMU detected
// ============================================================
void calibrateAll() {
    Serial.println("--- Beginning calibration ---");
    Serial.println("Place all IMUs flat and still");
    delay(3000);

    for (uint8_t ch = 0; ch < 8; ch++) {
        if (imuConnected[ch]) {
            calibrateIMU(ch);
        }
    }

    Serial.println("--- All calibrations complete ---");
}

// ============================================================
// READ AND PRINT ONE IMU
// Subtracts calibration offsets before converting
// Skips if no IMU on this channel
// ============================================================
void readAndPrintIMU(uint8_t channel) {
    if (!imuConnected[channel]) return;

    tcaSelect(channel);

    int16_t ax_raw = readRawData(0x3B);
    int16_t ay_raw = readRawData(0x3D);
    int16_t az_raw = readRawData(0x3F);
    int16_t gx_raw = readRawData(0x43);
    int16_t gy_raw = readRawData(0x45);
    int16_t gz_raw = readRawData(0x47);

    float ax_g  = (ax_raw - accelOffset[channel][0]) / 16384.0;
    float ay_g  = (ay_raw - accelOffset[channel][1]) / 16384.0;
    float az_g  = (az_raw - accelOffset[channel][2]) / 16384.0;
    float gx_ds = (gx_raw - gyroOffset[channel][0])  / 131.0;
    float gy_ds = (gy_raw - gyroOffset[channel][1])  / 131.0;
    float gz_ds = (gz_raw - gyroOffset[channel][2])  / 131.0;

    Serial.print(CHANNEL_LABELS[channel]);
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
// READ ALL DETECTED IMUs
// ============================================================
void readAllIMUs() {
    for (uint8_t ch = 0; ch < 8; ch++) {
        if (imuConnected[ch]) {
            readAndPrintIMU(ch);
        }
    }
    Serial.println("---");
}

// ============================================================
// FSR READ AND PRINT
// ============================================================
void readAndPrintFSR() {
    int heel = analogRead(FSR_HEEL_PIN);
    int fore = analogRead(FSR_FORE_PIN);

    float heel_v = heel * (3.3 / 4095.0);
    float fore_v = fore * (3.3 / 4095.0);

    Serial.print("FSR Heel: ");
    Serial.print(heel);
    Serial.print(" ("); Serial.print(heel_v, 2); Serial.print("V)");
    Serial.print(" | FSR Fore: ");
    Serial.print(fore);
    Serial.print(" ("); Serial.print(fore_v, 2); Serial.println("V)");
}

// ============================================================
// HAPTIC TEST
// ============================================================
void hapticTest() {
    Serial.println("Haptic: 3 short pulses");
    for (int i = 0; i < 3; i++) {
        digitalWrite(HAPTIC_PIN, HIGH); delay(100);
        digitalWrite(HAPTIC_PIN, LOW);  delay(100);
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

    pinMode(HAPTIC_PIN, OUTPUT);

    // I2C init required for any IMU mode
    if (MODE_SELECT == 1 || MODE_SELECT == 3 || MODE_SELECT == 9) {
        pinMode(SDA_PIN, INPUT_PULLUP);
        pinMode(SCL_PIN, INPUT_PULLUP);
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(100000);

        tcaCloseAll();

        // Detect which channels have IMUs
        detectIMUs();

        // Initialise all detected IMUs
        for (uint8_t ch = 0; ch < 8; ch++) {
            if (imuConnected[ch]) {
                tcaSelect(ch);
                initIMU();
            }
        }
        tcaCloseAll();

        // Calibrate all detected IMUs
        calibrateAll();
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
        // MODE 1: Auto-detect and stream all IMUs
        // Detects however many are connected
        // Labels each by anatomical position
        // ----------------------------------------
        case 1:
            readAllIMUs();
            delay(100);
            break;

        // ----------------------------------------
        // MODE 2: FSR test
        // ----------------------------------------
        case 2:
            readAndPrintFSR();
            delay(100);
            break;

        // ----------------------------------------
        // MODE 3: All detected IMUs — same as mode 1
        // Kept separate for future mode 1 single IMU use
        // ----------------------------------------
        case 3:
            readAllIMUs();
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