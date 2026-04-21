#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// MODE SELECT
// 1  = All detected IMUs — labelled output
// 2  = FSR test
// 3  = Mahony filter — orientation angles
// 4  = WiFi streaming
// 5  = SD card logging
// 6  = Haptic feedback test
// 7  = Gait event detection
// 8  = Bilateral synchronisation
// 9  = Full functionality
// 10 = Raw plotter mode (comma separated, single IMU)
// ============================================================
#define MODE_SELECT 3

// ============================================================
// HARDWARE CONSTANTS
// ============================================================
#define TCA_ADDRESS   0x70
#define MPU_ADDRESS   0x68
#define SDA_PIN       21
#define SCL_PIN       22
#define FSR_HEEL_PIN  32
#define FSR_FORE_PIN  33
#define HAPTIC_PIN    25
#define CALIB_SAMPLES 1000

// ============================================================
// MAHONY FILTER CONSTANTS
// Kp — proportional gain
//   Controls how strongly accelerometer corrects gyro drift
//   Higher = faster correction but more sensitive to vibration
//   Lower  = smoother but slower to correct drift
//   0.5 is a good starting point for walking/running
// Ki — integral gain
//   Corrects slowly accumulating gyro bias over time
//   0.0 disables integral correction — start here
//   Enable (0.01-0.1) if drift persists after calibration
// ============================================================
#define Kp 0.5f
#define Ki 0.0f

// ============================================================
// CHANNEL LABELS
// Update to match your physical wiring
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
// IMU STATE
// ============================================================
bool    imuConnected[8]   = {false};
uint8_t imuCount          = 0;
float   accelOffset[8][3] = {{0}};
float   gyroOffset[8][3]  = {{0}};

// ============================================================
// MAHONY FILTER STATE
// One quaternion per IMU channel — represents orientation
// Quaternion components: q0=w, q1=x, q2=y, q3=z
// Initialised to identity quaternion (no rotation)
// integralFB — integral feedback term for Ki correction
// lastTime — timestamp of last update per channel in microseconds
// ============================================================
float q0[8], q1[8], q2[8], q3[8];
float integralFBx[8], integralFBy[8], integralFBz[8];
unsigned long lastTime[8];

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
// ============================================================
void calibrateIMU(uint8_t channel) {
    if (!imuConnected[channel]) return;

    tcaSelect(channel);
    Serial.print("Calibrating ");
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.println(" — hold still...");

    for (int i = 0; i < 3; i++) {
        digitalWrite(HAPTIC_PIN, HIGH); delay(100);
        digitalWrite(HAPTIC_PIN, LOW);  delay(100);
    }

    delay(2000);

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

        if (i % 200 == 0) {
            digitalWrite(HAPTIC_PIN, HIGH); delay(30);
            digitalWrite(HAPTIC_PIN, LOW);
        }
        delay(5);
    }

    accelOffset[channel][0] = (float)ax_sum / CALIB_SAMPLES;
    accelOffset[channel][1] = (float)ay_sum / CALIB_SAMPLES;
    accelOffset[channel][2] = (float)az_sum / CALIB_SAMPLES - 16384.0;
    gyroOffset[channel][0]  = (float)gx_sum / CALIB_SAMPLES;
    gyroOffset[channel][1]  = (float)gy_sum / CALIB_SAMPLES;
    gyroOffset[channel][2]  = (float)gz_sum / CALIB_SAMPLES;

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

    digitalWrite(HAPTIC_PIN, HIGH); delay(300);
    digitalWrite(HAPTIC_PIN, LOW);  delay(150);
    digitalWrite(HAPTIC_PIN, HIGH); delay(300);
    digitalWrite(HAPTIC_PIN, LOW);

    Serial.print(CHANNEL_LABELS[channel]);
    Serial.println(" calibration complete");
    Serial.println();
}

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
// MAHONY FILTER INITIALISATION
// Sets each quaternion to identity — no rotation
// Clears integral feedback terms
// Records current time for first dt calculation
// Must be called after calibration, before filter runs
// ============================================================
void initMahony() {
    for (uint8_t ch = 0; ch < 8; ch++) {
        q0[ch] = 1.0f;     // w component — 1.0 = no rotation
        q1[ch] = 0.0f;     // x component
        q2[ch] = 0.0f;     // y component
        q3[ch] = 0.0f;     // z component
        integralFBx[ch] = 0.0f;
        integralFBy[ch] = 0.0f;
        integralFBz[ch] = 0.0f;
        lastTime[ch] = micros();
    }
    Serial.println("Mahony filter initialised");
}

// ============================================================
// MAHONY FILTER UPDATE
// Called once per sample per IMU
// Takes calibrated accelerometer (g) and gyroscope (rad/s)
// Updates quaternion for that channel
//
// Steps:
// 1. Normalise accelerometer vector — removes magnitude,
//    keeps direction which is all we need for gravity reference
// 2. Estimate gravity direction from current quaternion
// 3. Cross product of estimated vs measured gravity = error
// 4. Apply proportional (Kp) correction to gyro rates
// 5. Apply integral (Ki) correction if enabled
// 6. Integrate corrected gyro rates to update quaternion
// 7. Normalise quaternion — prevents numerical drift
//
// Note: gyro input must be in radians/second not degrees/second
// ============================================================
void mahonyUpdate(uint8_t ch,
                  float ax, float ay, float az,
                  float gx, float gy, float gz) {

    // Calculate dt in seconds since last update for this channel
    unsigned long now = micros();
    float dt = (now - lastTime[ch]) / 1000000.0f;
    lastTime[ch] = now;

    // Guard against bad dt — first call or timer overflow
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;

    // Step 1: Normalise accelerometer
    // If accelerometer reads all zero something is wrong — skip
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Step 2: Estimated gravity direction from current quaternion
    // These equations derive the gravity vector from the quaternion
    // by rotating the reference gravity vector [0, 0, 1] by the
    // inverse of the current orientation
    float vx = 2.0f * (q1[ch]*q3[ch] - q0[ch]*q2[ch]);
    float vy = 2.0f * (q0[ch]*q1[ch] + q2[ch]*q3[ch]);
    float vz = q0[ch]*q0[ch] - q1[ch]*q1[ch] - q2[ch]*q2[ch] + q3[ch]*q3[ch];

    // Step 3: Cross product of estimated vs measured gravity
    // This gives us the rotation error between where we think
    // gravity points and where the accelerometer says it points
    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    // Step 4: Integral feedback (Ki)
    // Accumulates error over time to correct slow drift
    // Disabled by default (Ki = 0.0)
    if (Ki > 0.0f) {
        integralFBx[ch] += Ki * ex * dt;
        integralFBy[ch] += Ki * ey * dt;
        integralFBz[ch] += Ki * ez * dt;
        gx += integralFBx[ch];
        gy += integralFBy[ch];
        gz += integralFBz[ch];
    }

    // Step 5: Proportional feedback (Kp)
    // Directly corrects gyro rates based on current error
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;

    // Step 6: Integrate corrected gyro to update quaternion
    // This is the first order quaternion integration formula
    // Multiply by dt/2 — quaternion derivative = 0.5 * q * omega
    float qa = q0[ch], qb = q1[ch], qc = q2[ch];
    q0[ch] += (-qb*gx - qc*gy - q3[ch]*gz) * (0.5f * dt);
    q1[ch] += ( qa*gx + qc*gz - q3[ch]*gy) * (0.5f * dt);
    q2[ch] += ( qa*gy - qb*gz + q3[ch]*gx) * (0.5f * dt);
    q3[ch] += ( qa*gz + qb*gy - qc*gx)     * (0.5f * dt);

    // Step 7: Normalise quaternion
    // Prevents numerical errors accumulating over time
    norm = sqrtf(q0[ch]*q0[ch] + q1[ch]*q1[ch] +
                 q2[ch]*q2[ch] + q3[ch]*q3[ch]);
    norm = 1.0f / norm;
    q0[ch] *= norm;
    q1[ch] *= norm;
    q2[ch] *= norm;
    q3[ch] *= norm;
}

// ============================================================
// QUATERNION TO EULER ANGLES
// Converts quaternion to roll, pitch, yaw in degrees
// Roll  = rotation around X axis (side to side tilt)
// Pitch = rotation around Y axis (forward/back tilt)
// Yaw   = rotation around Z axis (compass heading)
//
// Note: yaw will drift over time without a magnetometer
// For gait analysis roll and pitch are the useful angles
// as they capture joint flexion and lateral lean
// ============================================================
void quaternionToEuler(uint8_t ch,
                       float &roll, float &pitch, float &yaw) {
    roll  = atan2f(2.0f*(q0[ch]*q1[ch] + q2[ch]*q3[ch]),
                   1.0f - 2.0f*(q1[ch]*q1[ch] + q2[ch]*q2[ch]));
    pitch = asinf( 2.0f*(q0[ch]*q2[ch] - q3[ch]*q1[ch]));
    yaw   = atan2f(2.0f*(q0[ch]*q3[ch] + q1[ch]*q2[ch]),
                   1.0f - 2.0f*(q2[ch]*q2[ch] + q3[ch]*q3[ch]));

    // Convert radians to degrees
    roll  *= 180.0f / M_PI;
    pitch *= 180.0f / M_PI;
    yaw   *= 180.0f / M_PI;
}

// ============================================================
// READ IMU AND UPDATE MAHONY FILTER
// Reads raw data, applies calibration offsets,
// converts gyro to radians/second for filter input,
// updates Mahony filter, converts result to Euler angles,
// prints angles to serial
// ============================================================
void updateAndPrintIMU(uint8_t channel) {
    if (!imuConnected[channel]) return;

    tcaSelect(channel);

    int16_t ax_raw = readRawData(0x3B);
    int16_t ay_raw = readRawData(0x3D);
    int16_t az_raw = readRawData(0x3F);
    int16_t gx_raw = readRawData(0x43);
    int16_t gy_raw = readRawData(0x45);
    int16_t gz_raw = readRawData(0x47);

    // Apply calibration offsets and convert to physical units
    float ax = (ax_raw - accelOffset[channel][0]) / 16384.0f;
    float ay = (ay_raw - accelOffset[channel][1]) / 16384.0f;
    float az = (az_raw - accelOffset[channel][2]) / 16384.0f;

    // Gyro must be in radians/second for Mahony filter
    // Divide by 131.0 for deg/s then multiply by pi/180 for rad/s
    float gx = (gx_raw - gyroOffset[channel][0]) / 131.0f * (M_PI / 180.0f);
    float gy = (gy_raw - gyroOffset[channel][1]) / 131.0f * (M_PI / 180.0f);
    float gz = (gz_raw - gyroOffset[channel][2]) / 131.0f * (M_PI / 180.0f);

    // Update filter
    mahonyUpdate(channel, ax, ay, az, gx, gy, gz);

    // Get Euler angles
    float roll, pitch, yaw;
    quaternionToEuler(channel, roll, pitch, yaw);

    // Print
    Serial.print(CHANNEL_LABELS[channel]);
    Serial.print(" | Roll: ");
    Serial.print(roll, 1);
    Serial.print("° | Pitch: ");
    Serial.print(pitch, 1);
    Serial.print("° | Yaw: ");
    Serial.print(yaw, 1);
    Serial.println("°");
}

// ============================================================
// UPDATE ALL DETECTED IMUs
// ============================================================
void updateAllIMUs() {
    for (uint8_t ch = 0; ch < 8; ch++) {
        if (imuConnected[ch]) {
            updateAndPrintIMU(ch);
        }
    }
    Serial.println("---");
}

// ============================================================
// RAW READ AND PRINT — for mode 1 and plotter
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

    float ax_g  = (ax_raw - accelOffset[channel][0]) / 16384.0f;
    float ay_g  = (ay_raw - accelOffset[channel][1]) / 16384.0f;
    float az_g  = (az_raw - accelOffset[channel][2]) / 16384.0f;
    float gx_ds = (gx_raw - gyroOffset[channel][0])  / 131.0f;
    float gy_ds = (gy_raw - gyroOffset[channel][1])  / 131.0f;
    float gz_ds = (gz_raw - gyroOffset[channel][2])  / 131.0f;

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
    float heel_v = heel * (3.3f / 4095.0f);
    float fore_v = fore * (3.3f / 4095.0f);
    Serial.print("FSR Heel: "); Serial.print(heel);
    Serial.print(" ("); Serial.print(heel_v, 2); Serial.print("V)");
    Serial.print(" | FSR Fore: "); Serial.print(fore);
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

    if (MODE_SELECT == 1 || MODE_SELECT == 3 ||
        MODE_SELECT == 9 || MODE_SELECT == 10) {
        pinMode(SDA_PIN, INPUT);
        pinMode(SCL_PIN, INPUT);
        Wire.begin(SDA_PIN, SCL_PIN);
        Wire.setClock(100000);
        tcaCloseAll();
        detectIMUs();

        for (uint8_t ch = 0; ch < 8; ch++) {
            if (imuConnected[ch]) {
                tcaSelect(ch);
                initIMU();
            }
        }
        tcaCloseAll();
        calibrateAll();
    }

    // Mahony mode needs I2C and filter init
    if (MODE_SELECT == 3) {
        initMahony();
    }

    if (MODE_SELECT == 2) {
        Serial.println("FSR test mode");
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
        // MODE 1: Raw IMU data — all detected IMUs
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
        // MODE 3: Mahony filter — orientation angles
        // Expected when flat and still:
        //   Roll ~0°, Pitch ~0°, Yaw ~0°
        // Rotate IMU and watch angles track movement
        // Yaw will drift without magnetometer — normal
        // ----------------------------------------
        case 3:
            updateAllIMUs();
            delay(10);      // 100Hz update rate
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

        // ----------------------------------------
        // MODE 10: Plotter mode
        // Comma separated output for Python visualiser
        // Prints single IMU on channel 0 only
        // Change channel number to inspect others
        // ----------------------------------------
        case 10:
            if (imuConnected[0]) {
                tcaSelect(0);
                int16_t ax_raw = readRawData(0x3B);
                int16_t ay_raw = readRawData(0x3D);
                int16_t az_raw = readRawData(0x3F);
                int16_t gx_raw = readRawData(0x43);
                int16_t gy_raw = readRawData(0x45);
                int16_t gz_raw = readRawData(0x47);
                float ax = (ax_raw - accelOffset[0][0]) / 16384.0f;
                float ay = (ay_raw - accelOffset[0][1]) / 16384.0f;
                float az = (az_raw - accelOffset[0][2]) / 16384.0f;
                float gx = (gx_raw - gyroOffset[0][0]) / 131.0f;
                float gy = (gy_raw - gyroOffset[0][1]) / 131.0f;
                float gz = (gz_raw - gyroOffset[0][2]) / 131.0f;
                Serial.print(ax, 3); Serial.print(",");
                Serial.print(ay, 3); Serial.print(",");
                Serial.print(az, 3); Serial.print(",");
                Serial.print(gx, 3); Serial.print(",");
                Serial.print(gy, 3); Serial.print(",");
                Serial.println(gz, 3);
            }
            delay(10);
            break;

        default:
            Serial.println("Invalid mode — set MODE_SELECT to 1-9");
            delay(1000);
            break;
    }
}