# StrideSync

**Bilateral running gait analysis wearable** — IMU sensor fusion, force sensing, and a Python dashboard for post-run biomechanical analysis.

---

## Motivation

Professional gait analysis at a sports clinic typically costs $300–$500 per session, requires specialised camera systems, and is impractical to repeat across a training season. Running is otherwise one of the most accessible sports. StrideSync makes quantitative bilateral gait analysis available for the cost of a breadboard prototype — capturing the same clinically meaningful metrics (knee flexion, ankle dorsiflexion, ground contact time, tibial shock, symmetry index) during actual outdoor and treadmill runs.

The system is validated against a Garmin watch and HRM, targeting ≥95% agreement on cadence and ground contact time.

---

## What It Does

StrideSync measures both legs simultaneously using 6 IMU sensors and 4 force sensitive resistors, processes data on an ESP32 microcontroller, and logs to a microSD card. After the run, logs are downloaded over WiFi and loaded into a Python dashboard that produces:

- **3D bilateral skeleton animation** — hip, knee, ankle, and toe tracked in real time
- **Joint angle plots** — knee flexion and ankle dorsiflexion, left vs right
- **Gait metrics** — cadence, ground contact time, tibial shock, bilateral symmetry index, fatigue comparison
- **Garmin cross-validation** — cadence and GCT agreement percentage

---

## Repository Structure

```
project_stridesync/
├── README.md
├── STRIDESYNC_CONTEXT.md          # Claude AI context document
├── Stridesynch/
│   ├── ESP32/
│   │   ├── src/main.cpp           # All firmware — mode-select architecture
│   │   └── platformio.ini
│   └── Dashboard/
│       ├── app.py                 # Plotly Dash app — run this
│       ├── data_loader.py
│       ├── kinematics.py
│       ├── metrics.py
│       ├── garmin_compare.py
│       ├── static_export.py
│       ├── requirements.txt
│       ├── transfer/
│       │   └── fetch_log.py       # WiFi log downloader
│       └── sample_data/
│           └── generate_sample.py
```

---

## Hardware

| Component | Detail |
|---|---|
| Microcontroller | ESP32 DevKit V1 (38-pin), PlatformIO + Arduino framework |
| IMU | 6× MPU-6050 (GY-521) via PCA9548A I2C multiplexer (addr 0x70) |
| IMU I2C | SDA=GPIO21, SCL=GPIO22, 100 kHz, 5 kΩ external pull-ups |
| FSR | 4× FSR-402 in 10 kΩ voltage divider — GPIO32 (L-heel), GPIO33 (L-fore), GPIO34 (R-heel), GPIO35 (R-fore) |
| SD card | SanDisk 16 GB Class 10 microSD — SPI: CLK=18, MISO=19, MOSI=23, CS=5 |
| Haptic | 10 mm coin vibration motor, GPIO25 |
| Power | 3.7 V 1000 mAh LiPo + TP4056 charging, ~5 h runtime |

**Channel mapping:**

| Channel | Segment |
|---|---|
| 0 | Left foot (dorsal midfoot) |
| 1 | Left shank (proximal tibia, lateral) |
| 2 | Left thigh (lateral midpoint) |
| 3 | Right foot |
| 4 | Right shank |
| 5 | Right thigh |

---

## Sensor Placement

Correct placement is critical. The goal at every location is to measure segment orientation with minimum movement between the sensor and the underlying bone (soft tissue artifact).

**General rules:**
- Prefer bony surfaces over muscle bellies
- Strap firmly — the sensor must not rotate or slide during heel strike
- Calibrate with sensors lying flat on a table *before* strapping on (see [Calibration Note](#calibration-note))
- Mark placement on skin or garment for session-to-session repeatability

```
[Hip / Greater Trochanter]
        |
  Ch2/5: Thigh IMU      ← lateral midpoint of thigh
        |
  [Lateral Knee]
        |
  Ch1/4: Shank IMU      ← 3–5 cm below lateral tibial plateau
        |
  [Lateral Malleolus]
        |
  Ch0/3: Foot IMU       ← dorsal midfoot, 3rd metatarsal shaft
        |
    [Toes]

  FSRs are inside the shoe (under the insole).
```

### Thigh IMU — Ch2 (Left), Ch5 (Right)

**Location:** Lateral aspect of the thigh, midpoint between the greater trochanter (outer hip bony bump) and the lateral femoral condyle (outer knee prominence).

**Why:** Mid-lateral thigh has the least soft tissue excursion relative to the femur. The anterior surface (rectus femoris) and posterior surface (hamstrings) both have large belly displacement during running — avoid both.

**Orientation:** Long axis of board parallel to the femur; board face pointing laterally (away from body).

**Securing:** 5–7 cm wide elastic athletic tape (Leukotape K or Elastikon) wrapped fully around the thigh — not just across the sensor. A compression sleeve with a sewn pocket is preferred for longer runs. Test by jogging 50 m before starting data collection; re-secure if the sensor has rotated or migrated distally.

---

### Shank IMU — Ch1 (Left), Ch4 (Right)

**Location:** Lateral aspect of the lower leg, on the flat surface of the proximal tibia, approximately 3–5 cm below the lateral tibial plateau (outer knee joint line).

**Why:** The proximal tibia is subcutaneous — minimal soft tissue between skin and bone keeps artifact low. This is the standard clinical placement for **tibial shock** measurement: high-frequency impact transients propagate up the tibia at heel strike and are best captured here before being attenuated by soft tissue. Proximal placement also gives the cleanest shank angular velocity signal.

> Do **not** place on the anterior tibial crest (prominent and uncomfortable) or the medial border (sharp). Always use the flat lateral surface.

**Orientation:** Long axis of board parallel to the tibia; board face pointing laterally.

**Securing:** Neoprene compression sleeve with a sewn lateral pocket, or a 3–4 cm Velcro strap cinched firmly around the upper shin. Grab and try to rotate the sensor — it should not move.

---

### Foot IMU — Ch0 (Left), Ch3 (Right)

**Location:** Dorsal (top) surface of the foot, over the shaft of the 3rd metatarsal or navicular bone — approximately 40–45% of foot length from the heel.

**Why:** The midfoot is the most rigid segment of the foot during running. The hindfoot is avoided because the calcaneus moves relative to the midfoot during pronation, and the forefoot deforms heavily during push-off. Dorsal placement prevents compression.

**Orientation:** Long axis of board parallel to the long axis of the foot (heel-to-toe); board face pointing upward when the foot is flat.

**Securing:** Two crossed strips of 2.5 cm medical tape (Leukotape P or Cover-Roll Stretch) anchored to the shoe upper. A small neoprene pouch glued or sewn to the shoe upper is more secure for longer runs. Do **not** place the sensor inside the shoe.

---

### Heel FSR — GPIO32 (Left), GPIO34 (Right)

**Location:** Under the posterior plantar surface of the calcaneus, slightly medial of centre — placed *under the insole* in the heel cup, approximately 1–2 cm anterior to the posterior insole edge.

**Why:** Peak plantar pressure at heel strike occurs here in rearfoot runners. For midfoot/forefoot runners the signal will have lower amplitude — this is biomechanically correct. The timing between heel and forefoot FSR identifies strike pattern.

**Securing:** The insole weight provides natural compression. A small dab of double-sided tape on the non-sensing face prevents sliding.

---

### Forefoot FSR — GPIO33 (Left), GPIO35 (Right)

**Location:** Under the 2nd–3rd metatarsal heads, at approximately 30% of insole length from the toe edge — placed under the insole at the ball of the foot.

**Why:** Peak propulsive plantar pressure during push-off concentrates at the 2nd–3rd metatarsal heads in most runners.

---

### Cable Routing

Route all IMU cables along the **lateral** aspect of the leg (not medially between the thighs). Secure cable every 10–15 cm with medical tape. Leave a small slack loop at each joint to accommodate flexion without tension on connectors.

---

### Calibration Note

The Mahony filter calibrates by measuring gravity on the IMU Z-axis. This requires all sensors to be **approximately horizontal** during calibration.

**For the wearable:**
1. Power on the ESP32 *before* strapping the device on
2. Lay the garment completely flat on a table with all 6 sensors face-up
3. Wait for the calibration haptic sequence to complete (~30 seconds for 6 IMUs)
4. *Then* strap the device onto your legs and run

Calibrating while wearing the device will produce incorrect roll/pitch angles for the entire session.

---

## Firmware Modes

Change the active mode in `ESP32/src/main.cpp`:

```cpp
#define MODE_SELECT 5   // change this number
```

Re-upload with PlatformIO (`Ctrl+Alt+U`).

| Mode | Function | Status |
|---|---|---|
| 1 | Raw IMU output — all channels, labelled | ✅ |
| 2 | FSR ADC test | ✅ |
| 3 | Mahony filter — roll/pitch/yaw all 6 IMUs | ✅ validated |
| 4 | WiFi UDP real-time streaming | 🔲 planned |
| **5** | **SD card binary logging at 200 Hz** | ✅ use for runs |
| 6 | Haptic feedback test | ✅ |
| 7 | Gait event detection | 🔲 planned |
| 8 | Bilateral synchronisation | 🔲 planned |
| 9 | Full functionality | 🔲 planned |
| 10 | Raw plotter (comma-sep, single IMU) | ✅ |
| **11** | **WiFi file server — serve SD logs for download** | ✅ |

**Haptic feedback:**

| Pattern | Meaning |
|---|---|
| 3 short pulses | Calibration starting — place IMUs flat and still |
| 5 slow pulses | Hold still during calibration |
| 2 long pulses | IMU calibration complete (repeats per sensor) |
| 3 short pulses | Mode 5/11 ready (SD card and/or WiFi connected) |
| Motor on solid | File download in progress (Mode 11) |
| 5 quick pulses | File download complete (Mode 11) |

---

## Running a Session

### Outdoor Run — SD Card Logging (Mode 5)

**Before the run:**
1. Format the microSD card as FAT32 and insert into the SPI module
2. Set `MODE_SELECT 5` in `main.cpp`, re-upload firmware
3. Power on the ESP32
4. Lay sensors flat — wait for calibration to complete (~30 s)
5. The Serial monitor shows the log filename (`run_001.bin`, etc.)
6. Strap the device on and run

**During the run:** data logs continuously at 200 Hz. The SD card is flushed every 2 seconds — if power is cut, at most the last 2 s of data is lost.

**After the run:** power cycle the ESP32. Do **not** remove the SD card while powered.

---

### Treadmill / Indoor Run — WiFi Streaming (Mode 4, planned)

Not yet implemented. When built, Mode 4 will stream UDP packets to a Python receiver on your laptop in real time, writing a CSV automatically. See `STRIDESYNC_CONTEXT.md` for the planned packet structure.

---

## Transferring Data to the Laptop

### Step 1 — Switch to File Server Mode (Mode 11)

Add your WiFi credentials to `main.cpp`:
```cpp
#define WIFI_SSID  "your_network"
#define WIFI_PASS  "your_password"
#define MODE_SELECT 11
```
Re-upload and boot. The IP address prints to the Serial monitor:
```
Connected. IP: 192.168.1.42
  http://192.168.1.42/files
```
Three short haptic pulses confirm the server is ready.

### Step 2 — Run the Fetch Script

```bash
cd Stridesynch/Dashboard
pip install -r requirements.txt     # first time only

python transfer/fetch_log.py --ip 192.168.1.42           # list + download latest
python transfer/fetch_log.py --ip 192.168.1.42 --list    # list files only
python transfer/fetch_log.py --ip 192.168.1.42 --file run_003.bin
python transfer/fetch_log.py --ip 192.168.1.42 --out runs/2025-06-01/
python transfer/fetch_log.py --ip 192.168.1.42 --delete run_001.bin
```

The script downloads the `.bin` file, converts it to `.csv`, and prints the path to load in the dashboard. The binary is deleted automatically (`--keep-bin` to retain it).

### Alternative — Physical SD Card Transfer

1. Power off the ESP32, remove the microSD card
2. Copy `run_XXX.bin` to the Dashboard folder
3. Convert manually:
```bash
python -c "
from data_loader import load_binary
df = load_binary('run_001.bin')
df.to_csv('run_001.csv', index=False)
print(len(df), 'records written')
"
```

---

## Dashboard Usage

```bash
cd Stridesynch/Dashboard
python app.py
# Open http://localhost:8050
```

**Test without hardware:**
```bash
cd Stridesynch/Dashboard/sample_data
python generate_sample.py 30    # 30-second synthetic run
cd ..
python app.py
# Enter: sample_data/sample_run.csv → click Load Run
```

**Loading a real run:**
1. Enter the full CSV path in the **Run CSV** field
2. Click **Load Run** — allow ~3 seconds for the skeleton animation to compute
3. All three panels populate automatically

### Panel 1 — 3D Skeleton Animation

Blue = left leg, orange = right leg. Hip at top (Y = 0), toes at bottom. Use the embedded **Play** button and scrub bar to control playback (0.5× real-time). Rotate and zoom with click-and-drag. Y axis = vertical, Z axis = sagittal (forward/back swing).

### Panel 2 — Joint Angles

Knee flexion and ankle dorsiflexion over time, left and right overlaid. 0° knee = full extension, 60°+ = peak swing flexion. Positive ankle = dorsiflexion, negative = plantarflexion. Hover for exact values.

### Panel 3 — Gait Metrics

Cadence, ground contact time, tibial shock, symmetry indices, and fatigue comparison. See [Gait Metrics](#gait-metrics) below. Optionally enter a Garmin `.fit` path for cross-validation.

---

## Garmin Comparison

Export a `.fit` file from Garmin Connect (**Activity → Settings → Export Original**) after a run wearing both devices. Enter the path in the **Garmin .fit** field before clicking Load Run.

```
Cadence:             Garmin: 168 spm  |  StrideSync: 170 spm  |  Agreement: 98.8%
Ground Contact Time: Garmin: 241 ms   |  StrideSync: 238 ms   |  Agreement: 98.7%
```

≥ 95% shown in green (validated). < 95% shown in amber (check calibration and sensor placement).

> **Note:** Garmin's `cadence` field is steps per minute for one foot. The comparison script multiplies by 2 for bilateral comparison with StrideSync.

---

## Gait Metrics

| Metric | Description | Typical range |
|---|---|---|
| Cadence | Steps per minute, computed from FSR heel strike intervals | 160–180 spm |
| Ground contact time | Heel strike to toe-off per step | 200–280 ms |
| Tibial shock | Peak resultant acceleration at proximal shank IMU | 5–12 g |
| Symmetry index | `\|L − R\| / ((L + R) / 2) × 100` for knee, ankle, cadence | < 5% normal |
| Fatigue comparison | Mean knee flexion angle, first 10% vs last 10% of run | Δ > 2° flagged |

A symmetry index > 10% warrants investigation — it can indicate injury compensation, dominant-limb bias, or fatigue-related asymmetry. The fatigue delta flags progressive changes in knee flexion consistent with neuromuscular fatigue and reduced leg stiffness late in a run.

---

## Troubleshooting

**"No IMUs found — check wiring"**
- Verify SDA/SCL on GPIO 21/22
- Check 5 kΩ pull-up resistors on SDA and SCL
- Verify PCA9548A A0/A1/A2 tied to GND (address 0x70)
- Power cycle and retry

**"SD init failed — check wiring"**
- SPI wiring: CLK=18, MISO=19, MOSI=23, CS=5
- Confirm FAT32 format
- Try a different SD card

**"Could not reach [IP]"**
- Confirm `WIFI_SSID` and `WIFI_PASS` in `main.cpp` are correct
- Laptop and ESP32 must be on the same network
- Check Serial monitor for the actual IP; `ping <IP>` to verify reachability

**"Load error" in dashboard**
- Confirm the CSV has a header row and 20 columns
- Re-run the binary converter and check for file corruption
- Binary file size must be a multiple of 70 bytes

**"Yaw is drifting"**
- Expected — no magnetometer is fitted
- Yaw is not used in any joint angle or metric calculation

**"Knee angle stuck near 0°"**
- IMUs may be loose on the segment
- Confirm calibration was done with sensors horizontal (flat)
- If enabling `Ki` in the Mahony filter, start at 0.01

**"Joint angles look wrong immediately after strapping on"**
- Sensors were not horizontal during calibration
- Recalibrate with the garment flat on a table, sensors face-up (see [Calibration Note](#calibration-note))

**"Tibial shock reads very low (< 3 g)"**
- Shank IMU has likely slipped distally toward the ankle
- Re-secure at 3–5 cm below the lateral tibial plateau

**"FSR reads pressure on wrong foot"**
- Check GPIO assignments: L-heel=32, L-fore=33, R-heel=34, R-fore=35
- Confirm each FSR is physically under the correct foot
