"""
Generate synthetic StrideSync CSV data for dashboard testing.
Produces realistic sagittal-plane running kinematics at 200 Hz.

Coordinate convention matches the Mahony filter output:
  - Quaternion represents absolute segment orientation in world frame
  - Z up in world frame → gravity on +Z axis
  - Sagittal rotation axis = X

Target joint angles (distance running, ~170 spm):
  - Knee flexion:       0° (full extension at heel strike) → 60° (peak swing)
  - Ankle dorsiflexion: -25° (push-off plantarflex) → +10° (mid-stance dorsiflex)
  - Hip flex/ext:       -15° (extension at push-off) → +30° (flexion at swing)

Usage:
    python generate_sample.py          # 30-second run
    python generate_sample.py 60       # 60-second run
"""
import sys
import numpy as np
import pandas as pd
from pathlib import Path

SAMPLE_RATE  = 200    # Hz
CADENCE_SPM  = 170    # steps per minute (bilateral)
CHANNELS     = 6


def _rotation_quat(axis, angle_rad):
    ax  = np.asarray(axis, dtype=float)
    ax /= np.linalg.norm(ax)
    s   = np.sin(angle_rad / 2.0)
    return np.array([np.cos(angle_rad / 2.0), *(ax * s)])


def generate(duration_s: int = 30) -> pd.DataFrame:
    n           = duration_s * SAMPLE_RATE
    t           = np.arange(n) / SAMPLE_RATE
    stride_freq = CADENCE_SPM / 60.0 / 2.0   # one full stride (both steps) per cycle

    rows = []
    for i, ts in enumerate(t):
        timestamp_us = int(ts * 1e6)

        # Left and right legs are antiphase (contralateral gait)
        phase_L = 2.0 * np.pi * stride_freq * ts
        phase_R = phase_L + np.pi

        # ── Gait phase (0=stance ~60%, 1=swing ~40%) ──────────────────────
        cycle_L = (ts * stride_freq) % 1.0
        gait_phase = 0 if cycle_L < 0.60 else 1

        # ── FSR ───────────────────────────────────────────────────────────
        # Heel dominant during early stance, forefoot during late stance
        def fsr_stance(ph):
            c = (ph / (2 * np.pi)) % 1.0
            if c < 0.60:
                heel = int(3200 * np.sin(np.pi * c / 0.60) ** 2)
                fore = int(2400 * np.sin(np.pi * max(c - 0.15, 0) / 0.45) ** 2)
                return heel, fore
            return 0, 0

        fsr_hl, fsr_fl = fsr_stance(phase_L)
        fsr_hr, fsr_fr = fsr_stance(phase_R)

        for ch in range(CHANNELS):
            side = 'L' if ch < 3 else 'R'
            seg  = ch % 3          # 0=Foot  1=Shank  2=Thigh
            ph   = phase_L if side == 'L' else phase_R

            # ── Absolute segment angles in world frame (sagittal, degrees) ──
            #
            # Hip flex/ext (thigh absolute):
            #   +30° at peak swing, -15° at push-off
            #   Sinusoid: mean=7.5°, amp=22.5°
            if seg == 2:  # Thigh
                angle_deg = 7.5 + 22.5 * np.sin(ph)

            # Shank absolute = thigh angle + knee angle
            # Knee angle target: 5° at heel strike → 60° at peak swing → 5° at next heel strike
            #   Approximated as 32.5 - 27.5*cos(ph) → ranges 5° to 60°
            elif seg == 1:  # Shank
                thigh_deg = 7.5 + 22.5 * np.sin(ph)
                knee_deg  = 32.5 - 27.5 * np.cos(ph)   # 5° → 60°
                angle_deg = thigh_deg + knee_deg

            # Foot absolute = shank angle - ankle angle
            # Ankle target: 10° dorsiflex at midstance, -25° plantarflex at push-off
            #   Approximated with a phase-shifted sinusoid
            else:  # Foot (seg == 0)
                thigh_deg = 7.5 + 22.5 * np.sin(ph)
                knee_deg  = 32.5 - 27.5 * np.cos(ph)
                shank_deg = thigh_deg + knee_deg
                # Ankle: dorsiflexion peaks slightly after midstance
                ankle_deg = -7.5 + 17.5 * np.sin(ph - 0.4)   # -25° to +10°
                angle_deg = shank_deg - ankle_deg

            # ── Quaternion from sagittal rotation ─────────────────────────
            q = _rotation_quat([1, 0, 0], np.radians(angle_deg))
            q /= np.linalg.norm(q)
            w, x, y, z = q

            # ── Euler angles ──────────────────────────────────────────────
            roll  = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2)))
            pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0)))
            yaw   = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2)))

            # ── Synthetic IMU data (LSB at ±2g, 16384 LSB/g) ─────────────
            noise = 100
            ax_val = int(np.random.normal(0, noise))
            ay_val = int(np.random.normal(0, noise))
            az_val = int(np.random.normal(16384, noise))  # gravity on Z

            # Gyro approximates d(angle)/dt in deg/s
            gyro_amp = angle_deg * 2.0
            gx_val = float(np.random.normal(gyro_amp, 1.5))
            gy_val = float(np.random.normal(0, 0.8))
            gz_val = float(np.random.normal(0, 0.8))

            rows.append({
                'timestamp_us':   timestamp_us,
                'channel':        ch,
                'qw': round(float(w), 6), 'qx': round(float(x), 6),
                'qy': round(float(y), 6), 'qz': round(float(z), 6),
                'roll':  round(roll,  3),
                'pitch': round(pitch, 3),
                'yaw':   round(yaw,   3),
                'ax': ax_val, 'ay': ay_val, 'az': az_val,
                'gx': round(gx_val, 3), 'gy': round(gy_val, 3), 'gz': round(gz_val, 3),
                'fsr_heel_left':  fsr_hl,
                'fsr_fore_left':  fsr_fl,
                'fsr_heel_right': fsr_hr,
                'fsr_fore_right': fsr_fr,
                'gait_phase':     gait_phase,
            })

    return pd.DataFrame(rows)


if __name__ == '__main__':
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    out      = Path(__file__).parent / 'sample_run.csv'
    df       = generate(duration)
    df.to_csv(out, index=False)
    print(f"Generated {len(df):,} rows ({duration}s @ {SAMPLE_RATE}Hz, {CHANNELS} channels) → {out}")
