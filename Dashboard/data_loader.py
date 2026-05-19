from __future__ import annotations

import struct
import numpy as np
import pandas as pd
from pathlib import Path

COLUMNS = [
    'timestamp_us', 'channel',
    'qw', 'qx', 'qy', 'qz',
    'roll', 'pitch', 'yaw',
    'ax', 'ay', 'az',
    'gx', 'gy', 'gz',
    'fsr_heel_left', 'fsr_fore_left',
    'fsr_heel_right', 'fsr_fore_right',
    'gait_phase',
]

# Binary record: uint64 timestamp | uint8 channel | 13 float32 (q+euler+accel+gyro)
#                | 4 uint16 FSR | uint8 gait_phase  — 70 bytes total
_BINARY_FMT  = '<QB13f4HB'
_BINARY_SIZE = struct.calcsize(_BINARY_FMT)


def load_csv(filepath: str | Path) -> pd.DataFrame:
    df = pd.read_csv(filepath, names=COLUMNS, header=0)
    df['timestamp_us'] = df['timestamp_us'].astype(np.int64)
    df['channel']      = df['channel'].astype(int)
    return df.sort_values(['timestamp_us', 'channel']).reset_index(drop=True)


def load_binary(filepath: str | Path) -> pd.DataFrame:
    rows = []
    with open(filepath, 'rb') as f:
        while True:
            raw = f.read(_BINARY_SIZE)
            if len(raw) < _BINARY_SIZE:
                break
            rows.append(struct.unpack(_BINARY_FMT, raw))
    df = pd.DataFrame(rows, columns=COLUMNS)
    df['timestamp_us'] = df['timestamp_us'].astype(np.int64)
    df['channel']      = df['channel'].astype(int)
    return df.sort_values(['timestamp_us', 'channel']).reset_index(drop=True)


def pivot_wide(df: pd.DataFrame) -> pd.DataFrame:
    """One row per timestamp; per-channel IMU columns prefixed ch0_…ch5_."""
    imu_cols    = ['qw', 'qx', 'qy', 'qz', 'roll', 'pitch', 'yaw', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
    shared_cols = ['fsr_heel_left', 'fsr_fore_left', 'fsr_heel_right', 'fsr_fore_right', 'gait_phase']

    shared = df.groupby('timestamp_us')[shared_cols].first().reset_index()

    imu_pivot = df.pivot_table(
        index='timestamp_us', columns='channel', values=imu_cols, aggfunc='first'
    )
    imu_pivot.columns = [f'ch{int(ch)}_{field}' for field, ch in imu_pivot.columns]
    imu_pivot = imu_pivot.reset_index()

    wide = shared.merge(imu_pivot, on='timestamp_us')
    wide['time_s'] = (wide['timestamp_us'] - wide['timestamp_us'].iloc[0]) / 1e6
    return wide.reset_index(drop=True)
