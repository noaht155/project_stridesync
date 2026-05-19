from __future__ import annotations

import numpy as np
import pandas as pd

# MPU-6050 at default ±2g range
_LSB_PER_G = 16384.0

FSR_THRESHOLD = 2000  # ADC counts — tune after hardware characterisation


def detect_heel_strikes(fsr_heel: pd.Series, timestamps: pd.Series) -> list:
    """Rising-edge threshold crossings → heel strike timestamps (us)."""
    above   = fsr_heel > FSR_THRESHOLD
    strikes = []
    for i in range(1, len(above)):
        if above.iloc[i] and not above.iloc[i - 1]:
            strikes.append(int(timestamps.iloc[i]))
    return strikes


def detect_toe_offs(fsr_heel: pd.Series, timestamps: pd.Series) -> list:
    """Falling-edge threshold crossings → toe-off timestamps (us)."""
    above = fsr_heel > FSR_THRESHOLD
    offs  = []
    for i in range(1, len(above)):
        if not above.iloc[i] and above.iloc[i - 1]:
            offs.append(int(timestamps.iloc[i]))
    return offs


def compute_cadence(heel_strikes_us: list) -> float | None:
    """Steps per minute from heel strike timestamps in microseconds."""
    if len(heel_strikes_us) < 2:
        return None
    intervals_s = np.diff(heel_strikes_us) / 1e6
    return round(60.0 / float(np.mean(intervals_s)), 1)


def compute_ground_contact_time(heel_strikes_us: list, toe_offs_us: list) -> float | None:
    """Mean ground contact time in milliseconds."""
    gcts = []
    for hs in heel_strikes_us:
        after = [t for t in toe_offs_us if t > hs]
        if after:
            gcts.append((after[0] - hs) / 1e3)
    return round(float(np.mean(gcts)), 1) if gcts else None


def compute_symmetry_index(left_vals, right_vals) -> float | None:
    """Symmetry index (%) between left and right. 0 = perfect symmetry."""
    l, r = float(np.nanmean(left_vals)), float(np.nanmean(right_vals))
    if (l + r) == 0:
        return None
    return round(abs(l - r) / ((l + r) / 2.0) * 100.0, 2)


def compute_tibial_shock(wide_df: pd.DataFrame, side: str = 'L') -> float | None:
    """Peak resultant tibial acceleration in g. Side 'L'=ch1, 'R'=ch4."""
    ch   = 1 if side == 'L' else 4
    cols = [f'ch{ch}_ax', f'ch{ch}_ay', f'ch{ch}_az']
    if not all(c in wide_df.columns for c in cols):
        return None
    resultant = np.sqrt(
        wide_df[cols[0]]**2 + wide_df[cols[1]]**2 + wide_df[cols[2]]**2
    )
    return round(float(resultant.max()) / _LSB_PER_G, 2)


def _mean_knee_angle(wide_df: pd.DataFrame) -> float | None:
    if 'knee_L' not in wide_df.columns:
        return None
    return float(np.nanmean(wide_df[['knee_L', 'knee_R']].values))


def compute_fatigue_comparison(wide_df: pd.DataFrame, pct: float = 0.10) -> dict:
    """
    Compare mean knee angle over first vs last `pct` of the run.
    Returns {'first': value, 'last': value, 'delta': value}.
    """
    n    = len(wide_df)
    cut  = max(1, int(n * pct))
    first = _mean_knee_angle(wide_df.iloc[:cut])
    last  = _mean_knee_angle(wide_df.iloc[-cut:])
    delta = round(last - first, 2) if (first is not None and last is not None) else None
    return {
        'first_deg': round(first, 2) if first is not None else None,
        'last_deg':  round(last,  2) if last  is not None else None,
        'delta_deg': delta,
    }


def summary_metrics(wide_df: pd.DataFrame) -> dict:
    """Compute all gait metrics from a wide-format DataFrame."""
    m  = {}
    ts = wide_df['timestamp_us']

    for side, heel_col in [('L', 'fsr_heel_left'), ('R', 'fsr_heel_right')]:
        if heel_col not in wide_df.columns:
            continue
        strikes = detect_heel_strikes(wide_df[heel_col], ts)
        offs    = detect_toe_offs(wide_df[heel_col], ts)
        m[f'cadence_{side}']        = compute_cadence(strikes)
        m[f'gct_ms_{side}']         = compute_ground_contact_time(strikes, offs)
        m[f'tibial_shock_{side}_g'] = compute_tibial_shock(wide_df, side)

    if 'knee_L' in wide_df.columns and 'knee_R' in wide_df.columns:
        m['knee_symmetry_pct']  = compute_symmetry_index(wide_df['knee_L'], wide_df['knee_R'])
    if 'ankle_L' in wide_df.columns and 'ankle_R' in wide_df.columns:
        m['ankle_symmetry_pct'] = compute_symmetry_index(wide_df['ankle_L'], wide_df['ankle_R'])

    cL, cR = m.get('cadence_L'), m.get('cadence_R')
    if cL and cR:
        m['cadence_symmetry_pct'] = round(abs(cL - cR) / ((cL + cR) / 2.0) * 100.0, 2)

    m['fatigue'] = compute_fatigue_comparison(wide_df)
    return m
