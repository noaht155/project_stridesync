"""Garmin FIT file parsing and comparison against StrideSync metrics."""
from __future__ import annotations

import numpy as np
import pandas as pd


def load_fit(filepath: str) -> pd.DataFrame:
    try:
        from fitparse import FitFile
    except ImportError:
        raise ImportError("Install fitparse:  pip install fitparse")

    fit     = FitFile(filepath)
    records = []
    for msg in fit.get_messages('record'):
        row = {field.name: field.value for field in msg}
        records.append(row)

    df = pd.DataFrame(records)
    if 'timestamp' in df.columns:
        df = df.sort_values('timestamp').reset_index(drop=True)
    return df


def extract_garmin_metrics(garmin_df: pd.DataFrame) -> dict:
    """Extract cadence, GCT, and vertical oscillation from a Garmin record DataFrame."""
    m = {}

    if 'cadence' in garmin_df.columns:
        # Garmin 'cadence' field is steps/min for one foot; multiply by 2 for bilateral SPM
        m['cadence_spm'] = float(garmin_df['cadence'].dropna().mean()) * 2

    if 'ground_contact_time' in garmin_df.columns:
        m['gct_ms'] = float(garmin_df['ground_contact_time'].dropna().mean())

    if 'vertical_oscillation' in garmin_df.columns:
        m['vertical_oscillation_mm'] = float(garmin_df['vertical_oscillation'].dropna().mean())

    return m


def compare_cadence(garmin_metrics: dict, stridesync_metrics: dict) -> dict | None:
    g_cad = garmin_metrics.get('cadence_spm')
    ss_vals = [v for v in [stridesync_metrics.get('cadence_L'),
                            stridesync_metrics.get('cadence_R')] if v is not None]
    if not g_cad or not ss_vals:
        return None
    ss_cad    = float(np.mean(ss_vals))
    agreement = (1.0 - abs(g_cad - ss_cad) / g_cad) * 100.0
    return {
        'garmin_spm':     round(g_cad, 1),
        'stridesync_spm': round(ss_cad, 1),
        'agreement_pct':  round(agreement, 1),
    }


def compare_gct(garmin_metrics: dict, stridesync_metrics: dict) -> dict | None:
    g_gct   = garmin_metrics.get('gct_ms')
    ss_vals = [v for v in [stridesync_metrics.get('gct_ms_L'),
                            stridesync_metrics.get('gct_ms_R')] if v is not None]
    if not g_gct or not ss_vals:
        return None
    ss_gct    = float(np.mean(ss_vals))
    agreement = (1.0 - abs(g_gct - ss_gct) / g_gct) * 100.0
    return {
        'garmin_ms':     round(g_gct, 1),
        'stridesync_ms': round(ss_gct, 1),
        'agreement_pct': round(agreement, 1),
    }
