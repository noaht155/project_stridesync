from __future__ import annotations

import numpy as np
import pandas as pd

# Segment lengths in metres — update once Noah measures his leg
L_THIGH = 0.42
L_SHANK = 0.38
L_FOOT  = 0.26

HIP_SEPARATION = 0.20  # lateral distance between left and right hip origins


# ── Quaternion math ───────────────────────────────────────────────────────────

def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2),  2*(x*y - w*z),      2*(x*z + w*y)     ],
        [2*(x*y + w*z),        1 - 2*(x**2 + z**2), 2*(y*z - w*x)     ],
        [2*(x*z - w*y),        2*(y*z + w*x),       1 - 2*(x**2 + y**2)],
    ])


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def relative_angle_deg(q_distal: np.ndarray, q_proximal: np.ndarray) -> float:
    """Angle (degrees) between two segments via relative quaternion."""
    q_rel = quaternion_multiply(q_distal, quaternion_conjugate(q_proximal))
    q_rel = q_rel / np.linalg.norm(q_rel)
    return 2.0 * np.degrees(np.arccos(np.clip(abs(q_rel[0]), 0.0, 1.0)))


# ── Forward kinematics ────────────────────────────────────────────────────────

def reconstruct_skeleton(
    q_thigh: np.ndarray,
    q_shank: np.ndarray,
    q_foot:  np.ndarray,
    hip_pos: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    down      = np.array([0.0, -1.0, 0.0])
    knee_pos  = hip_pos  + (quaternion_to_rotation_matrix(q_thigh) @ down) * L_THIGH
    ankle_pos = knee_pos + (quaternion_to_rotation_matrix(q_shank) @ down) * L_SHANK
    toe_pos   = ankle_pos + (quaternion_to_rotation_matrix(q_foot)  @ down) * L_FOOT
    return hip_pos, knee_pos, ankle_pos, toe_pos


def _get_q(row: pd.Series, ch: int) -> np.ndarray:
    return np.array([row[f'ch{ch}_qw'], row[f'ch{ch}_qx'], row[f'ch{ch}_qy'], row[f'ch{ch}_qz']])


def get_skeleton_frame(
    wide_df: pd.DataFrame, idx: int
) -> tuple:
    """Return (left_pts, right_pts) for a single frame. O(1) per call."""
    row       = wide_df.iloc[idx]
    left_hip  = np.array([-HIP_SEPARATION / 2, 0.0, 0.0])
    right_hip = np.array([ HIP_SEPARATION / 2, 0.0, 0.0])
    try:
        left  = reconstruct_skeleton(_get_q(row, 2), _get_q(row, 1), _get_q(row, 0), left_hip)
        right = reconstruct_skeleton(_get_q(row, 5), _get_q(row, 4), _get_q(row, 3), right_hip)
    except (KeyError, ValueError):
        left = right = None
    return left, right


def build_skeleton_frames(wide_df: pd.DataFrame) -> list:
    """Build all frames — used by the matplotlib static exporter."""
    return [get_skeleton_frame(wide_df, i) for i in range(len(wide_df))]


# ── Joint angle computation ───────────────────────────────────────────────────

def compute_joint_angles(wide_df: pd.DataFrame) -> pd.DataFrame:
    """
    Return DataFrame with knee_L, ankle_L, knee_R, ankle_R columns (degrees).
    Channel map:  foot=0/3  shank=1/4  thigh=2/5  (L/R)
    """
    records = []
    for _, row in wide_df.iterrows():
        try:
            ankle_L = relative_angle_deg(_get_q(row, 0), _get_q(row, 1))
            knee_L  = relative_angle_deg(_get_q(row, 1), _get_q(row, 2))
            ankle_R = relative_angle_deg(_get_q(row, 3), _get_q(row, 4))
            knee_R  = relative_angle_deg(_get_q(row, 4), _get_q(row, 5))
        except (KeyError, ValueError):
            ankle_L = knee_L = ankle_R = knee_R = np.nan
        records.append({'knee_L': knee_L, 'ankle_L': ankle_L,
                        'knee_R': knee_R, 'ankle_R': ankle_R})
    return pd.DataFrame(records, index=wide_df.index)
