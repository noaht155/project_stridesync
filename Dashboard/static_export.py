"""
Static skeleton animation export using matplotlib + mpl_toolkits.mplot3d.
Run standalone to save a GIF from a CSV log file.

Usage:
    python static_export.py run.csv skeleton.gif
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 — registers 3d projection
from matplotlib.animation import FuncAnimation

from data_loader import load_csv, pivot_wide
from kinematics import build_skeleton_frames, L_THIGH, L_SHANK, L_FOOT, HIP_SEPARATION


def export_skeleton_gif(wide_df, output_path: str, fps: int = 25, stride: int = 8):
    """
    Save a 3D bilateral skeleton animation as a GIF.

    stride — take every Nth frame from wide_df to keep file size manageable.
             At 200 Hz source, stride=8 → 25 Hz animation.
    """
    all_frames = build_skeleton_frames(wide_df)
    frames     = all_frames[::stride]
    n_frames   = len(frames)

    fig = plt.figure(figsize=(5, 9))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-1.2, 0.1)
    ax.set_zlim(-0.3, 0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m, up)')
    ax.set_zlabel('Z (m)')
    ax.set_title('StrideSync — Bilateral Skeleton')

    left_line,  = ax.plot([], [], [], 'b-o', linewidth=2, markersize=5, label='Left')
    right_line, = ax.plot([], [], [], color='darkorange', marker='o',
                          linewidth=2, markersize=5, label='Right')
    time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, fontsize=9)
    ax.legend(loc='upper right')

    def _set_line(line, pts):
        if pts is None:
            line.set_data([], [])
            line.set_3d_properties([])
            return
        hip, knee, ankle, toe = pts
        xs = [p[0] for p in (hip, knee, ankle, toe)]
        ys = [p[1] for p in (hip, knee, ankle, toe)]
        zs = [p[2] for p in (hip, knee, ankle, toe)]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    dt_s = wide_df['time_s'].iloc[stride] if 'time_s' in wide_df.columns else stride / 200.0

    def update(i):
        left, right = frames[i]
        _set_line(left_line,  left)
        _set_line(right_line, right)
        time_text.set_text(f't = {i * dt_s:.2f} s')
        return left_line, right_line, time_text

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 // fps, blit=False)
    anim.save(output_path, writer='pillow', fps=fps)
    plt.close(fig)
    print(f"Saved → {output_path}  ({n_frames} frames @ {fps} fps)")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python static_export.py <run.csv> <output.gif>")
        sys.exit(1)

    csv_path = sys.argv[1]
    out_path = sys.argv[2]

    df   = load_csv(csv_path)
    wide = pivot_wide(df)
    export_skeleton_gif(wide, out_path)
