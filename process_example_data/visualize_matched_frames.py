"""
Visualize matched range-Doppler and camera frames from output_apple_moving.

Expected inputs:
- output_apple_moving/range_doppler_per_frame/frame_XXXX.csv
- output_apple_moving/cam_images/imageN.npy

Matching rule:
- Numeric frame index extracted from filenames (e.g., frame_0032 <-> image32.npy).

Controls:
- Right / n: next frame
- Left  / p: previous frame
- Space: play/pause
- q or Esc: quit
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd

from processing import EPSILON

SCRIPT_DIR = Path(__file__).resolve().parent

RD_PATTERN = re.compile(r"frame_(\d+)\.csv$")
CAM_PATTERN = re.compile(r"image(\d+)\.npy$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize matched range-Doppler CSV and camera NPY frames."
    )
    parser.add_argument(
        "folder",
        type=str,
        help="Output folder name (e.g. 'output_tape') containing 'range_doppler_per_frame/' and 'cam_images/'.",
    )
    parser.add_argument(
        "--rd-dir",
        type=Path,
        default=None,
        help="Override directory with frame_XXXX.csv files (defaults to <folder>/range_doppler_per_frame).",
    )
    parser.add_argument(
        "--cam-dir",
        type=Path,
        default=None,
        help="Override directory with imageN.npy files (defaults to <folder>/cam_images).",
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        default=0,
        help="First frame index to include.",
    )
    parser.add_argument(
        "--end-frame",
        type=int,
        default=None,
        help="Last frame index to include (inclusive).",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=4.0,
        help="Autoplay speed in frames per second.",
    )
    parser.add_argument(
        "--no-autoplay",
        action="store_true",
        help="Start paused instead of autoplaying.",
    )
    return parser.parse_args()


def _index_from_name(path: Path, pattern: re.Pattern[str]) -> int | None:
    match = pattern.search(path.name)
    if not match:
        return None
    return int(match.group(1))


def collect_frames(rd_dir: Path, cam_dir: Path, start: int, end: int | None):
    if not rd_dir.is_dir():
        raise FileNotFoundError(f"Range-Doppler directory not found: {rd_dir}")
    if not cam_dir.is_dir():
        raise FileNotFoundError(f"Camera directory not found: {cam_dir}")

    rd_by_idx: dict[int, Path] = {}
    for p in rd_dir.glob("frame_*.csv"):
        idx = _index_from_name(p, RD_PATTERN)
        if idx is not None:
            rd_by_idx[idx] = p

    cam_by_idx: dict[int, Path] = {}
    for p in cam_dir.glob("image*.npy"):
        idx = _index_from_name(p, CAM_PATTERN)
        if idx is not None:
            cam_by_idx[idx] = p

    common = sorted(set(rd_by_idx) & set(cam_by_idx))
    if end is None:
        selected = [i for i in common if i >= start]
    else:
        selected = [i for i in common if start <= i <= end]

    if not selected:
        raise ValueError(
            "No matched frames found. Check naming and start/end range."
        )

    pairs = [(i, rd_by_idx[i], cam_by_idx[i]) for i in selected]
    return pairs


def load_rd_grid(csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    df = pd.read_csv(csv_path)
    required = {"doppler_bin", "range_bin", "velocity_mps", "range_m", "magnitude_db"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path.name} missing columns: {sorted(missing)}")

    doppler_order = np.sort(df["doppler_bin"].unique())
    range_order = np.sort(df["range_bin"].unique())

    vel_axis = (
        df.sort_values("doppler_bin")
        .drop_duplicates("doppler_bin")
        .set_index("doppler_bin")
        .loc[doppler_order, "velocity_mps"]
        .to_numpy()
    )
    range_axis = (
        df.sort_values("range_bin")
        .drop_duplicates("range_bin")
        .set_index("range_bin")
        .loc[range_order, "range_m"]
        .to_numpy()
    )

    pivot = df.pivot(index="doppler_bin", columns="range_bin", values="magnitude_db")
    grid = pivot.loc[doppler_order, range_order].to_numpy()
    return grid, vel_axis, range_axis


def load_peak_rae(csv_path: Path) -> tuple[float, float, float, float]:
    df = pd.read_csv(csv_path)
    required = {"range_m", "azimuth_deg", "elevation_deg", "magnitude_db"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path.name} missing columns: {sorted(missing)}")

    if "is_target" in df.columns:
        targets = df[(df["is_target"] == 1) & (df["range_m"] > 0.5)]
    else:
        targets = df[df["range_m"] > 0.5]
    if targets.empty:
        return (0.0, 0.0, 0.0, 0.0)

    # peak_row = df.loc[df["magnitude_db"].idxmax()]
    peak_row = targets.loc[targets["magnitude_db"].idxmax()]
    velocity = float(peak_row.get("velocity_mps", 0.0))
    return (
        float(peak_row["range_m"]),
        float(peak_row["azimuth_deg"]),
        float(peak_row["elevation_deg"]),
        velocity,
    )
    
def get_interp_range_doppler(self, rd_cube, r_idx, d_idx):
        # Range interpolation
        r_prev, r_next = r_idx - 1, r_idx + 1
        y0 = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_idx])**2))
        y_l = np.sqrt(np.sum(np.abs(rd_cube[r_prev, :, :, d_idx])**2))
        y_r = np.sqrt(np.sum(np.abs(rd_cube[r_next, :, :, d_idx])**2))

        interp_r = (
            self.range_axis[r_idx]
            + (self.range_axis[1] - self.range_axis[0])
            * ((y_l - y_r) / (2 * (y_l + y_r - 2 * y0 + EPSILON)))
        ).item()

        # Doppler interpolation
        d_prev = (d_idx - 1) % self.num_chirp_loops
        d_next = (d_idx + 1) % self.num_chirp_loops
        v_l = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_prev])**2))
        v_r = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_next])**2))
        v0 = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_idx])**2))
        
        interp_v = (
            self.velocity_axis[d_idx]
            + (self.velocity_axis[1] - self.velocity_axis[0])
            * ((v_l - v_r) / (2 * (v_l + v_r - 2 * v0 + EPSILON)))
        ).item()
        
        return (interp_r, interp_v)

def main() -> None:
    args = parse_args()

    folder_path = Path(args.folder)
    if not folder_path.is_absolute():
        folder_path = SCRIPT_DIR / folder_path

    rd_dir = args.rd_dir if args.rd_dir is not None else folder_path / "range_doppler_per_frame"
    cam_dir = args.cam_dir if args.cam_dir is not None else folder_path / "cam_images"

    pairs = collect_frames(rd_dir, cam_dir, args.start_frame, args.end_frame)

    print(
        f"Found {len(pairs)} matched frames "
        f"(idx {pairs[0][0]} to {pairs[-1][0]})."
    )

    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(2, 3, height_ratios=[2, 1])
    ax_rd = fig.add_subplot(gs[0, 0])
    ax_cam = fig.add_subplot(gs[0, 1])
    ax_range = fig.add_subplot(gs[1, 0])
    ax_azimuth = fig.add_subplot(gs[1, 1])
    ax_elevation = fig.add_subplot(gs[1, 2])
    fig.suptitle("Range-Doppler, Camera, and RAE Viewer")
    plt.subplots_adjust(bottom=0.12, hspace=0.4)

    state = {"pos": 0, "playing": not args.no_autoplay}

    # Initialize artists once so subplot geometry stays fixed across frames.
    first_idx, first_rd_path, first_cam_path = pairs[0]
    first_grid, first_vel_axis, first_range_axis = load_rd_grid(first_rd_path)
    first_cam_img = np.load(first_cam_path)

    rd_extent = [
        float(first_vel_axis[0]),
        float(first_vel_axis[-1]),
        float(first_range_axis[0]),
        float(first_range_axis[-1]),
    ]

    rd_im = ax_rd.imshow(
        first_grid.T,
        aspect="auto",
        origin="lower",
        extent=rd_extent,
        cmap="viridis",
    )
    
    # Scatter plot for detected target
    target_scatter = ax_rd.scatter([], [], color="red", marker="o", s=16)
    
    ax_rd.set_xlabel("Velocity (m/s)")
    ax_rd.set_ylabel("Range (m)")
    cbar = fig.colorbar(rd_im, ax=ax_rd, pad=0.01)
    cbar.set_label("Magnitude (dB)")

    cam_im = ax_cam.imshow(first_cam_img)
    ax_cam.axis("off")

    frame_ids = np.array([idx for idx, _, _ in pairs], dtype=int)
    peak_range: list[float] = []
    peak_azimuth: list[float] = []
    peak_elevation: list[float] = []
    peak_velocity: list[float] = []
    for _, rd_path, _ in pairs:
        r_m, az_deg, el_deg, vel_mps = load_peak_rae(rd_path)
        # ax_rd.plot(interp_v, interp_r, 'ro', markersize=4)
        peak_range.append(r_m)
        peak_azimuth.append(az_deg)
        peak_elevation.append(el_deg)
        peak_velocity.append(vel_mps)

    ax_range.plot(frame_ids, peak_range, label="Range (m)", color="tab:blue", linewidth=1.8)
    current_frame_line_range = ax_range.axvline(
        frame_ids[0], color="tab:red", linestyle="--", linewidth=1.5, label="Current frame"
    )
    ax_range.set_xlabel("Frame index")
    ax_range.set_ylabel("Range (m)")
    ax_range.set_title("Peak-bin Range per frame")
    ax_range.grid(True, alpha=0.3)
    ax_range.legend(loc="lower right")

    ax_azimuth.plot(frame_ids, peak_azimuth, label="Azimuth (deg)", color="tab:orange", linewidth=1.8)
    current_frame_line_azimuth = ax_azimuth.axvline(
        frame_ids[0], color="tab:red", linestyle="--", linewidth=1.5, label="Current frame"
    )
    ax_azimuth.set_xlabel("Frame index")
    ax_azimuth.set_ylabel("Azimuth (deg)")
    ax_azimuth.set_title("Peak-bin Azimuth per frame")
    ax_azimuth.grid(True, alpha=0.3)
    ax_azimuth.legend(loc="lower right")

    ax_elevation.plot(frame_ids, peak_elevation, label="Elevation (deg)", color="tab:green", linewidth=1.8)
    current_frame_line_elevation = ax_elevation.axvline(
        frame_ids[0], color="tab:red", linestyle="--", linewidth=1.5, label="Current frame"
    )
    ax_elevation.set_xlabel("Frame index")
    ax_elevation.set_ylabel("Elevation (deg)")
    ax_elevation.set_title("Peak-bin Elevation per frame")
    ax_elevation.grid(True, alpha=0.3)
    ax_elevation.legend(loc="lower right")

    fig.text(
        0.5,
        0.02,
        "Keys: Left/Right (prev/next), Space (play/pause), q or Esc (quit)",
        ha="center",
        fontsize=10,
    )

    def draw_current() -> None:
        idx, rd_path, cam_path = pairs[state["pos"]]
        rd_grid, _vel_axis, _range_axis = load_rd_grid(rd_path)
        vmax = float(rd_grid.max())
        vmin = vmax - 40.0  # or however you want to derive it

        cam_img = np.load(cam_path)

        rd_im.set_data(rd_grid.T)
        rd_im.set_clim(
            vmin=vmin, 
            vmax=vmax
        )
        
        # Update target scatter position
        target_vel = peak_velocity[state["pos"]]
        target_range = peak_range[state["pos"]]
        if target_range > 0:  # Only show if valid target detected
            target_scatter.set_offsets([[target_vel, target_range]])
        else:
            target_scatter.set_offsets(np.empty((0, 2)))
        
        # interp_r, interp_v = get_interp_range_doppler()
        # ax_rd.plot(interp_v, interp_r, 'ro', markersize=4)
        
        cam_im.set_data(cam_img)
        current_frame_line_range.set_xdata([idx, idx])
        current_frame_line_azimuth.set_xdata([idx, idx])
        current_frame_line_elevation.set_xdata([idx, idx])
        ax_rd.set_title(f"Range-Doppler (frame {idx})")
        ax_cam.set_title(f"Camera image (frame {idx})")
        ax_range.set_title(f"Peak-bin Range per frame (current: {peak_range[state['pos']]:.2f} m)")
        ax_azimuth.set_title(f"Peak-bin Azimuth per frame (current: {peak_azimuth[state['pos']]:.2f} deg)")
        ax_elevation.set_title(f"Peak-bin Elevation per frame (current: {peak_elevation[state['pos']]:.2f} deg)")
        fig.canvas.draw_idle()

    def step_next(_frame_number: int | None = None):
        if state["playing"]:
            state["pos"] = (state["pos"] + 1) % len(pairs)
            draw_current()

    def on_key(event) -> None:
        key = event.key.lower() if event.key else ""
        if key in {"right", "n"}:
            state["pos"] = min(state["pos"] + 1, len(pairs) - 1)
            draw_current()
        elif key in {"left", "p"}:
            state["pos"] = max(state["pos"] - 1, 0)
            draw_current()
        elif key == " ":
            state["playing"] = not state["playing"]
        elif key in {"q", "escape"}:
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    draw_current()

    interval_ms = max(1, int(1000 / max(args.fps, 1e-6)))
    _anim = FuncAnimation(fig, step_next, interval=interval_ms)
    plt.show()


if __name__ == "__main__":
    main()
