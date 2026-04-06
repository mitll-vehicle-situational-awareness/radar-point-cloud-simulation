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


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_ROOT = SCRIPT_DIR / "output_apple_moving"
DEFAULT_RD_DIR = DEFAULT_ROOT / "range_doppler_per_frame"
DEFAULT_CAM_DIR = DEFAULT_ROOT / "cam_images"

RD_PATTERN = re.compile(r"frame_(\d+)\.csv$")
CAM_PATTERN = re.compile(r"image(\d+)\.npy$")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize matched range-Doppler CSV and camera NPY frames."
    )
    parser.add_argument(
        "--rd-dir",
        type=Path,
        default=DEFAULT_RD_DIR,
        help="Directory with frame_XXXX.csv files.",
    )
    parser.add_argument(
        "--cam-dir",
        type=Path,
        default=DEFAULT_CAM_DIR,
        help="Directory with imageN.npy files.",
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


def load_peak_rae(csv_path: Path) -> tuple[float, float, float]:
    df = pd.read_csv(csv_path)
    required = {"range_m", "azimuth_deg", "elevation_deg", "magnitude_db"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path.name} missing columns: {sorted(missing)}")

    peak_row = df.loc[df["magnitude_db"].idxmax()]
    return (
        float(peak_row["range_m"]),
        float(peak_row["azimuth_deg"]),
        float(peak_row["elevation_deg"]),
    )


def main() -> None:
    args = parse_args()
    pairs = collect_frames(args.rd_dir, args.cam_dir, args.start_frame, args.end_frame)

    print(
        f"Found {len(pairs)} matched frames "
        f"(idx {pairs[0][0]} to {pairs[-1][0]})."
    )

    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[3, 2])
    ax_rd = fig.add_subplot(gs[0, 0])
    ax_cam = fig.add_subplot(gs[0, 1])
    ax_rae = fig.add_subplot(gs[1, :])
    fig.suptitle("Range-Doppler, Camera, and RAE Viewer")
    plt.subplots_adjust(bottom=0.12)

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
        cmap="turbo",
    )
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
    for _, rd_path, _ in pairs:
        r_m, az_deg, el_deg = load_peak_rae(rd_path)
        peak_range.append(r_m)
        peak_azimuth.append(az_deg)
        peak_elevation.append(el_deg)

    ax_rae.plot(frame_ids, peak_range, label="Range (m)", color="tab:blue", linewidth=1.8)
    ax_rae.plot(
        frame_ids,
        peak_azimuth,
        label="Azimuth (deg)",
        color="tab:orange",
        linewidth=1.8,
    )
    ax_rae.plot(
        frame_ids,
        peak_elevation,
        label="Elevation (deg)",
        color="tab:green",
        linewidth=1.8,
    )
    current_frame_line = ax_rae.axvline(
        frame_ids[0], color="tab:red", linestyle="--", linewidth=1.5, label="Current frame"
    )
    ax_rae.set_xlabel("Frame index")
    ax_rae.set_ylabel("Value")
    ax_rae.set_title("Peak-bin Range / Azimuth / Elevation per frame")
    ax_rae.grid(True, alpha=0.3)
    ax_rae.legend(loc="upper right")

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
        cam_img = np.load(cam_path)

        rd_im.set_data(rd_grid.T)
        cam_im.set_data(cam_img)
        current_frame_line.set_xdata([idx, idx])
        ax_rd.set_title(f"Range-Doppler (frame {idx})")
        ax_cam.set_title(f"Camera image (frame {idx})")
        ax_rae.set_title(
            "Peak-bin Range / Azimuth / Elevation per frame "
            f"(range={peak_range[state['pos']]:.2f} m, "
            f"az={peak_azimuth[state['pos']]:.2f} deg, "
            f"el={peak_elevation[state['pos']]:.2f} deg)"
        )
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
