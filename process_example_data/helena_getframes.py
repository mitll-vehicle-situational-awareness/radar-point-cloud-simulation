"""
Load per-frame radar bins from output_apple_moving/radar, compute range–Doppler
maps (same pipeline as the original script), and write one CSV per frame.

Each radarN.bin is one frame (48 chirps, 4 RX, 256 ADC — matches OUTPUT_BIN_CONFIG).
Processes the first 98 frames: radar0.bin … radar97.bin.

Output: output_apple_moving/range_doppler_per_frame/frame_XXXX.csv
"""

from __future__ import annotations

import argparse
import contextlib
import io
from pathlib import Path

import numpy as np
import pandas as pd

from processing import EPSILON, RadarSensor

# Paths (script lives in acd_simulation/)
_SCRIPT_DIR = Path(__file__).resolve().parent

NUM_FRAMES_TO_EXPORT = 10
MIN_RANGE = 0 # meters
MAX_RANGE = 2 # meters

# OUTPUT_BIN_CONFIG — 48 chirps/frame, 4 RX, 256 ADC (TDM-MIMO with 3 TX -> 16 loops)
CONFIG_TYPE = "OUTPUT_BIN_CONFIG"

if CONFIG_TYPE == "THE_REAL_CONFIG":
    NUM_ADC_SAMPLES = 256
    NUM_RX = 4
    NUM_TX = 3
    NUM_CHIRPS_PER_FRAME = 96
    NUM_CHIRP_LOOPS = NUM_CHIRPS_PER_FRAME // NUM_TX
    RANGE_RESOLUTION = 0.044
    VELOCITY_RESOLUTION = 1.26
    FRAME_TIME_LENGTH = 100
elif CONFIG_TYPE == "NEW_CONFIG":
    NUM_ADC_SAMPLES = 249
    NUM_RX = 4
    NUM_TX = 3
    NUM_CHIRPS_PER_FRAME = 81
    NUM_CHIRP_LOOPS = NUM_CHIRPS_PER_FRAME // NUM_TX
    RANGE_RESOLUTION = 0.044
    VELOCITY_RESOLUTION = 0.13
    FRAME_TIME_LENGTH = 100
elif CONFIG_TYPE == "OUTPUT_BIN_CONFIG":
    NUM_ADC_SAMPLES = 256
    NUM_RX = 4
    NUM_TX = 3
    NUM_CHIRPS_PER_FRAME = 48
    NUM_CHIRP_LOOPS = NUM_CHIRPS_PER_FRAME // NUM_TX
    RANGE_RESOLUTION = 0.044
    VELOCITY_RESOLUTION = 1.26
    FRAME_TIME_LENGTH = 100


def _read_frame_with_processing_bin_logic(bin_path: Path) -> np.ndarray:
    """
    Read one radar .bin using the same IIQQ->IQIQ decode/reshape logic used by
    process_rda_to_csv.py / processing.py flow.
    Returns one frame shaped [tx, chirp_loop, rx, adc].
    """
    raw_data = np.fromfile(bin_path, dtype=np.int16)
    if raw_data.size == 0:
        raise ValueError(f"Empty radar bin: {bin_path}")

    # Align to IIQQ packet width.
    if raw_data.size % 4 != 0:
        raw_data = raw_data[: (raw_data.size // 4) * 4]

    # Reorder IIQQ -> IQIQ, then convert to complex IQ.
    iq_reordered = np.copy(raw_data)
    iq_reordered[1::4] = raw_data[2::4]
    iq_reordered[2::4] = raw_data[1::4]
    iq_data = iq_reordered[0::2] + 1j * iq_reordered[1::2]

    samples_per_frame = NUM_CHIRPS_PER_FRAME * NUM_RX * NUM_ADC_SAMPLES
    num_frames = iq_data.size // samples_per_frame
    if num_frames < 1:
        raise ValueError(
            f"Insufficient IQ samples in {bin_path}. Got {iq_data.size}, "
            f"need at least {samples_per_frame} for one frame."
        )

    # Keep only complete frames, then extract frame 0.
    iq_data = iq_data[: num_frames * samples_per_frame]
    data = iq_data.reshape(num_frames, NUM_CHIRPS_PER_FRAME, NUM_RX, NUM_ADC_SAMPLES)
    data = data.reshape(num_frames, NUM_CHIRP_LOOPS, NUM_TX, NUM_RX, NUM_ADC_SAMPLES)
    data = np.transpose(data, (0, 2, 1, 3, 4))
    return data[0]


def iter_radar_bins_sorted(radar_dir: Path, max_frames: int):
    files = sorted(
        radar_dir.glob("radar*.bin"),
        key=lambda p: int(p.stem.replace("radar", "")),
    )
    return files[:max_frames]


def frame_to_range_doppler_rows(
    frame_idx: int,
    bin_path: Path,
) -> list[dict]:
    """Read one bin file and export full RD map rows using processing.py logic."""
    # TODO: uncomment this later
    with contextlib.redirect_stdout(io.StringIO()):
        frame_raw = _read_frame_with_processing_bin_logic(bin_path)
        radar = RadarSensor(frame_raw)
        range_cube, rd_cube = radar.process_tdm_mimo_cube(frame_raw)

    if MAX_RANGE is None or MIN_RANGE is None:
        rd_cube_trimmed = rd_cube
    else:
        rd_cube_trimmed, dist_axis = radar.get_range_azimuth_subset(rd_cube, MIN_RANGE, MAX_RANGE)
        radar.range_axis = dist_axis
    
    # Match processing.py visualization: TX/RX-summed power for [range, doppler].
    rd_power = np.sum(np.abs(rd_cube_trimmed) ** 2, axis=(1, 2))
    rd_db = 10.0 * np.log10(rd_power + EPSILON)

    # print("SHAPE TRIMMED AGAIN: ", rd_cube_trimmed.shape)

    n_rng, _, _, n_dop = rd_cube_trimmed.shape # 256, 16
    detections = set(radar.detect_targets_2d(rd_cube_trimmed)) # returns a list of (rng_idx, dop_idx)
    
    rows = []
    for r in range(n_rng):
        for d in range(n_dop):
            is_target = (r, d) in detections
            if is_target:
                azimuth_deg, elevation_deg = radar.phased_array_aoa(range_cube, r, d)
            else:
                azimuth_deg, elevation_deg = 0.0, 0.0

            rows.append(
                {
                    "frame": frame_idx,
                    "radar_bin_file": bin_path.name,
                    "doppler_bin": d,
                    "range_bin": r,
                    "velocity_mps": float(radar.velocity_axis[d]),
                    "range_m": float(radar.range_axis[r]),
                    "azimuth_deg": float(azimuth_deg),
                    "elevation_deg": float(elevation_deg),
                    "magnitude_linear": float(rd_power[r, d]),
                    "magnitude_db": float(rd_db[r, d]),
                    "is_target": int(is_target)
                }
            )
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export per-frame range-Doppler CSVs from a radar output folder."
    )
    parser.add_argument(
        "folder",
        type=str,
        help="Output folder name (e.g. 'output_tape') containing a 'radar/' subdirectory.",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    folder_path = Path(args.folder)
    if not folder_path.is_absolute():
        folder_path = _SCRIPT_DIR / folder_path

    radar_dir = folder_path / "radar"
    output_frames_dir = folder_path / "range_doppler_per_frame"

    if not radar_dir.is_dir():
        raise FileNotFoundError(f"Radar directory not found: {radar_dir}")

    files = iter_radar_bins_sorted(radar_dir, NUM_FRAMES_TO_EXPORT)
    if len(files) < NUM_FRAMES_TO_EXPORT:
        raise FileNotFoundError(
            f"Expected at least {NUM_FRAMES_TO_EXPORT} radar*.bin files in {radar_dir}, "
            f"found {len(files)}"
        )

    output_frames_dir.mkdir(parents=True, exist_ok=True)
    for i, bin_path in enumerate(files):
        print(f"Frame {i + 1}/{len(files)}: {bin_path.name}")
        rows = frame_to_range_doppler_rows(i, bin_path)
        out_path = output_frames_dir / f"frame_{i:04d}.csv"
        pd.DataFrame(rows).to_csv(out_path, index=False)

    print(f"Wrote {len(files)} CSV files under {output_frames_dir}")


if __name__ == "__main__":
    main()
