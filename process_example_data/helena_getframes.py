"""
Load per-frame radar bins from output_apple_moving/radar, compute range–Doppler
maps (same pipeline as the original script), and write one CSV per frame.

Each radarN.bin is one frame (48 chirps, 4 RX, 256 ADC — matches OUTPUT_BIN_CONFIG).
Processes the first 98 frames: radar0.bin … radar97.bin.

Output: output_apple_moving/range_doppler_per_frame/frame_XXXX.csv
"""

from __future__ import annotations

import contextlib
import io
from pathlib import Path

import numpy as np
import pandas as pd

from processing import EPSILON, RadarSensor

# Paths (script lives in acd_simulation/)
_SCRIPT_DIR = Path(__file__).resolve().parent
RADAR_DIR = _SCRIPT_DIR / "output_apple_moving" / "radar"
OUTPUT_FRAMES_DIR = _SCRIPT_DIR / "output_apple_moving" / "range_doppler_per_frame"

NUM_FRAMES_TO_EXPORT = 98

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
    with contextlib.redirect_stdout(io.StringIO()):
        frame_raw = _read_frame_with_processing_bin_logic(bin_path)
        radar = RadarSensor(frame_raw)
        range_cube, rd_cube = radar.process_tdm_mimo_cube(frame_raw)

    # Match processing.py visualization: integrate TX/RX for [range, doppler].
    rd_amp = np.sqrt(np.sum(np.abs(rd_cube) ** 2, axis=(1, 2)))
    rd_db = 20.0 * np.log10(rd_amp + EPSILON)

    n_rng, n_dop = rd_db.shape
    # compute_aoa_fft depends on range bin index (not Doppler bin), so precompute once/range.
    aoa_per_range = [radar.compute_aoa_fft(range_cube, r) for r in range(n_rng)]
    rows = []
    for d in range(n_dop):
        for r in range(n_rng):
            azimuth_deg, elevation_deg = aoa_per_range[r]
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
                    "magnitude_linear": float(rd_amp[r, d]),
                    "magnitude_db": float(rd_db[r, d]),
                }
            )
    return rows


def main():
    if not RADAR_DIR.is_dir():
        raise FileNotFoundError(f"Radar directory not found: {RADAR_DIR}")

    files = iter_radar_bins_sorted(RADAR_DIR, NUM_FRAMES_TO_EXPORT)
    if len(files) < NUM_FRAMES_TO_EXPORT:
        raise FileNotFoundError(
            f"Expected at least {NUM_FRAMES_TO_EXPORT} radar*.bin files in {RADAR_DIR}, "
            f"found {len(files)}"
        )

    OUTPUT_FRAMES_DIR.mkdir(parents=True, exist_ok=True)
    for i, bin_path in enumerate(files):
        print(f"Frame {i + 1}/{len(files)}: {bin_path.name}")
        rows = frame_to_range_doppler_rows(i, bin_path)
        out_path = OUTPUT_FRAMES_DIR / f"frame_{i:04d}.csv"
        pd.DataFrame(rows).to_csv(out_path, index=False)

    print(f"Wrote {len(files)} CSV files under {OUTPUT_FRAMES_DIR}")


if __name__ == "__main__":
    main()
