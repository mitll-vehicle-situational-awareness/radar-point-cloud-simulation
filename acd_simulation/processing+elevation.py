import os
import sys
import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import stream_data_cs_team

EPSILON = 1e-12

class RadarSensor:
    def __init__(self, sample):
        self.f0 = 60e9
        self.fs = 12.5e6
        self.c = 299792458.0
        self.wavelength = self.c / self.f0

        # frame_raw shape: [tx, chirp_loop, rx, adc_sample]
        self.num_tx = sample.shape[0]
        self.num_chirp_loops = sample.shape[1]
        self.num_rx = sample.shape[2]
        self.num_adc_samples = sample.shape[3]

        self.num_antennas = self.num_tx * self.num_rx
        self.num_chirps = self.num_chirp_loops * self.num_tx

        # Keep your current axis assumptions for now
        self.bw = 3.41e9
        self.slope = self.bw / (self.num_adc_samples / self.fs)
        self.antenna_spacing = self.wavelength / 2.0

        self.range_resolution = 0.044
        self.range_axis = np.arange(self.num_adc_samples) * self.range_resolution

        self.velocity_resolution = 1.26
        doppler_bins = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)
        self.velocity_axis = doppler_bins * self.velocity_resolution

        # Commented out noisy prints
        # print("sample.shape:", sample.shape)
        # print("num_chirp_loops:", self.num_chirp_loops)
        # print("num_chirps:", self.num_chirps)
        # print("velocity axis:", self.velocity_axis[0], self.velocity_axis[-1])

        # Cache geometry once
        self.geometry = self.get_iwr6843levm_virtual_geometry()

        # Angle search grids
        self.az_grid_deg = np.arange(-70.0, 70.1, 1.0)
        self.el_grid_deg = np.arange(-25.0, 25.1, 1.0)

        # Precompute steering matrix once
        self.steering_matrix = self._precompute_steering_matrix()

    def process_tdm_mimo_cube(self, frame_raw):
        """
        frame_raw: [tx, chirp_loop, rx, adc]
        returns rd_cube_txrx: [range, tx, rx, doppler]
        """
        if frame_raw.shape != (self.num_tx, self.num_chirp_loops, self.num_rx, self.num_adc_samples):
            raise ValueError(
                f"frame_raw shape {frame_raw.shape} does not match "
                f"({self.num_tx}, {self.num_chirp_loops}, {self.num_rx}, {self.num_adc_samples})"
            )

        data = np.transpose(frame_raw, (3, 0, 2, 1))  # [adc, tx, rx, chirp_loop]

        range_win = np.hamming(self.num_adc_samples).reshape(-1, 1, 1, 1)
        doppler_win = np.hamming(self.num_chirp_loops).reshape(1, 1, 1, -1)

        range_fft = np.fft.fft(data * range_win, axis=0)
        rd_cube_txrx = np.fft.fft(range_fft * doppler_win, axis=3)
        rd_cube_txrx = np.fft.fftshift(rd_cube_txrx, axes=3)

        return rd_cube_txrx

    def detect_targets_2d(
        self,
        radar_cube,
        range_train=8,
        range_guard=4,
        range_threshold_db=12.0,
        doppler_train=4,
        doppler_guard=2,
        doppler_threshold_db=10.0,
    ):
        """
        radar_cube: [range, tx, rx, doppler]
        returns detections: list of (r_idx, d_idx)
        """
        range_doppler_power = np.sum(np.abs(radar_cube) ** 2, axis=(1, 2))
        range_doppler_power[:4, :] = 0

        num_range_bins, num_doppler_bins = range_doppler_power.shape

        range_pass = np.zeros_like(range_doppler_power, dtype=bool)
        doppler_pass = np.zeros_like(range_doppler_power, dtype=bool)

        range_scale = 10 ** (range_threshold_db / 10.0)
        for d in range(num_doppler_bins):
            for r in range(range_train + range_guard, num_range_bins - (range_train + range_guard)):
                cut_power = range_doppler_power[r, d]

                left_start = r - range_guard - range_train
                left_end = r - range_guard
                right_start = r + range_guard + 1
                right_end = r + range_guard + 1 + range_train

                left_noise = np.mean(range_doppler_power[left_start:left_end, d])
                right_noise = np.mean(range_doppler_power[right_start:right_end, d])

                noise_est = min(left_noise, right_noise)
                threshold = noise_est * range_scale

                if cut_power > threshold and cut_power > 0:
                    range_pass[r, d] = True

        doppler_scale = 10 ** (doppler_threshold_db / 10.0)
        for r in range(num_range_bins):
            for d in range(doppler_train + doppler_guard, num_doppler_bins - (doppler_train + doppler_guard)):
                if not range_pass[r, d]:
                    continue

                cut_power = range_doppler_power[r, d]

                left_start = d - doppler_guard - doppler_train
                left_end = d - doppler_guard
                right_start = d + doppler_guard + 1
                right_end = d + doppler_guard + 1 + doppler_train

                left_noise = range_doppler_power[r, left_start:left_end]
                right_noise = range_doppler_power[r, right_start:right_end]

                noise_est = np.mean(np.concatenate((left_noise, right_noise)))
                threshold = noise_est * doppler_scale

                if cut_power > threshold and cut_power > 0:
                    doppler_pass[r, d] = True

        detections = []
        candidates = np.argwhere(doppler_pass)

        for r, d in candidates:
            region = range_doppler_power[
                max(0, r - 1):min(num_range_bins, r + 2),
                max(0, d - 1):min(num_doppler_bins, d + 2)
            ]
            if range_doppler_power[r, d] == np.max(region):
                detections.append((r, d))

        return detections

    def get_iwr6843levm_virtual_geometry(self):
        """
        Coordinates in units of lambda/2.
        Order must match:
        [TX0RX0..TX0RX3, TX1RX0..TX1RX3, TX2RX0..TX2RX3]
        """
        rx_pos = [
            (0, 0),
            (1, 0),
            (2, 0),
            (3, 0),
        ]

        tx_pos = [
            (0, 0),  # first TX block
            (2, 1),  # second TX block
            (4, 0),  # third TX block
        ]

        geometry = []
        for tx_idx, (tx_x, tx_z) in enumerate(tx_pos):
            for rx_idx, (rx_x, rx_z) in enumerate(rx_pos):
                geometry.append({
                    "tx": tx_idx,
                    "rx": rx_idx,
                    "x": tx_x + rx_x,
                    "z": tx_z + rx_z,
                })
        return geometry

    def _precompute_steering_matrix(self):
        """
        Precompute steering vectors for all az/el grid points.
        Output shape: [num_grid_points, num_antennas]
        """
        x_coords = np.array([g["x"] for g in self.geometry], dtype=np.float64)
        z_coords = np.array([g["z"] for g in self.geometry], dtype=np.float64)

        steering_rows = []

        for el_deg in self.el_grid_deg:
            el = np.deg2rad(el_deg)
            sin_el = np.sin(el)
            cos_el = np.cos(el)

            for az_deg in self.az_grid_deg:
                az = np.deg2rad(az_deg)
                phase = np.pi * (
                    x_coords * np.sin(az) * cos_el +
                    z_coords * sin_el
                )
                steering_rows.append(np.exp(1j * phase))

        return np.asarray(steering_rows, dtype=np.complex128)

    def build_virtual_ant_samples_3d(self, rd_cube_txrx, r_idx, d_idx, phase_sign=-1):
        """
        rd_cube_txrx: [range, tx, rx, doppler]
        returns:
            samples: [12]
        """
        txrx_slice = rd_cube_txrx[r_idx, :, :, d_idx].copy()

        doppler_bin_signed = d_idx - (self.num_chirp_loops // 2)
        omega = 2.0 * np.pi * doppler_bin_signed / self.num_chirp_loops

        for tx_idx in range(self.num_tx):
            phase_corr = np.exp(phase_sign * 1j * omega * tx_idx)
            txrx_slice[tx_idx, :] *= phase_corr

        samples = txrx_slice.reshape(self.num_tx * self.num_rx)
        return samples

    def beamform_az_el(self, samples):
        """
        Fast beamforming using precomputed steering matrix.
        Returns power_map of shape [len(el_grid), len(az_grid)]
        """
        responses = self.steering_matrix.conj() @ samples
        power = np.abs(responses) ** 2
        power_map = power.reshape(len(self.el_grid_deg), len(self.az_grid_deg))
        return power_map

    def estimate_az_el(self, rd_cube_txrx, r_idx, d_idx, phase_sign=-1):
        samples = self.build_virtual_ant_samples_3d(
            rd_cube_txrx, r_idx, d_idx, phase_sign=phase_sign
        )

        power_map = self.beamform_az_el(samples)
        peak_idx = np.argmax(power_map)
        ei, ai = np.unravel_index(peak_idx, power_map.shape)

        azimuth_deg = self.az_grid_deg[ai]
        elevation_deg = self.el_grid_deg[ei]

        return azimuth_deg, elevation_deg, power_map

    def detection_to_point_3d(self, rd_cube_txrx, r_idx, d_idx, phase_sign=-1):
        # Range interpolation
        if 0 < r_idx < self.num_adc_samples - 1:
            r_prev, r_next = r_idx - 1, r_idx + 1
            y0 = np.sqrt(np.sum(np.abs(rd_cube_txrx[r_idx, :, :, d_idx]) ** 2))
            y_l = np.sqrt(np.sum(np.abs(rd_cube_txrx[r_prev, :, :, d_idx]) ** 2))
            y_r = np.sqrt(np.sum(np.abs(rd_cube_txrx[r_next, :, :, d_idx]) ** 2))

            range_m = (
                self.range_axis[r_idx]
                + (self.range_axis[1] - self.range_axis[0])
                * ((y_l - y_r) / (2 * (y_l + y_r - 2 * y0 + EPSILON)))
            )
        else:
            range_m = self.range_axis[r_idx]

        # Doppler interpolation
        d_prev = (d_idx - 1) % self.num_chirp_loops
        d_next = (d_idx + 1) % self.num_chirp_loops

        v0 = np.sqrt(np.sum(np.abs(rd_cube_txrx[r_idx, :, :, d_idx]) ** 2))
        v_l = np.sqrt(np.sum(np.abs(rd_cube_txrx[r_idx, :, :, d_prev]) ** 2))
        v_r = np.sqrt(np.sum(np.abs(rd_cube_txrx[r_idx, :, :, d_next]) ** 2))

        velocity_mps = (
            self.velocity_axis[d_idx]
            + (self.velocity_axis[1] - self.velocity_axis[0])
            * ((v_l - v_r) / (2 * (v_l + v_r - 2 * v0 + EPSILON)))
        )

        azimuth_deg, elevation_deg, _ = self.estimate_az_el(
            rd_cube_txrx, r_idx, d_idx, phase_sign=phase_sign
        )

        az = np.deg2rad(azimuth_deg)
        el = np.deg2rad(elevation_deg)

        x_m = range_m * np.cos(el) * np.sin(az)
        y_m = range_m * np.cos(el) * np.cos(az)
        z_m = range_m * np.sin(el)

        return {
            "range_m": float(range_m),
            "velocity_mps": float(velocity_mps),
            "azimuth_deg": float(azimuth_deg),
            "elevation_deg": float(elevation_deg),
            "x_m": float(x_m),
            "y_m": float(y_m),
            "z_m": float(z_m),
            "r_idx": int(r_idx),
            "d_idx": int(d_idx),
        }

    def export_frame_points_to_csv(self, filename, points):
        parent = os.path.dirname(filename)
        if parent:
            os.makedirs(parent, exist_ok=True)

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "range_m",
                "velocity_mps",
                "azimuth_deg",
                "elevation_deg",
                "x_m",
                "y_m",
                "z_m",
                "r_idx",
                "d_idx",
            ])

            for p in points:
                writer.writerow([
                    p["range_m"],
                    p["velocity_mps"],
                    p["azimuth_deg"],
                    p["elevation_deg"],
                    p["x_m"],
                    p["y_m"],
                    p["z_m"],
                    p["r_idx"],
                    p["d_idx"],
                ])


def get_topk_rd_points(radar, rd_cube, top_k=40, min_range_bin=4, phase_sign=-1, point_cache=None):
    """
    Build a faint context cloud from strongest RD bins,
    not just CFAR detections.
    """
    rd_power = np.sum(np.abs(rd_cube) ** 2, axis=(1, 2)).copy()
    rd_power[:min_range_bin, :] = 0

    flat = rd_power.ravel()
    if top_k >= flat.size:
        top_idx = np.argsort(flat)[::-1]
    else:
        top_idx = np.argpartition(flat, -top_k)[-top_k:]
        top_idx = top_idx[np.argsort(flat[top_idx])[::-1]]

    rd_indices = np.array(np.unravel_index(top_idx, rd_power.shape)).T

    points = []
    seen = set()

    for r_idx, d_idx in rd_indices:
        key = (int(r_idx), int(d_idx))
        if key in seen:
            continue
        seen.add(key)

        try:
            if point_cache is not None and key in point_cache:
                point = dict(point_cache[key])
            else:
                point = radar.detection_to_point_3d(rd_cube, int(r_idx), int(d_idx), phase_sign=phase_sign)
                if point_cache is not None:
                    point_cache[key] = dict(point)

            point["power"] = float(rd_power[r_idx, d_idx])
            points.append(point)
        except Exception:
            continue

    return points


def run_radar_simulation():
    ss = stream_data_cs_team.dataStream()
    radar = RadarSensor(ss.data[0])

    # print("ss.num_frames:", ss.num_frames)

    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file_path = os.path.join(log_dir, "radar_stream_log.txt")

    csv_dir = "csv_frames"
    os.makedirs(csv_dir, exist_ok=True)

    plt.ion()
    fig = plt.figure(figsize=(13, 5))
    gs = fig.add_gridspec(1, 2)

    ax_rd = fig.add_subplot(gs[0, 0])
    ax_3d = fig.add_subplot(gs[0, 1], projection='3d')

    progress_bar = tqdm(range(ss.num_frames), desc="Processing Frames", file=sys.stdout)

    with open(log_file_path, "w", encoding="utf-8") as f:
        for frame_idx in progress_bar:
            t0 = time.time()

            frame_raw = ss.data[frame_idx]  # [tx, chirp_loop, rx, adc]
            rd_cube = radar.process_tdm_mimo_cube(frame_raw)  # [range, tx, rx, doppler]
            t1 = time.time()

            detections = radar.detect_targets_2d(rd_cube)
            t2 = time.time()

            point_cache = {}

            # Dense-ish context cloud
            topk_points = get_topk_rd_points(
                radar=radar,
                rd_cube=rd_cube,
                top_k=35,
                min_range_bin=4,
                phase_sign=-1,
                point_cache=point_cache
            )
            t3 = time.time()

            # CFAR detections
            detection_points = []
            for r_idx, d_idx in detections:
                key = (int(r_idx), int(d_idx))
                try:
                    if key in point_cache:
                        point = dict(point_cache[key])
                    else:
                        point = radar.detection_to_point_3d(rd_cube, r_idx, d_idx, phase_sign=-1)
                        point_cache[key] = dict(point)

                    detection_points.append(point)
                except Exception:
                    # print(f"Skipping detection ({r_idx}, {d_idx}) due to error: {e}")
                    continue
            t4 = time.time()

            csv_path = os.path.join(csv_dir, f"frame_{frame_idx:03d}_3d.csv")
            radar.export_frame_points_to_csv(csv_path, detection_points)
            t5 = time.time()

            rd_amp = np.sqrt(np.sum(np.abs(rd_cube) ** 2, axis=(1, 2)))
            rd_db = 20.0 * np.log10(rd_amp + EPSILON)

            ax_rd.cla()
            ax_rd.imshow(
                rd_db,
                aspect='auto',
                origin='lower',
                extent=[
                    radar.velocity_axis[0],
                    radar.velocity_axis[-1],
                    radar.range_axis[0],
                    radar.range_axis[-1]
                ]
            )
            ax_rd.set_title(f"Range-Doppler Frame {frame_idx}")
            ax_rd.set_xlabel("Velocity (m/s)")
            ax_rd.set_ylabel("Slant Range (m)")

            for p in detection_points:
                ax_rd.plot(p["velocity_mps"], p["range_m"], 'ro', markersize=4)

            ax_3d.cla()
            ax_3d.set_title(f"3D Radar Cloud - Frame {frame_idx}")
            ax_3d.set_xlabel("X lateral (m)")
            ax_3d.set_ylabel("Y forward (m)")
            ax_3d.set_zlabel("Z height (m)")
            ax_3d.set_xlim(-5, 5)
            ax_3d.set_ylim(0, 10)
            ax_3d.set_zlim(-3, 3)

            if topk_points:
                xs = [p["x_m"] for p in topk_points]
                ys = [p["y_m"] for p in topk_points]
                zs = [p["z_m"] for p in topk_points]
                ax_3d.scatter(xs, ys, zs, s=10, alpha=0.25)

            if detection_points:
                xs = [p["x_m"] for p in detection_points]
                ys = [p["y_m"] for p in detection_points]
                zs = [p["z_m"] for p in detection_points]
                ax_3d.scatter(xs, ys, zs, s=40, alpha=0.9)

            ax_3d.scatter([0], [0], [0], marker='^', s=80)

            plt.tight_layout()
            plt.draw()
            plt.pause(0.001)
            t6 = time.time()

            processing_time = (t6 - t0) * 1000.0
            progress_bar.set_postfix({
                "lat_ms": f"{processing_time:.1f}",
                "fft_ms": f"{(t1 - t0) * 1000:.0f}",
                "cfar_ms": f"{(t2 - t1) * 1000:.0f}",
                "topk_ms": f"{(t3 - t2) * 1000:.0f}",
                "det_ms": f"{(t4 - t3) * 1000:.0f}",
                "plot_ms": f"{(t6 - t5) * 1000:.0f}",
                "det": len(detection_points)
            })

            f.write(f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | Latency: {processing_time:.2f} ms\n")

            for p in detection_points:
                f.write(
                    f"R:{p['range_m']:.3f} | V:{p['velocity_mps']:.3f} | "
                    f"AZ:{p['azimuth_deg']:.2f} | EL:{p['elevation_deg']:.2f} | "
                    f"X:{p['x_m']:.3f} | Y:{p['y_m']:.3f} | Z:{p['z_m']:.3f}\n"
                )

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    run_radar_simulation()