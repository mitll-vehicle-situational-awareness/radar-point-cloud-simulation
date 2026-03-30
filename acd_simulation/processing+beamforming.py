import os
import time
import numpy as np
import matplotlib.pyplot as plt
import stream_data_cs_team

EPSILON = 1e-12


class RadarSensor:
    def __init__(self, sample):
        # -----------------------------
        # Hardware / waveform constants
        # -----------------------------
        self.f0 = 60e9
        self.fs = 12.5e6
        self.c = 299792458.0
        self.wavelength = self.c / self.f0
        self.antenna_spacing = self.wavelength / 2.0

        # sample shape expected: [tx, chirp_loop, rx, adc]
        self.num_tx = sample.shape[0]
        self.num_chirp_loops = sample.shape[1]
        self.num_rx = sample.shape[2]
        self.num_adc_samples = sample.shape[3]

        self.num_antennas = self.num_tx * self.num_rx
        self.num_chirps = self.num_tx * self.num_chirp_loops

        # ---------------------------------------------
        # Keep these aligned with your acquisition cfg
        # ---------------------------------------------
        self.bw = 3.41e9
        self.range_resolution = 0.044
        self.velocity_resolution = 1.26

        self.range_axis = np.arange(self.num_adc_samples) * self.range_resolution
        doppler_bins = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)
        self.velocity_axis = doppler_bins * self.velocity_resolution

        # Angle FFT over the RX array after TX beamforming.
        # Since TXs are coherently collapsed into one steered beam,
        # angle estimation here is from the physical RX array only.
        self.num_angle_bins = 64
        self.rx_spatial_freq_axis = np.fft.fftshift(
            np.fft.fftfreq(self.num_angle_bins, d=1.0)
        )
        sin_theta = np.clip(
            self.rx_spatial_freq_axis * (self.wavelength / self.antenna_spacing),
            -1.0,
            1.0
        )
        self.rx_angle_axis = np.rad2deg(np.arcsin(sin_theta))

        print("sample shape:", sample.shape)
        print("num_tx:", self.num_tx)
        print("num_chirp_loops:", self.num_chirp_loops)
        print("num_rx:", self.num_rx)
        print("num_adc_samples:", self.num_adc_samples)
        print("velocity axis:", self.velocity_axis[0], self.velocity_axis[-1])

    # ---------------------------------------------------------
    # 1) Standard TDM-MIMO range/Doppler cube
    #    output: [range_bin, tx, rx, doppler_bin]
    # ---------------------------------------------------------
    def process_tdm_mimo_cube(self, frame_raw):
        if frame_raw.shape != (
            self.num_tx,
            self.num_chirp_loops,
            self.num_rx,
            self.num_adc_samples,
        ):
            raise ValueError(
                f"frame_raw shape {frame_raw.shape} does not match "
                f"({self.num_tx}, {self.num_chirp_loops}, {self.num_rx}, {self.num_adc_samples})"
            )

        # [tx, chirp, rx, adc] -> [adc, tx, rx, chirp]
        data = np.transpose(frame_raw, (3, 0, 2, 1))

        range_win = np.hamming(self.num_adc_samples).reshape(-1, 1, 1, 1)
        doppler_win = np.hamming(self.num_chirp_loops).reshape(1, 1, 1, -1)

        range_fft = np.fft.fft(data * range_win, axis=0)
        rd_cube_txrx = np.fft.fft(range_fft * doppler_win, axis=3)
        rd_cube_txrx = np.fft.fftshift(rd_cube_txrx, axes=3)

        return rd_cube_txrx

    # ---------------------------------------------------------
    # 2) TX steering weights for a desired azimuth angle
    #    This mirrors the beam-steering math you were using:
    #    phase_m = 2*pi*m*sin(theta)
    # ---------------------------------------------------------
    def tx_steering_weights(self, theta_deg):
        m_ind = np.arange(self.num_tx)
        phase_rad = 2.0 * np.pi * (m_ind * np.sin(np.deg2rad(theta_deg)))
        # Conjugate steering for coherent combining
        w_tx = np.exp(-1j * phase_rad)

        # Optional normalization so power scale is less sensitive to num_tx
        w_tx = w_tx / np.sqrt(self.num_tx)
        return w_tx  # shape: [tx]

    # ---------------------------------------------------------
    # 3) Offline TX beamforming from a TDM cube
    #
    #    For each Doppler bin:
    #      - compensate TDM inter-TX motion phase
    #      - apply TX steering weights
    #      - sum TXs coherently
    #
    #    output: [range_bin, rx, doppler_bin]
    # ---------------------------------------------------------
    def beamform_tx_direction(self, rd_cube_txrx, theta_deg, phase_sign=-1):
        if rd_cube_txrx.shape != (
            self.num_adc_samples,
            self.num_tx,
            self.num_rx,
            self.num_chirp_loops,
        ):
            raise ValueError(
                f"rd_cube_txrx shape {rd_cube_txrx.shape} does not match "
                f"({self.num_adc_samples}, {self.num_tx}, {self.num_rx}, {self.num_chirp_loops})"
            )

        w_tx = self.tx_steering_weights(theta_deg)  # [tx]

        # Signed Doppler bin indices after fftshift
        d_signed = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)

        # Doppler phase advance per chirp-loop index
        omega = 2.0 * np.pi * d_signed / self.num_chirp_loops  # [doppler]

        # TDM delay compensation:
        # TX0 observed at time offset 0
        # TX1 observed one chirp later
        # TX2 observed two chirps later
        tx_idx = np.arange(self.num_tx)[:, None]  # [tx, 1]
        phase_corr = np.exp(phase_sign * 1j * (tx_idx * omega[None, :]))  # [tx, doppler]

        # Combine compensation and desired TX steering
        tx_combiner = w_tx[:, None] * phase_corr  # [tx, doppler]

        # rd_cube_txrx: [range, tx, rx, doppler]
        # einsum over tx -> beamformed [range, rx, doppler]
        beam_cube_rx = np.einsum("rtxd,td->rxd", rd_cube_txrx, tx_combiner)

        return beam_cube_rx

    # ---------------------------------------------------------
    # 4) Integrated RD power after TX beamforming
    # ---------------------------------------------------------
    def integrate_beam_rd_power(self, beam_cube_rx):
        return np.sum(np.abs(beam_cube_rx) ** 2, axis=1)

    # ---------------------------------------------------------
    # 5) Angle spectrum from RX-only array after TX beamforming
    #    output for one RD cell
    # ---------------------------------------------------------
    def get_rx_angle_spectrum(self, beam_cube_rx, r_idx, d_idx):
        rx_vec = beam_cube_rx[r_idx, :, d_idx]
        angle_fft = np.fft.fftshift(np.fft.fft(rx_vec, n=self.num_angle_bins))
        return np.abs(angle_fft) ** 2

    # ---------------------------------------------------------
    # 6) 2-stage CFAR on a 2D RD power map
    # ---------------------------------------------------------
    def detect_targets_2d(
        self,
        rd_power,
        range_train=8,
        range_guard=4,
        range_threshold_db=12.0,
        doppler_train=4,
        doppler_guard=2,
        doppler_threshold_db=10.0,
    ):
        rd_power = rd_power.copy()
        rd_power[:4, :] = 0  # suppress near-zero-range blob

        num_range_bins, num_doppler_bins = rd_power.shape

        range_pass = np.zeros_like(rd_power, dtype=bool)
        doppler_pass = np.zeros_like(rd_power, dtype=bool)

        range_scale = 10 ** (range_threshold_db / 10.0)
        doppler_scale = 10 ** (doppler_threshold_db / 10.0)

        # Pass 1: CASO along range
        for d in range(num_doppler_bins):
            for r in range(range_train + range_guard, num_range_bins - (range_train + range_guard)):
                cut = rd_power[r, d]

                left_noise = np.mean(
                    rd_power[r - range_guard - range_train : r - range_guard, d]
                )
                right_noise = np.mean(
                    rd_power[r + range_guard + 1 : r + range_guard + 1 + range_train, d]
                )

                noise_est = min(left_noise, right_noise)
                threshold = noise_est * range_scale

                if cut > threshold and cut > 0:
                    range_pass[r, d] = True

        # Pass 2: CA along Doppler
        for r in range(num_range_bins):
            for d in range(doppler_train + doppler_guard, num_doppler_bins - (doppler_train + doppler_guard)):
                if not range_pass[r, d]:
                    continue

                cut = rd_power[r, d]

                left_noise = rd_power[r, d - doppler_guard - doppler_train : d - doppler_guard]
                right_noise = rd_power[r, d + doppler_guard + 1 : d + doppler_guard + 1 + doppler_train]

                noise_est = np.mean(np.concatenate((left_noise, right_noise)))
                threshold = noise_est * doppler_scale

                if cut > threshold and cut > 0:
                    doppler_pass[r, d] = True

        detections = []
        candidates = np.argwhere(doppler_pass)

        for r, d in candidates:
            region = rd_power[
                max(0, r - 1) : min(num_range_bins, r + 2),
                max(0, d - 1) : min(num_doppler_bins, d + 2),
            ]
            if rd_power[r, d] == np.max(region):
                detections.append((r, d))

        return detections


def quadratic_interp_1d(y_l, y_0, y_r, bin_center, bin_spacing):
    denom = 2.0 * (y_l + y_r - 2.0 * y_0 + EPSILON)
    delta = (y_l - y_r) / denom
    delta = np.clip(delta, -0.5, 0.5)
    return bin_center + delta * bin_spacing


def run_radar_simulation():
    ss = stream_data_cs_team.dataStream()
    radar = RadarSensor(ss.data[0])

    print("ss.num_frames:", ss.num_frames)

    beam_theta_deg = -33.75  # choose your TX steering direction here

    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file_path = os.path.join(log_dir, "radar_stream_log_tx_beamforming.txt")

    plt.ion()
    fig = plt.figure(figsize=(12, 5))
    gs = fig.add_gridspec(1, 2)
    ax_rd = fig.add_subplot(gs[0, 0])
    ax_bev = fig.add_subplot(gs[0, 1])

    with open(log_file_path, "w", encoding="utf-8") as f:
        for frame_idx in range(ss.num_frames):
            start_time = time.time()
            raw_frame = ss.data[frame_idx]

            # Step 1: standard TDM cube
            rd_cube_txrx = radar.process_tdm_mimo_cube(raw_frame)

            # Step 2: apply offline TX beamforming for one steered direction
            beam_cube_rx = radar.beamform_tx_direction(
                rd_cube_txrx,
                theta_deg=beam_theta_deg,
                phase_sign=-1
            )

            # Step 3: detect on beamformed RD map
            rd_power = radar.integrate_beam_rd_power(beam_cube_rx)
            detections = radar.detect_targets_2d(rd_power)

            rd_amp = np.sqrt(rd_power)
            rd_db = 20.0 * np.log10(rd_amp + EPSILON)

            ax_rd.cla()
            ax_rd.imshow(
                rd_db,
                aspect="auto",
                origin="lower",
                extent=[
                    radar.velocity_axis[0],
                    radar.velocity_axis[-1],
                    radar.range_axis[0],
                    radar.range_axis[-1],
                ],
            )
            ax_rd.set_title(f"Beamformed RD | Frame {frame_idx} | TX steer {beam_theta_deg:.2f}°")
            ax_rd.set_xlabel("Velocity (m/s)")
            ax_rd.set_ylabel("Slant Range (m)")

            ax_bev.cla()
            ax_bev.set_xlim(-10, 10)
            ax_bev.set_ylim(-10, 10)
            ax_bev.grid(True)
            ax_bev.plot(0, 0, "r^")
            ax_bev.set_title(f"BEV | detections: {len(detections)}")
            ax_bev.set_xlabel("Lateral (m)")
            ax_bev.set_ylabel("Forward Range (m)")

            processing_time_ms = (time.time() - start_time) * 1000.0
            f.write(
                f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | "
                f"Latency: {processing_time_ms:.2f} ms | Beam: {beam_theta_deg:.2f} deg\n"
            )

            for r_idx, d_idx in detections:
                # Range interpolation
                if 0 < r_idx < radar.num_adc_samples - 1:
                    y_l = np.sqrt(rd_power[r_idx - 1, d_idx])
                    y_0 = np.sqrt(rd_power[r_idx, d_idx])
                    y_r = np.sqrt(rd_power[r_idx + 1, d_idx])
                    interp_r = quadratic_interp_1d(
                        y_l,
                        y_0,
                        y_r,
                        radar.range_axis[r_idx],
                        radar.range_axis[1] - radar.range_axis[0],
                    )
                else:
                    interp_r = radar.range_axis[r_idx]

                # Doppler interpolation
                d_prev = (d_idx - 1) % radar.num_chirp_loops
                d_next = (d_idx + 1) % radar.num_chirp_loops
                v_l = np.sqrt(rd_power[r_idx, d_prev])
                v_0 = np.sqrt(rd_power[r_idx, d_idx])
                v_r = np.sqrt(rd_power[r_idx, d_next])

                interp_v = quadratic_interp_1d(
                    v_l,
                    v_0,
                    v_r,
                    radar.velocity_axis[d_idx],
                    radar.velocity_axis[1] - radar.velocity_axis[0],
                )

                ax_rd.plot(interp_v, interp_r, "ro", markersize=4)

                # RX-only AoA after TX beamforming
                angle_spectrum = radar.get_rx_angle_spectrum(beam_cube_rx, r_idx, d_idx)
                ang_idx = np.argmax(angle_spectrum)

                if 0 < ang_idx < radar.num_angle_bins - 1:
                    a_l = np.log(angle_spectrum[ang_idx - 1] + EPSILON)
                    a_0 = np.log(angle_spectrum[ang_idx] + EPSILON)
                    a_r = np.log(angle_spectrum[ang_idx + 1] + EPSILON)

                    den = a_l - 2.0 * a_0 + a_r
                    frac = 0.0 if abs(den) < 1e-12 else 0.5 * (a_l - a_r) / den
                    frac = np.clip(frac, -0.5, 0.5)

                    u_interp = radar.rx_spatial_freq_axis[ang_idx] + frac * (
                        radar.rx_spatial_freq_axis[1] - radar.rx_spatial_freq_axis[0]
                    )
                    sin_theta_interp = np.clip(
                        u_interp * (radar.wavelength / radar.antenna_spacing),
                        -1.0,
                        1.0,
                    )
                    aoa_deg = np.rad2deg(np.arcsin(sin_theta_interp))
                else:
                    aoa_deg = radar.rx_angle_axis[ang_idx]

                x = interp_r * np.sin(np.deg2rad(aoa_deg))
                y = interp_r * np.cos(np.deg2rad(aoa_deg))

                ax_bev.plot(x, y, "bo")
                f.write(
                    f"  DETECT | R:{interp_r:6.3f} m | V:{interp_v:6.3f} m/s | "
                    f"AoA:{aoa_deg:6.2f} deg | X:{x:6.3f} | Y:{y:6.3f}\n"
                )

            plt.tight_layout()
            plt.draw()
            plt.pause(0.05)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    run_radar_simulation()