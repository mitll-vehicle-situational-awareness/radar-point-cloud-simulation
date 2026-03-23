import os
import time
import numpy as np
import matplotlib.pyplot as plt
import stream_data_cs_team
from scipy.ndimage import convolve

# global
EPSILON = 1e-12


class RadarSensor:
    def __init__(self, sample):
        # Hardware constants
        self.f0 = 60e9
        self.fs = 12.5e6
        self.c = 299792458.0
        self.wavelength = self.c / self.f0

        self.num_tx = sample.shape[0]
        self.num_chirp_loops = sample.shape[1]
        self.num_rx = sample.shape[2]
        self.num_adc_samples = sample.shape[3]

        self.num_antennas = self.num_tx * self.num_rx
        self.num_chirps = self.num_chirp_loops * self.num_tx

        # self.pri = 62e-6
        self.bw = 3.41e9
        self.slope = self.bw / (self.num_adc_samples / self.fs)
        self.antenna_spacing = self.wavelength / 2.0

        # self.range_axis = (self.c * (np.arange(self.num_adc_samples) / self.num_adc_samples) * self.fs) / (2.0 * self.slope)

        self.range_resolution = 0.044
        self.range_axis = np.arange(self.num_adc_samples) * self.range_resolution

        self.velocity_resolution = 1.26
        doppler_bins = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)
        self.velocity_axis = doppler_bins * self.velocity_resolution

        self.num_angle_bins = self.num_rx * self.num_tx
        self.spatial_freq_axis = np.fft.fftshift(
            np.fft.fftfreq(self.num_angle_bins, d=1.0)
        )
        sin_theta = np.clip(
            self.spatial_freq_axis * (self.wavelength / self.antenna_spacing),
            -1.0,
            1.0
        )
        self.angle_axis = np.rad2deg(np.arcsin(sin_theta))

        print("num_chirp_loops:", self.num_chirp_loops)
        print("num_chirps:", self.num_chirps)
        print("velocity axis:", self.velocity_axis[0], self.velocity_axis[-1])

    def process_radar_cube(self, raw_data):
        """
        raw_data shape: [range_sample, virtual_ant, chirp_loop]
        returns:
            rd_fft   : [range_bin, virtual_ant, doppler_bin]
            range_fft: [range_bin, virtual_ant, chirp_loop]
        """

        if raw_data.shape != (self.num_adc_samples, self.num_antennas, self.num_chirp_loops):
            raise ValueError(
                f"raw_data shape {raw_data.shape} does not match "
                f"({self.num_adc_samples}, {self.num_antennas}, {self.num_chirp_loops})"
            )

        # Note: uncomment if we want to remove static refectors
        # if remove_static:
        #     raw_data = raw_data - np.mean(raw_data, axis=2, keepdims=True)
        # raw_data -= np.mean(raw_data, axis=0, keepdims=True)

        range_win = np.hamming(self.num_adc_samples).reshape(-1, 1, 1)
        doppler_win = np.hamming(self.num_chirp_loops).reshape(1, 1, -1)

        # Range FFT
        range_fft = np.fft.fft(raw_data * range_win, axis=0)

        # Doppler FFT
        rd_fft = np.fft.fft(range_fft * doppler_win, axis=2)
        rd_fft = np.fft.fftshift(rd_fft, axes=2)

        # Angle FFT
        # rda_fft = np.fft.fft(rd_fft, n=self.num_angle_bins, axis=1)
        # rda_fft = np.fft.fftshift(rda_fft, axes=1)

        # return rda_fft
        return rd_fft

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
        Two-stage CFAR detector:
        1) CASO-CFAR along range axis
        2) CA-CFAR along Doppler axis

        Input:
            radar_cube: [range_bin, virtual_ant, doppler_bin]

        Output:
            detections: list of (range_idx, doppler_idx)
        """

        # Sum across antennas to get integrated RD power
        range_doppler_power = np.sum(np.abs(radar_cube) ** 2, axis=1)

        # --- DC GUARD ZONE ---
        # Zero out the first 3-5 bins (adjust based on your range resolution)
        # This prevents the "blob" at 0m from being detected
        range_doppler_power[:4, :] = 0

        num_range_bins, num_doppler_bins = range_doppler_power.shape

        # Boolean maps for each CFAR stage
        range_pass = np.zeros_like(range_doppler_power, dtype=bool)
        doppler_pass = np.zeros_like(range_doppler_power, dtype=bool)

        # ============================================================
        # First pass: CASO-CFAR along RANGE axis
        # CASO = Cell Averaging Smaller Of
        #
        # For each CUT, compute:
        #   left noise estimate
        #   right noise estimate
        # Use the smaller of the two as the noise estimate.
        # ============================================================
        range_scale = 10 ** (range_threshold_db / 10.0)

        for d in range(num_doppler_bins):
            for r in range(range_train + range_guard, num_range_bins - (range_train + range_guard)):
                cut_power = range_doppler_power[r, d]

                # Training windows on left and right of CUT
                left_start = r - range_guard - range_train
                left_end = r - range_guard
                right_start = r + range_guard + 1
                right_end = r + range_guard + 1 + range_train

                left_noise = np.mean(range_doppler_power[left_start:left_end, d])
                right_noise = np.mean(range_doppler_power[right_start:right_end, d])

                # CASO picks the smaller of the two side estimates
                noise_est = min(left_noise, right_noise)
                threshold = noise_est * range_scale

                if cut_power > threshold and cut_power > 0:
                    range_pass[r, d] = True

        # ============================================================
        # Second pass: CA-CFAR along DOPPLER axis
        #
        # Only evaluate cells that already passed the range CASO test.
        # ============================================================
        doppler_scale = 10 ** (doppler_threshold_db / 10.0)

        for r in range(num_range_bins):
            for d in range(doppler_train + doppler_guard, num_doppler_bins - (doppler_train + doppler_guard)):
                if not range_pass[r, d]:
                    continue

                cut_power = range_doppler_power[r, d]

                # Training windows on left and right of CUT in Doppler
                left_start = d - doppler_guard - doppler_train
                left_end = d - doppler_guard
                right_start = d + doppler_guard + 1
                right_end = d + doppler_guard + 1 + doppler_train

                left_noise = range_doppler_power[r, left_start:left_end]
                right_noise = range_doppler_power[r, right_start:right_end]

                # Standard CA-CFAR uses both sides
                noise_est = np.mean(np.concatenate((left_noise, right_noise)))
                threshold = noise_est * doppler_scale

                if cut_power > threshold and cut_power > 0:
                    doppler_pass[r, d] = True

        # ============================================================
        # Final detections:
        # cell must pass both CFAR stages and be a local maximum
        # ============================================================
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

    def get_angle_spectrum(self, rd_cube, r_idx, d_idx, phase_sign=-1):
        """
        Build angle spectrum for one detected RD cell.

        Steps:
        1. Extract virtual antenna vector at (r_idx, d_idx)
        2. Apply per-detection TDM-MIMO Doppler compensation
        3. Run angle FFT
        """

        ant_vec = rd_cube[r_idx, :, d_idx].copy()

        # Signed Doppler bin after fftshift
        doppler_bin_signed = d_idx - (self.num_chirp_loops // 2)
        omega = 2.0 * np.pi * doppler_bin_signed / self.num_chirp_loops

        # TDM-MIMO Doppler phase compensation
        for tx_idx in range(self.num_tx):
            start_ant = tx_idx * self.num_rx
            end_ant = (tx_idx + 1) * self.num_rx
            phase_corr = np.exp(phase_sign * 1j * omega * (tx_idx / self.num_tx))
            ant_vec[start_ant:end_ant] *= phase_corr

        angle_fft = np.fft.fftshift(np.fft.fft(ant_vec, n=self.num_angle_bins))
        angle_spectrum = np.abs(angle_fft)**2

        return angle_spectrum


def run_radar_simulation():
    ss = stream_data_cs_team.dataStream()
    radar = RadarSensor(ss.data[0])

    print("ss.num_frames:", ss.num_frames)

    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file_path = os.path.join(log_dir, "radar_stream_log.txt")

    frame_dir = "rd_frames"
    if not os.path.exists(frame_dir):
        os.makedirs(frame_dir, exist_ok=True)

    plt.ion()
    fig = plt.figure(figsize=(10, 5))
    gs = fig.add_gridspec(1, 2)

    ax_rd = fig.add_subplot(gs[0, 0])
    ax_bev = fig.add_subplot(gs[0, 1])

    with open(log_file_path, "w", encoding="utf-8") as f:
        for frame_idx in range(ss.num_frames):
            start_time = time.time()

            frame_raw = ss.data[frame_idx]

            # test ---
            # range_win = np.hamming(radar.num_adc_samples)
            # simple_range_power = np.zeros(radar.num_adc_samples)

            # for tx in range(radar.num_tx):
            #     for loop in range(radar.num_chirp_loops):
            #         for rx in range(radar.num_rx):
            #             sig = frame_raw[tx, loop, rx, :]
            #             rng_fft = np.fft.fft(sig * range_win)
            #             simple_range_power += np.abs(rng_fft)**2

            # top_bins_simple = np.argsort(simple_range_power)[-10:][::-1]
            # print("Simple raw-cube range peaks:")
            # for b in top_bins_simple:
            #     print(f"  bin {b:3d} -> {radar.range_axis[b]:.3f} m, power={simple_range_power[b]:.3e}")

            # ----
            temp = frame_raw.transpose(3, 1, 0, 2)
            virtual_array = temp.reshape(
                radar.num_adc_samples,
                radar.num_chirp_loops,
                radar.num_tx * radar.num_rx
            )
            raw_data = virtual_array.transpose(0, 2, 1)

            # print("frame_raw:", frame_raw.shape)
            # print("raw_data:", raw_data.shape)

            # virtual antenna order:
            # [TX0-RX0, TX0-RX1, ..., TX1-RX0, ..., TX2-RX3]
            rd_cube = radar.process_radar_cube(raw_data)

            # top_bins = np.argsort(range_profile)[-10:][::-1]
            # print("Top 10 range bins:")
            # for b in top_bins:
            #     print(f"  bin {b:3d} -> {radar.range_axis[b]:.3f} m, power={range_profile[b]:.3e}")

            detections = radar.detect_targets_2d(rd_cube)

            # RD Plotting - Update data instead of clearing
            rd_amp = np.sqrt(np.sum(np.abs(rd_cube)**2, axis=1))
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

            # ax_rd.set_xlim(-8, 8)   # x-axis = velocity
            # ax_rd.set_ylim(0, 3)    # y-axis = range
            ax_rd.set_title(f"Range-Doppler Frame: {frame_idx}")
            ax_rd.set_xlabel("Velocity (m/s)")
            ax_rd.set_ylabel("Slant Range (m)")

            # for r_idx, d_idx in detections:
            #     # ax_rd.plot(radar.velocity_axis[d_idx], radar.range_axis[r_idx], 'ro', markersize=4)
            #     ax_rd.plot(interp_v, interp_r, 'ro', markersize=4)
            #     f.write(f"DETECT! velocity: {radar.velocity_axis[d_idx]} | range: {radar.range_axis[r_idx]}")

            # BEV Plotting
            ax_bev.cla()
            ax_bev.set_xlim(-10, 10)
            ax_bev.set_ylim(-10, 10)
            ax_bev.grid(True)
            ax_bev.plot(0, 0, 'r^')
            ax_bev.set_title(f"Bird's Eye View - Detections: {len(detections)}")
            ax_bev.set_xlabel("Lateral (m)")
            ax_bev.set_ylabel("Forward Range (m)")

            # Simple Raw-Cube Range Profile Debug Plot
            # ax_simple.cla()
            # ax_simple.plot(
            #     radar.range_axis,
            #     10.0 * np.log10(simple_range_power + EPSILON)
            # )
            # ax_simple.set_title("Simple Raw-Cube Range FFT")
            # ax_simple.set_xlabel("Range (m)")
            # ax_simple.set_ylabel("Power (dB)")
            # ax_simple.grid(True)

            processing_time = (time.time() - start_time) * 1000  # in ms
            f.write(f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | Latency: {processing_time:.2f}ms\n")

            for r_idx, d_idx in detections:
                # Interpolation Logic

                # Range interpolation
                if 0 < r_idx < radar.num_adc_samples - 1:
                    r_prev, r_next = r_idx - 1, r_idx + 1
                    y0 = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, d_idx])**2))
                    y_l = np.sqrt(np.sum(np.abs(rd_cube[r_prev, :, d_idx])**2))
                    y_r = np.sqrt(np.sum(np.abs(rd_cube[r_next, :, d_idx])**2))

                    # FIXED: Ensure interpolation results are scalars using .item()
                    interp_r = (
                        radar.range_axis[r_idx]
                        + (radar.range_axis[1] - radar.range_axis[0])
                        * ((y_l - y_r) / (2 * (y_l + y_r - 2 * y0 + EPSILON)))
                    ).item()
                else:
                    interp_r = radar.range_axis[r_idx].item()

                # Doppler interpolation
                d_prev, d_next = (d_idx - 1) % radar.num_chirp_loops, (d_idx + 1) % radar.num_chirp_loops
                v_l = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, d_prev])**2))
                v_r = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, d_next])**2))
                v0 = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, d_idx])**2))
                interp_v = (
                    radar.velocity_axis[d_idx]
                    + (radar.velocity_axis[1] - radar.velocity_axis[0])
                    * ((v_l - v_r) / (2 * (v_l + v_r - 2 * v0 + EPSILON)))
                ).item()
                
                ax_rd.plot(interp_v, interp_r, 'ro', markersize=4)
                f.write(
                    f"OBJECT DETECTEd! velocity: {interp_v:.3f} | slant range: {interp_r:.3f}\n"
                )

                # Build angle spectrum from:
                # 1) antenna vector at detected RD cell
                # 2) per-detection TDM-MIMO Doppler compensation
                # 3) angle FFT
                angle_spectrum = radar.get_angle_spectrum(rd_cube, r_idx, d_idx)
                ang_idx = np.argmax(angle_spectrum)

                a0 = angle_spectrum[ang_idx]  # current peak
                a_l = angle_spectrum[ang_idx - 1] if ang_idx > 0 else a0
                a_r = angle_spectrum[ang_idx + 1] if ang_idx < radar.num_angle_bins - 1 else a0

                if 0 < ang_idx < radar.num_angle_bins - 1:
                    # phase-based intrepolation
                    # p_l = np.log(a_l + EPSILON)
                    # p_0 = np.log(a0 + EPSILON)
                    # p_r = np.log(a_r + EPSILON)

                    # den = p_l - 2*p_0 + p_r

                    # if abs(den) < 1e-12:
                    #     frac_offset = 0.0
                    # else:
                    #     frac_offset = 0.5 * (p_l - p_r) / den

                    # frac_offset = np.clip(frac_offset, -0.5, 0.5)

                    # interp_r = radar.range_axis[r_idx] + frac_offset * (radar.range_axis[1] - radar.range_axis[0])

                    # log-based intrepolation
                    p_l = np.log(a_l + EPSILON)
                    p_0 = np.log(a0 + EPSILON)
                    p_r = np.log(a_r + EPSILON)

                    den = (p_l - 2 * p_0 + p_r)

                    if abs(den) < 1e-12:
                        frac_offset = 0.0
                    else:
                        frac_offset = 0.5 * (p_l - p_r) / den

                    frac_offset = np.clip(frac_offset, -0.5, 0.5)

                    # linear-based intrepolation
                    # frac_offset = (a_l - a_r) / (2 * (a_l + a_r - 2 * a0 + EPSILON))
                    # frac_offset = np.clip(frac_offset, -0.5, 0.5)
                    # ang_bin_width = radar.angle_axis[1] - radar.angle_axis[0]
                    # m_aoa_deg = (radar.angle_axis[ang_idx] + frac_offset * ang_bin_width).item()

                    u_interp = radar.spatial_freq_axis[ang_idx] + frac_offset * (
                        radar.spatial_freq_axis[1] - radar.spatial_freq_axis[0]
                    )
                    sin_theta_interp = np.clip(
                        u_interp * (radar.wavelength / radar.antenna_spacing),
                        -1.0,
                        1.0
                    )
                    m_aoa_deg = np.rad2deg(np.arcsin(sin_theta_interp)).item()

                else:
                    m_aoa_deg = radar.angle_axis[ang_idx].item()

                mx = interp_r * np.sin(np.deg2rad(m_aoa_deg))
                my = interp_r * np.cos(np.deg2rad(m_aoa_deg))
                
                bev_range = np.sqrt(mx**2 + my**2)
                print("RD range:", radar.range_axis[r_idx], "BEV slant range:", bev_range)

                ax_bev.plot(mx, my, 'bo')
                f.write(
                    f"  > R:{interp_r:6.3f}m | V:{interp_v:6.3f}m/s | "
                    f"A:{m_aoa_deg:6.2f}deg | X:{mx:6.3f} | Y:{my:6.3f}\n"
                )

            plt.tight_layout()
            plt.draw()
            plt.pause(0.001)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    run_radar_simulation()