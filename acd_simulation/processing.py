import os
import time
import numpy as np
import matplotlib.pyplot as plt
# import stream_data_cs_team

# global
EPSILON = 1e-12

class RadarSensor:
    def __init__(self, sample):
        f0 = 60e9
        fs = 12.5e6
        c = 299792458.0
        self.wavelength = c / f0

        self.num_tx = sample.shape[0]
        self.num_chirp_loops = sample.shape[1]
        self.num_rx = sample.shape[2]
        self.num_adc_samples = sample.shape[3]

        # Total chirps in the frame
        self.num_chirps = self.num_chirp_loops * self.num_tx

        bw = 3.41e9
        slope = bw / (self.num_adc_samples / fs)
        self.antenna_spacing = self.wavelength / 2.0

        range_res = c * fs / (2.0 * self.num_adc_samples * slope)
        self.range_axis = np.arange(self.num_adc_samples) * range_res

        chirp_time = self.num_adc_samples / fs
        active_frame_time = chirp_time * self.num_chirps
        velocity_res = self.wavelength / (2.0 * active_frame_time)
        doppler_bins = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)
        self.velocity_axis = doppler_bins * velocity_res

        # # Full virtual array size for TDM-MIMO
        # self.num_virtual_ant = self.num_tx * self.num_rx

        # # Capon / steering-vector grid
        # self.angle_res_deg = 1.0
        # self.angle_rng_deg = 90.0
        # self.angle_axis = np.arange(
        #     -self.angle_rng_deg,
        #     self.angle_rng_deg + self.angle_res_deg,
        #     self.angle_res_deg
        # )

        # # Half-wavelength ULA steering matrix
        # self.steering_vector = self.compute_steering_vector(
        #     num_ant=self.num_virtual_ant,
        #     angle_res=self.angle_res_deg,
        #     angle_rng=self.angle_rng_deg
        # )
        
        # self.steering_vector_rx_only = self.compute_steering_vector(
        #     num_ant=self.num_rx,
        #     angle_res=self.angle_res_deg,
        #     angle_rng=self.angle_rng_deg
        # )

        print(sample.shape)
        print("range_res: ", range_res)
        print("velocity_res: ", velocity_res)
        print("num_chirp_loops:", self.num_chirp_loops)
        print("num_chirps:", self.num_chirps)
        # print("num_virtual_ant:", self.num_virtual_ant)
        print("range axis:", self.range_axis[0], self.range_axis[-1])
        print("velocity axis:", self.velocity_axis[0], self.velocity_axis[-1])

    def process_tdm_mimo_cube(self, frame_raw):
        """
        frame_raw shape: [tx, chirp_loop, rx, adc_sample]

        Returns:
            range_cube:   [range_bin, tx, rx, chirp_loop]
            rd_cube_txrx: [range_bin, tx, rx, doppler_bin]
        """
        data = np.transpose(frame_raw, (3, 0, 2, 1))  # [adc, tx, rx, loop]

        range_win = np.hamming(self.num_adc_samples).reshape(-1, 1, 1, 1)
        doppler_win = np.hamming(self.num_chirp_loops).reshape(1, 1, 1, -1)

        # Range FFT
        range_cube = np.fft.fft(data * range_win, axis=0)

        # Doppler FFT
        rd_cube_txrx = np.fft.fft(range_cube * doppler_win, axis=3)
        rd_cube_txrx = np.fft.fftshift(rd_cube_txrx, axes=3)

        return range_cube, rd_cube_txrx
    
    # def get_angle_estimation(self, rd_map):
    #     rd_padded = np.pad(rd_map, pad_width=[(0, 0), (86, 87), (0, 0)], mode='constant')
    #     az_fft = np.fft.fftshift(np.fft.fft(rd_padded, axis=1), axes=1)
    #     az_power = np.abs(az_fft) ** 2
    #     beamformed_img = np.flipud(np.mean(az_power, axis=2))
    #     plt.figure(figsize=(5, 10))
    #     plt.imshow(np.log(beamformed_img))
    #     plt.show()
    
    def get_best_aoa_bin(self, range_cube, r_idx, tx_idx=0, width=4):
        candidates = []

        r_start = max(0, r_idx - width)
        r_end = min(range_cube.shape[0], r_idx + width + 1)

        for r in range(r_start, r_end):
            cell = range_cube[r, tx_idx, ::-1, :]   # match estimate_aoa RX order
            x = cell[:, 0]                          # match estimate_aoa chirp choice
            x = x * np.exp(-1j * np.angle(x[0]))

            # adjacent phase differences
            phase_diff = np.angle(x[1:] * np.conj(x[:-1]))
            phase_diff_avg = np.mean(phase_diff)

            theta = np.arcsin(np.clip(phase_diff_avg / np.pi, -1.0, 1.0))
            theta_deg = np.rad2deg(theta)

            power = np.sum(np.abs(cell) ** 2)

            candidates.append((r, power, theta_deg, x))

        best_r = r_idx
        best_score = -np.inf

        for i, (r, power, theta_deg, x) in enumerate(candidates):
            neighbor_thetas = []

            if i > 0:
                neighbor_thetas.append(candidates[i - 1][2])
            if i < len(candidates) - 1:
                neighbor_thetas.append(candidates[i + 1][2])

            if neighbor_thetas:
                consistency = -np.mean(np.abs(np.array(neighbor_thetas) - theta_deg))
            else:
                consistency = 0.0

            score = np.log(power + 1e-12) + 0.2 * consistency

            print(
                f"r={r}, range={self.range_axis[r]:.3f} m, power={power:.3e}, "
                f"theta={theta_deg:6.2f} deg, "
                f"phase={np.rad2deg(np.unwrap(np.angle(x)))}"
            )

            if score > best_score:
                best_score = score
                best_r = r

        print("BEST AOA IDX:", best_r)
        return best_r


    def estimate_aoa(self, range_cube, r_idx, tx_idx=0):
        cell = range_cube[r_idx, tx_idx, ::-1, :]  # same RX order as selector
        x = cell[:, 0]                             # same chirp choice as selector
        x = x * np.exp(-1j * np.angle(x[0]))

        phase_diff = np.angle(x[1:] * np.conj(x[:-1]))  # adjacent pairs
        phase_diff_avg = np.mean(phase_diff)

        theta = np.arcsin(np.clip(phase_diff_avg / np.pi, -1.0, 1.0))
        theta_deg = np.rad2deg(theta)

        print("mag:", np.abs(x))
        print("wrapped phase (deg):", np.rad2deg(np.angle(x)))
        print("unwrapped phase (deg):", np.rad2deg(np.unwrap(np.angle(x))))
        print("adjacent phase_diff (deg):", np.rad2deg(phase_diff))
        print("basic aoa (deg):", theta_deg)

        return theta_deg

    
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
        Two-stage CFAR detector
        Input: radar_cube: [range_bin, tx, rx, doppler_bin]
        Output: detections: list of (range_idx, doppler_idx)
        """

        # Sum across TX and RX to get integrated RD power
        range_doppler_power = np.sum(np.abs(radar_cube) ** 2, axis=(1, 2))

        # --- DC GUARD ZONE ---
        # Zero out the first 3-5 bins (adjust based on your range resolution)
        # This prevents the "blob" at 0m from being detected
        # range_doppler_power[:4, :] = 0

        num_range_bins, num_doppler_bins = range_doppler_power.shape
        # print("NUM OF BINS: ", range_doppler_power.shape)

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

    def inspect_range_bins_basic(self, range_cube, r_center, tx_idx=0, width=4):
        for r in range(max(0, r_center - width), min(range_cube.shape[0], r_center + width + 1)):
            X = range_cube[r, tx_idx, :, :]   # [rx, chirp]

            # first chirp only
            x = X[:, 0]
            x = x * np.exp(-1j * np.angle(x[0]))

            dphi = np.angle(x[1:] * np.conj(x[:-1]))
            theta_deg = np.rad2deg(np.arcsin(np.clip(np.mean(dphi) / np.pi, -1.0, 1.0)))

            power = np.sum(np.abs(X)**2)

            print(
                f"r={r:3d}, range={self.range_axis[r]:.3f} m, "
                f"power={power:.3e}, theta={theta_deg:7.2f} deg, "
                f"phase={np.rad2deg(np.unwrap(np.angle(x)))}"
            )


def run_radar_simulation(ss):
    # ss = stream_data_cs_team.dataStream()
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
    gs = fig.add_gridspec(2, 2)

    ax_rd = fig.add_subplot(gs[0, 0])
    ax_bev = fig.add_subplot(gs[0, 1])
    ax_rp = fig.add_subplot(gs[1, 0])
    # ax_raw_rp = fig.add_subplot(gs[1, 1])

    with open(log_file_path, "w", encoding="utf-8") as f:
        for frame_idx in range(ss.num_frames):
            start_time = time.time()

            raw_frame = ss.data[frame_idx]

            # virtual antenna order:
            # [TX0-RX0, TX0-RX1, ..., TX1-RX0, ..., TX2-RX3]
            range_cube, rd_cube = radar.process_tdm_mimo_cube(raw_frame)
            detections = radar.detect_targets_2d(rd_cube)

            # rd_cube => [range, tx, rx, doppler]
            rd_amp = np.sqrt(np.sum(np.abs(rd_cube)**2, axis=(1, 2)))
            rd_db = 20.0 * np.log10(rd_amp + EPSILON)

            ax_rd.cla()
            im = ax_rd.imshow(
                rd_db,
                aspect='auto',
                origin='lower',
                extent=[
                    radar.velocity_axis[0],
                    radar.velocity_axis[-1],
                    radar.range_axis[0],
                    radar.range_axis[-1]
                ],
                vmin=50,
                # vmax=100
            )
            
            if frame_idx == 0:
                cbar = fig.colorbar(im, ax=ax_rd)
                cbar.set_label("Power (dB)")
            else:
                cbar.update_normal(im)

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

            processing_time = (time.time() - start_time) * 1000  # in ms
            f.write(f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | Latency: {processing_time:.2f}ms\n")

            for r_idx, d_idx in detections:
                # Range interpolation
                r_prev, r_next = r_idx - 1, r_idx + 1
                y0 = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_idx])**2))
                y_l = np.sqrt(np.sum(np.abs(rd_cube[r_prev, :, :, d_idx])**2))
                y_r = np.sqrt(np.sum(np.abs(rd_cube[r_next, :, :, d_idx])**2))

                interp_r = (
                    radar.range_axis[r_idx]
                    + (radar.range_axis[1] - radar.range_axis[0])
                    * ((y_l - y_r) / (2 * (y_l + y_r - 2 * y0 + EPSILON)))
                ).item()

                # Doppler interpolation
                d_prev = (d_idx - 1) % radar.num_chirp_loops
                d_next = (d_idx + 1) % radar.num_chirp_loops
                v_l = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_prev])**2))
                v_r = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_next])**2))
                v0 = np.sqrt(np.sum(np.abs(rd_cube[r_idx, :, :, d_idx])**2))
                
                interp_v = (
                    radar.velocity_axis[d_idx]
                    + (radar.velocity_axis[1] - radar.velocity_axis[0])
                    * ((v_l - v_r) / (2 * (v_l + v_r - 2 * v0 + EPSILON)))
                ).item()
                
                ax_rd.plot(interp_v, interp_r, 'ro', markersize=4)
                f.write(
                    f"OBJECT DETECTED! velocity: {interp_v:.3f} | slant range: {interp_r:.3f}\n"
                )
                
                ax_rp.cla()

                rp = np.sqrt(np.sum(np.abs(rd_cube)**2, axis=(1, 2, 3)))
                rp_db = 20.0 * np.log10(rp + EPSILON)

                ax_rp.plot(radar.range_axis, rp_db, 'b-')
                ax_rp.set_title("Integrated Range Profile")
                ax_rp.set_xlabel("Range (m)")
                ax_rp.set_ylabel("Power (dB)")
                ax_rp.grid(True)
                
                # ax_raw_rp.cla()

                # # raw range fft profile
                # data = np.transpose(raw_frame, (3, 0, 2, 1))  # [adc, tx, rx, chirp]

                # range_win = np.hamming(radar.num_adc_samples).reshape(-1, 1, 1, 1)
                # range_fft = np.fft.fft(data * range_win, axis=0)

                # # Flatten all TX/RX/chirp traces, keep range bins on axis 0
                # range_fft_mag = np.abs(range_fft).reshape(radar.num_adc_samples, -1)
                # range_fft_db = 20.0 * np.log10(range_fft_mag + EPSILON)
                # ax_raw_rp.plot(radar.range_axis, range_fft_db)
                # ax_raw_rp.set_title("Raw Range FFT (All Chirps/RX/TX)")
                # ax_raw_rp.set_xlabel("Range (m)")
                # ax_raw_rp.set_ylabel("Power (dB)")
                # ax_raw_rp.grid(True)

                r_aoa_idx = radar.get_best_aoa_bin(range_cube, r_idx, tx_idx=0, width=4)
                m_aoa_deg = radar.estimate_aoa(range_cube, r_aoa_idx, tx_idx=0)
                radar.inspect_range_bins_basic(range_cube, r_idx, tx_idx=0, width=4)
                
                # angle_spectrum = radar.get_angle_spectrum(range_cube, r_idx, d_idx)
                # ang_idx = np.argmax(angle_spectrum)

                # a0 = angle_spectrum[ang_idx]
                # a_l = angle_spectrum[ang_idx - 1] if ang_idx > 0 else a0
                # a_r = angle_spectrum[ang_idx + 1] if ang_idx < len(angle_spectrum) - 1 else a0

                # if 0 < ang_idx < len(angle_spectrum) - 1:
                #     p_l = np.log(a_l + EPSILON)
                #     p_0 = np.log(a0 + EPSILON)
                #     p_r = np.log(a_r + EPSILON)

                #     denom = (p_l - 2 * p_0 + p_r)
                #     frac_offset = 0.0 if abs(denom) < 1e-12 else 0.5 * (p_l - p_r) / denom
                #     frac_offset = np.clip(frac_offset, -0.5, 0.5)

                #     m_aoa_deg = (
                #         radar.angle_axis[ang_idx]
                #         + frac_offset * (radar.angle_axis[1] - radar.angle_axis[0])
                #     ).item()
                # else:
                #     m_aoa_deg = radar.angle_axis[ang_idx].item()

                mx = interp_r * np.sin(np.deg2rad(m_aoa_deg))
                my = interp_r * np.cos(np.deg2rad(m_aoa_deg))
                
                bev_range = np.sqrt(mx**2 + my**2)
                print("RD range:", radar.range_axis[r_idx]) 
                print("BEV slant range:", bev_range)
                print("angle (deg): ", m_aoa_deg)
                
                # print("corrected (v1):", m_aoa_deg - 90)
                # print("corrected (v2):", 90 - m_aoa_deg)
                # print("mx:", mx)
                # print("my:", my)

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