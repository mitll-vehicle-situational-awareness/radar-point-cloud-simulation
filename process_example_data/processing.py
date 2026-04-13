import os
import time
import numpy as np
import matplotlib.pyplot as plt
# import stream_data_cs_team

# global
EPSILON = 1e-12
DEBUG_RD_LOGIC = True
MIN_RANGE = 0 # meters
MAX_RANGE = 2 # meters


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
        # self.virtual_

        bw = 3.41e9
        slope = bw / (self.num_adc_samples / fs)
        self.antenna_spacing = self.wavelength / 2.0

        self.range_res = c * fs / (2.0 * self.num_adc_samples * slope)
        self.range_axis = np.linspace(0, self.num_adc_samples * self.range_res, self.num_adc_samples)

        chirp_time = self.num_adc_samples / fs
        active_frame_time = chirp_time * self.num_chirps
        velocity_res = self.wavelength / (2.0 * active_frame_time)
        self.doppler_fft_size = int(2 ** np.ceil(np.log2(self.num_chirp_loops)))

        # For TDM-MIMO, each TX contributes one chirp every num_tx chirps.
        slow_time_dt = chirp_time * self.num_tx
        doppler_freq = np.fft.fftshift(np.fft.fftfreq(self.doppler_fft_size, d=slow_time_dt))
        self.velocity_axis = doppler_freq * (self.wavelength / 2.0)

        print(sample.shape)
        print("range_res: ", self.range_res)
        print("velocity_res (native): ", velocity_res)
        print("doppler_fft_size:", self.doppler_fft_size)
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
        data = np.flip(data, axis=0) # 0-idx = acd_samples

        range_win = np.hamming(self.num_adc_samples).reshape(-1, 1, 1, 1)
        doppler_win = np.hamming(self.num_chirp_loops).reshape(1, 1, 1, -1)

        # Range FFT
        range_cube = np.fft.fft(data * range_win, axis=0)

        # Apply Range Compensation
        N = range_cube.shape[0] # num of range samples
        r = np.arange(0, N)
        compensation = np.minimum(r**2, (N/2)**2)
        range_cube *= compensation[:, None, None, None]

        # Doppler FFT
        rd_cube_txrx = np.fft.fft(range_cube * doppler_win, axis=3, n=self.doppler_fft_size)
        rd_cube_txrx = np.fft.fftshift(rd_cube_txrx, axes=3)

        return range_cube, rd_cube_txrx
    
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
        Two-stage CFAR detector on the range-doppler map.

        radar_cube shape: [range_bin, tx, rx, doppler_bin]
        returns: list of (range_idx, doppler_idx)
        """
        # Collapse TX/RX so CFAR runs on a 2D RD power map.
        range_doppler_power = np.sum(np.abs(radar_cube) ** 2, axis=(1, 2))

        num_range_bins, num_doppler_bins = range_doppler_power.shape

        # Per-stage pass masks.
        range_pass = np.zeros_like(range_doppler_power, dtype=bool)
        doppler_pass = np.zeros_like(range_doppler_power, dtype=bool)

        # Pass 1: CASO-CFAR along range, independently for each doppler bin.
        # CASO = take the smaller left/right noise estimate.
        range_scale = 10 ** (range_threshold_db / 10.0)

        for d in range(num_doppler_bins):
            for r in range(range_train + range_guard, num_range_bins - (range_train + range_guard)):
                cut_power = range_doppler_power[r, d]

                # Training windows around CUT (guard cells excluded).
                left_start = r - range_guard - range_train
                left_end = r - range_guard
                right_start = r + range_guard + 1
                right_end = r + range_guard + 1 + range_train

                left_noise = np.mean(range_doppler_power[left_start:left_end, d])
                right_noise = np.mean(range_doppler_power[right_start:right_end, d])

                # Use the quieter side to reduce target leakage into threshold.
                noise_est = min(left_noise, right_noise)
                threshold = noise_est * range_scale

                if cut_power > threshold and cut_power > 0:
                    range_pass[r, d] = True

        # Pass 2: CA-CFAR along doppler, only for cells that passed pass 1.
        doppler_scale = 10 ** (doppler_threshold_db / 10.0)

        for r in range(num_range_bins):
            for d in range(doppler_train + doppler_guard, num_doppler_bins - (doppler_train + doppler_guard)):
                if not range_pass[r, d]:
                    continue

                cut_power = range_doppler_power[r, d]

                # Doppler training windows around CUT.
                left_start = d - doppler_guard - doppler_train
                left_end = d - doppler_guard
                right_start = d + doppler_guard + 1
                right_end = d + doppler_guard + 1 + doppler_train

                left_noise = range_doppler_power[r, left_start:left_end]
                right_noise = range_doppler_power[r, right_start:right_end]

                # Standard CA-CFAR averages both sides.
                noise_est = np.mean(np.concatenate((left_noise, right_noise)))
                threshold = noise_est * doppler_scale

                if cut_power > threshold and cut_power > 0:
                    doppler_pass[r, d] = True

        # Final gate: keep local 3x3 maxima from cells that passed both stages.
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
    
    # phaser = np.exp(j * 2 * np.pi * np.sin(theta))
    # x1 x2
    # x3 x4
    
    # adding the phase shift to the opposite side that we want to look
    # 
    
    
    # def get_angle_spectrum_from_virtual_array(self, rd_cube_txrx, r_idx, d_idx):         
    #     ant_slice = rd_cube_txrx[r_idx, :, :, d_idx].copy()

    #     # convert shifted Doppler index to signed Doppler bin
    #     doppler_bin_signed = d_idx - (self.num_chirp_loops // 2)
    #     omega = 2.0 * np.pi * doppler_bin_signed / self.num_chirp_loops

    #     # TDM-MIMO compensation:
    #     # TX1 delay = 0 chirps
    #     # TX2 delay = 1 chirp
    #     # TX3 delay = 2 chirps
    #     for tx_idx in range(self.num_tx):
    #         phase_corr = np.exp(-1j * omega * tx_idx)
    #         ant_slice[tx_idx, :] *= phase_corr

    #     # Flatten as [TX0RX0...RXn, TX1RX0...RXn, ...]
    #     ant_vec = ant_slice.reshape(self.num_tx * self.num_rx)
    #     angle_fft = np.fft.fftshift(np.fft.fft(ant_vec, n=self.num_angle_bins))
    #     angle_spectrum = np.abs(angle_fft) ** 2
    #     return angle_spectrum

    def compute_aoa_fft(self, range_cube, r_idx, d_idx, tx_idx=0):   
        """
        Compute angle of arrival (AoA) in azimuth and elevation using 2D FFT.
        This method performs 2D FFT on a 2x2 antenna grid to estimate the direction
        of arrival of a signal at a specific range bin and Doppler bin.
        Args:
            range_cube: 4D array of radar data with shape (range, tx, rx, doppler)
            r_idx (int): Range bin index
            d_idx (int): Doppler bin index
            tx_idx (int, optional): Transmitter index. Defaults to 0.
        Returns:
            tuple: (az_deg, el_deg) where:
                - az_deg (float): Azimuth angle in degrees
                - el_deg (float): Elevation angle in degrees
        Notes:
            - Extracts 4 receiver antenna samples at the specified range and Doppler bins
            - Reshapes antennas into 2x2 spatial grid based on antenna geometry
            - Applies 2D FFT with 64x64 zero-padding
            - Finds peak in power spectrum (magnitude squared)
            - Converts FFT bin indices to angles using arcsin mapping
        """
        # two pairs across azimuth / elevation
        # compute the phase shift between the pair
        
        ant = range_cube[r_idx, tx_idx, :, d_idx] # [RX0, RX1, RX2, RX3]
    
        # Reshape into 2x2 grid based on AOP geometry
        
        ant_grid = np.array([
            [ant[0], ant[2]], # 1, 3
            [ant[1], ant[3]] # 2, 4
        ])
        
        # zero-padding 128x128
        aoa_2d_fft = np.fft.fftshift(np.fft.fft2(ant_grid, s=(128, 128)))
        mag_sq = np.abs(aoa_2d_fft)**2 # power spectrum
        
        # find argmax -> turn idx into tuple of coordinates matching og shape
        el_idx, az_idx = np.unravel_index(np.argmax(mag_sq), mag_sq.shape)

        # Convert bins to angles
        bins = np.fft.fftshift(np.fft.fftfreq(128))
        az_deg = np.degrees(np.arcsin(np.clip(2 * bins[az_idx], -1.0, 1.0)))
        el_deg = np.degrees(np.arcsin(np.clip(2 * bins[el_idx], -1.0, 1.0)))
        
        return az_deg, el_deg
    
    def get_filtered_rng_dop(self, rd_cube, r_idx, d_idx):
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

    def get_range_azimuth_subset(self, rd_cube, min_range, max_range):
        # dist_axis = np.linspace(0, NUM_ADC_SAMPLES * RANGE_RESOLUTION, NUM_ADC_SAMPLES)
        dist_axis = np.linspace(0, self.num_adc_samples * self.range_res, self.num_adc_samples)

        min_dist = np.abs(dist_axis - min_range)
        max_dist = np.abs(dist_axis - max_range)

        min_index = np.argmin(min_dist)
        max_index = np.argmin(max_dist)

        print(f'min index: {min_index}')
        print(f'max index: {max_index}')

        return rd_cube[min_index : max_index, :, :, :], dist_axis[min_index : max_index]

def _prepare_output_dirs():
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file_path = os.path.join(log_dir, "radar_stream_log.txt")

    frame_dir = "rd_frames"
    if not os.path.exists(frame_dir):
        os.makedirs(frame_dir, exist_ok=True)

    return log_file_path


def _plot_range_doppler(ax_rd, fig, cbar, radar, rd_cube, frame_idx):
    # rd_cube => [range, tx, rx, doppler]
    # Sum TX/RX power so visualization is consistent with the CFAR detector basis.
    rd_power = np.sum(np.abs(rd_cube) ** 2, axis=(1, 2))
    rd_db = 10.0 * np.log10(rd_power + EPSILON)
    
    print("FRAME IDX: ", frame_idx)

    if DEBUG_RD_LOGIC and frame_idx == 0:
        expected_shape = (radar.range_axis.size, radar.velocity_axis.size)
        print(f"[RD DEBUG] rd_db shape: {rd_db.shape}, expected: {expected_shape}")
        if rd_db.shape == expected_shape:
            print("[RD DEBUG] Orientation check: [range, doppler] (no transpose needed).")
        elif rd_db.shape == (expected_shape[1], expected_shape[0]):
            print("[RD DEBUG] Orientation check: [doppler, range] (transpose likely needed).")
        else:
            print("[RD DEBUG] Orientation check: unexpected shape.")

        # Horizontal mirror check: compare integrated energy at negative vs positive Doppler.
        center_d = rd_db.shape[1] // 2
        neg_energy = np.mean(rd_db[:, :center_d])
        pos_energy = np.mean(rd_db[:, center_d:])
        print(
            "[RD DEBUG] Doppler-side mean power (dB) | "
            f"negative: {neg_energy:.2f}, positive: {pos_energy:.2f}, delta(pos-neg): {pos_energy - neg_energy:.2f}"
        )

        # Vertical flip check: compare near-range vs far-range energy.
        center_r = rd_db.shape[0] // 2
        near_energy = np.mean(rd_db[:center_r, :])
        far_energy = np.mean(rd_db[center_r:, :])
        print(
            "[RD DEBUG] Range-half mean power (dB) | "
            f"near: {near_energy:.2f}, far: {far_energy:.2f}, delta(far-near): {far_energy - near_energy:.2f}"
        )

    ax_rd.cla()
    plot_data = rd_db
    x_axis = radar.velocity_axis
    y_axis = radar.range_axis
    x_label = "Doppler Velocity (m/s)"
    y_label = "Range (m)"

    mesh = ax_rd.pcolormesh(
        x_axis,
        y_axis,
        plot_data,
        shading='auto',
        vmin=np.max(plot_data) - 40.0,
        vmax=np.max(plot_data),
    )

    if cbar is None:
        cbar = fig.colorbar(mesh, ax=ax_rd)
        cbar.set_label("Power (dB)")
    else:
        cbar.update_normal(mesh)

    ax_rd.set_title(f"Range-Doppler Frame: {frame_idx}")
    ax_rd.set_xlabel(x_label)
    ax_rd.set_ylabel(y_label)

    return cbar


def _plot_bev(ax_bev, detections):
    ax_bev.cla()
    ax_bev.set_xlim(-10, 10)
    ax_bev.set_ylim(-10, 10)
    ax_bev.grid(True)
    ax_bev.plot(0, 0, 'r^')
    ax_bev.set_title(f"Bird's Eye View - Detections: {len(detections)}")
    ax_bev.set_xlabel("Lateral (m)")
    ax_bev.set_ylabel("Forward Range (m)")


def _plot_range_profile(ax_rp, radar, rd_cube):
    ax_rp.cla()
    rp = np.sqrt(np.sum(np.abs(rd_cube) ** 2, axis=(1, 2, 3)))
    rp_db = 20.0 * np.log10(rp + EPSILON)

    ax_rp.plot(radar.range_axis, rp_db, 'b-')
    ax_rp.set_title("Range Profile (Sum of TX and RX)")
    ax_rp.set_xlabel("Range (m)")
    ax_rp.set_ylabel("Power (dB)")
    ax_rp.grid(True)


def _process_detections(ax_rd, ax_bev, ax_rp, radar, range_cube, rd_cube, detections, log_handle):
    for r_idx, d_idx in detections:
        interp_r, interp_v = radar.get_filtered_rng_dop(rd_cube, r_idx, d_idx)
        ax_rd.plot(interp_v, interp_r, 'ro', markersize=4)
        log_handle.write(
            f"OBJECT DETECTED! velocity: {interp_v:.3f} | slant range: {interp_r:.3f}\n"
        )

        _plot_range_profile(ax_rp, radar, rd_cube)

        m_aoa_deg, m_elev_deg = radar.compute_aoa_fft(range_cube, r_idx, d_idx)

        print("AZIMUTH: ", m_aoa_deg)
        print("ELEVATION: ", m_elev_deg)

        mx = interp_r * np.sin(np.deg2rad(m_aoa_deg))
        my = interp_r * np.cos(np.deg2rad(m_aoa_deg))

        bev_range = np.sqrt(mx**2 + my**2)
        print("RD range:", radar.range_axis[r_idx])
        print("BEV slant range:", bev_range)
        # print("angle (deg): ", m_aoa_deg)
        # print("mx:", mx)
        # print("my:", my)

        ax_bev.plot(mx, my, 'bo')
        log_handle.write(
            f"  > R:{interp_r:6.3f}m | V:{interp_v:6.3f}m/s | "
            f"A:{m_aoa_deg:6.2f}deg | X:{mx:6.3f} | Y:{my:6.3f}\n"
        )


def run_radar_simulation(ss):
    # ss = stream_data_cs_team.dataStream()
    radar = RadarSensor(ss.data[0])

    print("ss.num_frames:", ss.num_frames)
    log_file_path = _prepare_output_dirs()

    plt.ion()
    fig = plt.figure(figsize=(10, 5))
    gs = fig.add_gridspec(2, 2)

    ax_rd = fig.add_subplot(gs[0, 0])
    ax_bev = fig.add_subplot(gs[0, 1])
    ax_rp = fig.add_subplot(gs[1, 0])
    # ax_raw_rp = fig.add_subplot(gs[1, 1])
    cbar = None

    with open(log_file_path, "w", encoding="utf-8") as f:
        for frame_idx in range(ss.num_frames):
            start_time = time.time()

            raw_frame = ss.data[frame_idx]

            # virtual antenna order:
            # [TX0-RX0, TX0-RX1, ..., TX1-RX0, ..., TX2-RX3]
            range_cube, rd_cube = radar.process_tdm_mimo_cube(raw_frame)
            
            if MAX_RANGE is None or MIN_RANGE is None:
                rd_cube_trimmed = rd_cube
            else:
                rd_cube_trimmed, dist_axis = radar.get_range_azimuth_subset(rd_cube, MIN_RANGE, MAX_RANGE)
                radar.range_axis = dist_axis

            detections = radar.detect_targets_2d(rd_cube_trimmed)

            cbar = _plot_range_doppler(ax_rd, fig, cbar, radar, rd_cube_trimmed, frame_idx)
            _plot_bev(ax_bev, detections)

            processing_time = (time.time() - start_time) * 1000  # in ms
            f.write(f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | Latency: {processing_time:.2f}ms\n")

            _process_detections(ax_rd, ax_bev, ax_rp, radar, range_cube, rd_cube_trimmed, detections, f)

            plt.tight_layout()
            plt.draw()
            plt.pause(0.001)

    plt.ioff()
    plt.show()