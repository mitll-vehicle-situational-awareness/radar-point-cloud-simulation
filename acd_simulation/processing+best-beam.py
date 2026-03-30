import os
import time
import numpy as np
import matplotlib.pyplot as plt
import stream_data_cs_team

EPSILON = 1e-12


class RadarSensor:
    def __init__(self, sample):
        self.f0 = 60e9
        self.fs = 12.5e6
        self.c = 299792458.0
        self.wavelength = self.c / self.f0
        self.antenna_spacing = self.wavelength / 2.0

        # sample shape: [tx, chirp_loop, rx, adc]
        self.num_tx = sample.shape[0]
        self.num_chirp_loops = sample.shape[1]
        self.num_rx = sample.shape[2]
        self.num_adc_samples = sample.shape[3]

        self.num_antennas = self.num_tx * self.num_rx
        self.num_chirps = self.num_tx * self.num_chirp_loops

        # keep aligned with acquisition config
        self.bw = 3.41e9
        self.range_resolution = 0.044
        self.velocity_resolution = 1.26

        self.range_axis = np.arange(self.num_adc_samples) * self.range_resolution
        doppler_bins = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)
        self.velocity_axis = doppler_bins * self.velocity_resolution

        # RX-only angle FFT after TX beamforming
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

    def tx_steering_weights(self, theta_deg):
        m_ind = np.arange(self.num_tx)
        phase_rad = 2.0 * np.pi * (m_ind * np.sin(np.deg2rad(theta_deg)))
        w_tx = np.exp(-1j * phase_rad)
        w_tx = w_tx / np.sqrt(self.num_tx)
        return w_tx

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

        w_tx = self.tx_steering_weights(theta_deg)

        d_signed = np.arange(self.num_chirp_loops) - (self.num_chirp_loops // 2)
        omega = 2.0 * np.pi * d_signed / self.num_chirp_loops

        tx_idx = np.arange(self.num_tx)[:, None]
        phase_corr = np.exp(phase_sign * 1j * (tx_idx * omega[None, :]))

        tx_combiner = w_tx[:, None] * phase_corr

        # [range, tx, rx, doppler] -> [range, rx, doppler]
        beam_cube_rx = np.einsum("rtxd,td->rxd", rd_cube_txrx, tx_combiner)
        return beam_cube_rx

    def integrate_beam_rd_power(self, beam_cube_rx):
        return np.sum(np.abs(beam_cube_rx) ** 2, axis=1)

    def sweep_tx_beams(self, rd_cube_txrx, beam_angles_deg, phase_sign=-1):
        beam_rd_maps = []
        beam_cube_list = []

        for theta_deg in beam_angles_deg:
            beam_cube_rx = self.beamform_tx_direction(
                rd_cube_txrx,
                theta_deg=theta_deg,
                phase_sign=phase_sign,
            )
            rd_power = self.integrate_beam_rd_power(beam_cube_rx)
            beam_cube_list.append(beam_cube_rx)
            beam_rd_maps.append(rd_power)

        # [beam, range, doppler] -> [range, doppler, beam]
        beam_rd_cube = np.stack(beam_rd_maps, axis=0)
        beam_rd_cube = np.transpose(beam_rd_cube, (1, 2, 0))

        return beam_rd_cube, beam_cube_list

    def get_rx_angle_spectrum(self, beam_cube_rx, r_idx, d_idx):
        rx_vec = beam_cube_rx[r_idx, :, d_idx]
        angle_fft = np.fft.fftshift(np.fft.fft(rx_vec, n=self.num_angle_bins))
        return np.abs(angle_fft) ** 2

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
        rd_power[:4, :] = 0

        num_range_bins, num_doppler_bins = rd_power.shape

        range_pass = np.zeros_like(rd_power, dtype=bool)
        doppler_pass = np.zeros_like(rd_power, dtype=bool)

        range_scale = 10 ** (range_threshold_db / 10.0)
        doppler_scale = 10 ** (doppler_threshold_db / 10.0)

        # range CASO
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

        # Doppler CA
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
                max(0, r - 1): min(num_range_bins, r + 2),
                max(0, d - 1): min(num_doppler_bins, d + 2),
            ]
            if rd_power[r, d] == np.max(region):
                detections.append((r, d))

        return detections


def quadratic_interp_1d(y_l, y_0, y_r, bin_center, bin_spacing):
    denom = 2.0 * (y_l + y_r - 2.0 * y_0 + EPSILON)
    if abs(denom) < 1e-12:
        delta = 0.0
    else:
        delta = (y_l - y_r) / denom
    delta = np.clip(delta, -0.5, 0.5)
    return bin_center + delta * bin_spacing


def estimate_rx_aoa_deg(radar, beam_cube_rx, r_idx, d_idx):
    angle_spectrum = radar.get_rx_angle_spectrum(beam_cube_rx, r_idx, d_idx)
    ang_idx = int(np.argmax(angle_spectrum))

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

    return float(aoa_deg)


def interpolate_beam_angle_deg(beam_rd_cube, beam_angles_deg, r_idx, d_idx, b_idx):
    num_beams = beam_rd_cube.shape[2]

    if b_idx <= 0 or b_idx >= num_beams - 1:
        return float(beam_angles_deg[b_idx])

    p_l = np.log(beam_rd_cube[r_idx, d_idx, b_idx - 1] + EPSILON)
    p_0 = np.log(beam_rd_cube[r_idx, d_idx, b_idx] + EPSILON)
    p_r = np.log(beam_rd_cube[r_idx, d_idx, b_idx + 1] + EPSILON)

    denom = 2.0 * (p_l + p_r - 2.0 * p_0)
    if abs(denom) < 1e-12:
        delta = 0.0
    else:
        delta = (p_l - p_r) / denom

    delta = np.clip(delta, -0.5, 0.5)
    beam_spacing = beam_angles_deg[1] - beam_angles_deg[0]

    return float(beam_angles_deg[b_idx] + delta * beam_spacing)


def nms_bev_detections(det_list, range_merge_m=0.35, lateral_merge_m=0.35, vel_merge_mps=1.0):
    if not det_list:
        return []

    det_sorted = sorted(det_list, key=lambda d: d["power"], reverse=True)
    kept = []

    for cand in det_sorted:
        suppress = False
        for ref in kept:
            dx = cand["x_m"] - ref["x_m"]
            dy = cand["y_m"] - ref["y_m"]
            dv = cand["vel_mps"] - ref["vel_mps"]

            if abs(dx) <= lateral_merge_m and abs(dy) <= range_merge_m and abs(dv) <= vel_merge_mps:
                suppress = True
                break

        if not suppress:
            kept.append(cand)

    return kept


def run_radar_simulation():
    ss = stream_data_cs_team.dataStream()
    radar = RadarSensor(ss.data[0])

    print("ss.num_frames:", ss.num_frames)

    beam_angles_deg = np.linspace(-60.0, 60.0, 25)

    use_beam_interp = True
    use_rx_fft_for_fine_aoa = False
    use_final_bev_nms = True

    log_dir = "logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file_path = os.path.join(log_dir, "radar_stream_log_multibeam_hybrid_debug.txt")

    plt.ion()
    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(2, 2)

    ax_rd = fig.add_subplot(gs[0, 0])
    ax_ra = fig.add_subplot(gs[0, 1])
    ax_bev = fig.add_subplot(gs[1, 0])
    ax_beam = fig.add_subplot(gs[1, 1])

    cbar_rd = None
    cbar_ra = None

    with open(log_file_path, "w", encoding="utf-8") as f:
        for frame_idx in range(ss.num_frames):
            start_time = time.time()
            raw_frame = ss.data[frame_idx]

            # 1) build TDM cube
            rd_cube_txrx = radar.process_tdm_mimo_cube(raw_frame)

            # 2) sweep beams
            beam_rd_cube, beam_cube_list = radar.sweep_tx_beams(
                rd_cube_txrx,
                beam_angles_deg,
                phase_sign=-1,
            )
            # [range, doppler, beam]

            # 3) clean detection map
            best_beam_idx = np.argmax(beam_rd_cube, axis=2)
            best_rd_power = np.max(beam_rd_cube, axis=2)

            # 4) detect once
            detections = radar.detect_targets_2d(best_rd_power)

            # 5) visualization maps
            range_angle_map = np.max(beam_rd_cube, axis=1)

            # RD plot
            rd_amp = np.sqrt(best_rd_power)
            rd_db = 20.0 * np.log10(rd_amp + EPSILON)

            ax_rd.cla()
            im_rd = ax_rd.imshow(
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
            ax_rd.set_title(f"Hybrid Best-Beam RD - Frame {frame_idx}")
            ax_rd.set_xlabel("Velocity (m/s)")
            ax_rd.set_ylabel("Slant Range (m)")

            if cbar_rd is None:
                cbar_rd = fig.colorbar(im_rd, ax=ax_rd)
                cbar_rd.set_label("Power (dB)")
            else:
                cbar_rd.update_normal(im_rd)

            # Range-angle plot
            ra_db = 20.0 * np.log10(np.sqrt(range_angle_map) + EPSILON)

            ax_ra.cla()
            im_ra = ax_ra.imshow(
                ra_db,
                aspect="auto",
                origin="lower",
                extent=[
                    beam_angles_deg[0],
                    beam_angles_deg[-1],
                    radar.range_axis[0],
                    radar.range_axis[-1],
                ],
            )
            ax_ra.set_title(f"Range-Angle Map - Frame {frame_idx}")
            ax_ra.set_xlabel("Beam Angle (deg)")
            ax_ra.set_ylabel("Slant Range (m)")

            if cbar_ra is None:
                cbar_ra = fig.colorbar(im_ra, ax=ax_ra)
                cbar_ra.set_label("Power (dB)")
            else:
                cbar_ra.update_normal(im_ra)

            det_candidates = []

            for r_idx, d_idx in detections:
                # range interpolation
                if 0 < r_idx < radar.num_adc_samples - 1:
                    y_l = np.sqrt(best_rd_power[r_idx - 1, d_idx])
                    y_0 = np.sqrt(best_rd_power[r_idx, d_idx])
                    y_r = np.sqrt(best_rd_power[r_idx + 1, d_idx])
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
                v_l = np.sqrt(best_rd_power[r_idx, d_prev])
                v_0 = np.sqrt(best_rd_power[r_idx, d_idx])
                v_r = np.sqrt(best_rd_power[r_idx, d_next])

                interp_v = quadratic_interp_1d(
                    v_l,
                    v_0,
                    v_r,
                    radar.velocity_axis[d_idx],
                    radar.velocity_axis[1] - radar.velocity_axis[0],
                )

                # coarse winning beam
                b_idx = int(best_beam_idx[r_idx, d_idx])
                coarse_beam_angle_deg = float(beam_angles_deg[b_idx])

                # beam interpolation
                if use_beam_interp:
                    beam_angle_deg = interpolate_beam_angle_deg(
                        beam_rd_cube,
                        beam_angles_deg,
                        r_idx,
                        d_idx,
                        b_idx,
                    )
                else:
                    beam_angle_deg = coarse_beam_angle_deg

                # optional RX-only refinement
                if use_rx_fft_for_fine_aoa:
                    winning_beam_cube = beam_cube_list[b_idx]
                    aoa_deg = estimate_rx_aoa_deg(radar, winning_beam_cube, r_idx, d_idx)
                else:
                    aoa_deg = beam_angle_deg

                x = interp_r * np.sin(np.deg2rad(aoa_deg))
                y = interp_r * np.cos(np.deg2rad(aoa_deg))

                det_candidates.append({
                    "r_idx": int(r_idx),
                    "d_idx": int(d_idx),
                    "beam_idx": b_idx,
                    "range_m": float(interp_r),
                    "vel_mps": float(interp_v),
                    "coarse_beam_deg": float(coarse_beam_angle_deg),
                    "beam_interp_deg": float(beam_angle_deg),
                    "angle_deg": float(aoa_deg),
                    "x_m": float(x),
                    "y_m": float(y),
                    "power": float(best_rd_power[r_idx, d_idx]),
                })

            if use_final_bev_nms:
                det_final = nms_bev_detections(
                    det_candidates,
                    range_merge_m=0.35,
                    lateral_merge_m=0.35,
                    vel_merge_mps=1.0,
                )
            else:
                det_final = det_candidates

            # BEV plot
            ax_bev.cla()
            ax_bev.set_xlim(-10, 10)
            ax_bev.set_ylim(0, 10)
            ax_bev.grid(True)
            ax_bev.plot(0, 0, "r^", markersize=8)
            ax_bev.set_title(f"BEV - RAW:{len(det_candidates)} KEPT:{len(det_final)}")
            ax_bev.set_xlabel("Lateral (m)")
            ax_bev.set_ylabel("Forward Range (m)")
            ax_bev.set_ylim(-10, 10)
            ax_bev.set_xlim(-10, 10)

            # plot detections
            for det in det_final:
                ax_rd.plot(det["vel_mps"], det["range_m"], "ro", markersize=4)
                ax_bev.plot(det["x_m"], det["y_m"], "bo")

            # -----------------------------
            # 4th plot: beam-angle slice
            # strongest kept detection in frame
            # -----------------------------
            ax_beam.cla()
            ax_beam.set_title("Beam Power Slice")
            ax_beam.set_xlabel("Beam Angle (deg)")
            ax_beam.set_ylabel("Power (dB)")
            ax_beam.grid(True)

            selected_det = None
            if len(det_final) > 0:
                selected_det = max(det_final, key=lambda d: d["power"])

                r_idx_sel = selected_det["r_idx"]
                d_idx_sel = selected_det["d_idx"]

                beam_slice = beam_rd_cube[r_idx_sel, d_idx_sel, :]
                beam_slice_db = 10.0 * np.log10(beam_slice + EPSILON)

                ax_beam.plot(beam_angles_deg, beam_slice_db, marker="o")

                ax_beam.axvline(
                    selected_det["coarse_beam_deg"],
                )
                ax_beam.axvline(
                    selected_det["beam_interp_deg"],
                    color="purple",
                    linestyle="--"
                )

                ax_beam.set_title(
                    f"Beam Slice (R={selected_det['range_m']:.2f} m | "
                    f"V={selected_det['vel_mps']:.2f} m/s)"
                )

            processing_time_ms = (time.time() - start_time) * 1000.0
            f.write(
                f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | "
                f"Latency: {processing_time_ms:.2f} ms | "
                f"NumBeams: {len(beam_angles_deg)} | "
                f"RawDetections: {len(det_candidates)} | "
                f"FinalDetections: {len(det_final)}\n"
            )

            for det in det_final:
                f.write(
                    f"  DETECT | "
                    f"R:{det['range_m']:6.3f} m | "
                    f"V:{det['vel_mps']:6.3f} m/s | "
                    f"BeamCoarse:{det['coarse_beam_deg']:7.2f} deg | "
                    f"BeamInterp:{det['beam_interp_deg']:7.2f} deg | "
                    f"AoA:{det['angle_deg']:7.2f} deg | "
                    f"X:{det['x_m']:6.3f} | "
                    f"Y:{det['y_m']:6.3f} | "
                    f"P:{det['power']:.3e}\n"
                )

            if selected_det is not None:
                f.write(
                    f"  DEBUG_SELECTED | "
                    f"Ridx:{selected_det['r_idx']} | "
                    f"Didx:{selected_det['d_idx']} | "
                    f"BeamIdx:{selected_det['beam_idx']} | "
                    f"BeamCoarse:{selected_det['coarse_beam_deg']:.2f} | "
                    f"BeamInterp:{selected_det['beam_interp_deg']:.2f}\n"
                )

            plt.tight_layout()
            plt.draw()
            plt.pause(0.05)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    run_radar_simulation()