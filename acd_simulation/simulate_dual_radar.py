import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def polar_to_cartesian(range_m, azimuth_rad, radar_pos):
    """Convert polar coordinates to Cartesian coordinates.
    
    Args:
        range_m: Range in meters
        azimuth_rad: Azimuth angle in radians
        radar_pos: Radar position as [x, y]
    
    Returns:
        Cartesian position [x, y] relative to global origin
    """
    x = radar_pos[0] + range_m * np.cos(azimuth_rad)
    y = radar_pos[1] + range_m * np.sin(azimuth_rad)
    return np.array([x, y])


def sensor_fusion(pos1, pos2, cov1=None, cov2=None):
    """Fuse two radar position estimates using weighted average.
    
    Args:
        pos1: Position estimate from radar 1 [x, y]
        pos2: Position estimate from radar 2 [x, y]
        cov1: Covariance/uncertainty from radar 1 (default: equal weight)
        cov2: Covariance/uncertainty from radar 2 (default: equal weight)
    
    Returns:
        Fused position [x, y]
    """
    if cov1 is None and cov2 is None:
        # Equal weight fusion
        return 0.5 * pos1 + 0.5 * pos2
    else:
        # Weighted by inverse covariance
        w1 = 1.0 / (cov1 + 1e-12) if cov1 is not None else 1.0
        w2 = 1.0 / (cov2 + 1e-12) if cov2 is not None else 1.0
        return (w1 * pos1 + w2 * pos2) / (w1 + w2)


def run_dual_radar_simulation():
    """Simulate two radars tracking one moving target with range-doppler visualization."""
    
    # --- PARAMETERS ---
    noADC = 256
    noRx = 2
    noChirps = 32
    f0 = 60e9               
    fs = 12.5e6       
    c = 299792458.0
    wavelength = c / f0

    # Chirp and radar parameters
    chirpTime = noADC / fs
    BW = 4e9
    slp = BW / chirpTime
    PRI = 300e-6
    antennaSpacing = wavelength / 2.0
    rcs = 1.0
    Pt = 0.0158
    Gt = 10.0
    Gr = 10.0

    # FFT sizes
    FFTRNGSIZE = int(2 ** np.ceil(np.log2(noADC)))
    FFTDOPSIZE = int(2 ** np.ceil(np.log2(noChirps)))

    # Target motion
    startPos = np.array([5.0, 8.0])
    endPos = np.array([2.0, 1.0])
    duration = 10.0
    target_velocity = (endPos - startPos) / duration
    UPDATE_RATE_SEC = 0.1

    # Two radars positioned at different locations
    radarPos1 = np.array([-2.0, 0.0])
    radarPos2 = np.array([2.0, 0.0])

    # Axes
    f_bins = (np.arange(FFTRNGSIZE) / FFTRNGSIZE) * fs
    rangeAxis = (c * f_bins) / (2.0 * slp)
    fAx = np.fft.fftshift(np.fft.fftfreq(FFTDOPSIZE, d=PRI))
    velocityAxis = fAx * (wavelength / 2.0)

    # --- STORAGE FOR TIME SERIES ---
    timestamps = []
    target_positions = []
    radar1_ranges = []
    radar1_velocities = []
    radar1_aoas = []
    radar1_cartesian = []  # Cartesian estimates from radar 1
    radar2_ranges = []
    radar2_velocities = []
    radar2_aoas = []
    radar2_cartesian = []  # Cartesian estimates from radar 2
    fused_cartesian = []   # Fused Cartesian estimates
    position_errors = []   # Error between true and estimated position

    # --- PLOTTING SETUP ---
    fig = plt.figure(figsize=(22, 14))

    # Radar 1 Range-Doppler
    ax1 = fig.add_subplot(3, 3, 1)
    RD_dB_init = np.full((FFTRNGSIZE, FFTDOPSIZE), -100.0)
    im1 = ax1.imshow(
        RD_dB_init,
        aspect="auto",
        origin="lower",
        extent=[velocityAxis[0], velocityAxis[-1], rangeAxis[0], rangeAxis[-1]],
        cmap="jet",
        vmin=-100.0,
        vmax=0.0,
    )
    ax1.set_xlabel("Radial velocity (m/s)")
    ax1.set_ylabel("Range (m)")
    ax1.set_title("Radar 1: Range-Doppler")
    plt.colorbar(im1, ax=ax1, label="Power (dB)")

    # Radar 2 Range-Doppler
    ax2 = fig.add_subplot(3, 3, 2)
    im2 = ax2.imshow(
        RD_dB_init,
        aspect="auto",
        origin="lower",
        extent=[velocityAxis[0], velocityAxis[-1], rangeAxis[0], rangeAxis[-1]],
        cmap="jet",
        vmin=-100.0,
        vmax=0.0,
    )
    ax2.set_xlabel("Radial velocity (m/s)")
    ax2.set_ylabel("Range (m)")
    ax2.set_title("Radar 2: Range-Doppler")
    plt.colorbar(im2, ax=ax2, label="Power (dB)")

    # Top-down geometry with Cartesian estimates
    ax3 = fig.add_subplot(3, 3, 3)
    ax3.set_aspect("equal")
    ax3.grid(True)
    axis_limit = 10.0
    all_positions = np.vstack([radarPos1, radarPos2, startPos, endPos])
    ax3.set_xlim(all_positions[:, 0].min() - 2, all_positions[:, 0].max() + 2)
    ax3.set_ylim(all_positions[:, 1].min() - 2, all_positions[:, 1].max() + 2)
    ax3.plot(radarPos1[0], radarPos1[1], "r*", markersize=15, label="Radar 1")
    ax3.plot(radarPos2[0], radarPos2[1], "b*", markersize=15, label="Radar 2")
    target_point, = ax3.plot(startPos[0], startPos[1], "go", markersize=10, label="True Target")
    target_trajectory, = ax3.plot([startPos[0]], [startPos[1]], "g--", alpha=0.5, linewidth=1.5, label="True Trajectory")
    est_point1, = ax3.plot(startPos[0], startPos[1], "r^", markersize=8, label="Radar 1 Est")
    est_point2, = ax3.plot(startPos[0], startPos[1], "b^", markersize=8, label="Radar 2 Est")
    fused_point, = ax3.plot(startPos[0], startPos[1], "m*", markersize=12, label="Fused Est")
    aoa_line1, = ax3.plot([0, 0], [0, 0], "r--", linewidth=1.5, alpha=0.7)
    aoa_line2, = ax3.plot([0, 0], [0, 0], "b--", linewidth=1.5, alpha=0.7)
    ax3.set_title(f"Top-down view (Time: 0.0s)")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.legend(loc='upper right', fontsize=7)

    # Range vs Time
    ax4 = fig.add_subplot(3, 3, 4)
    line_r1, = ax4.plot([], [], 'r-', linewidth=2, label="Radar 1")
    line_r2, = ax4.plot([], [], 'b-', linewidth=2, label="Radar 2")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Range (m)")
    ax4.set_title("Range vs Time")
    ax4.set_xlim(0, duration)
    ax4.set_ylim(0, 15)
    ax4.grid(True)
    ax4.legend()

    # Velocity vs Time
    ax5 = fig.add_subplot(3, 3, 5)
    line_v1, = ax5.plot([], [], 'r-', linewidth=2, label="Radar 1")
    line_v2, = ax5.plot([], [], 'b-', linewidth=2, label="Radar 2")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Radial Velocity (m/s)")
    ax5.set_title("Radial Velocity vs Time")
    ax5.set_xlim(0, duration)
    ax5.set_ylim(-3, 3)
    ax5.grid(True)
    ax5.legend()
    ax5.axhline(0, color='k', linestyle='--', alpha=0.3)

    # AoA vs Time
    ax6 = fig.add_subplot(3, 3, 6)
    line_a1, = ax6.plot([], [], 'r-', linewidth=2, label="Radar 1")
    line_a2, = ax6.plot([], [], 'b-', linewidth=2, label="Radar 2")
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Azimuth (degrees)")
    ax6.set_title("Azimuth of Arrival vs Time")
    ax6.set_xlim(0, duration)
    ax6.set_ylim(-180, 180)
    ax6.grid(True)
    ax6.legend()

    # Cartesian X vs Time
    ax7 = fig.add_subplot(3, 3, 7)
    line_x_true, = ax7.plot([], [], 'g-', linewidth=2.5, label="True X")
    line_x_r1, = ax7.plot([], [], 'r--', linewidth=1.5, label="Radar 1 Est")
    line_x_r2, = ax7.plot([], [], 'b--', linewidth=1.5, label="Radar 2 Est")
    line_x_fused, = ax7.plot([], [], 'm-', linewidth=2, label="Fused Est")
    ax7.set_xlabel("Time (s)")
    ax7.set_ylabel("X Position (m)")
    ax7.set_title("X Coordinate vs Time")
    ax7.set_xlim(0, duration)
    ax7.grid(True)
    ax7.legend(fontsize=8)

    # Cartesian Y vs Time
    ax8 = fig.add_subplot(3, 3, 8)
    line_y_true, = ax8.plot([], [], 'g-', linewidth=2.5, label="True Y")
    line_y_r1, = ax8.plot([], [], 'r--', linewidth=1.5, label="Radar 1 Est")
    line_y_r2, = ax8.plot([], [], 'b--', linewidth=1.5, label="Radar 2 Est")
    line_y_fused, = ax8.plot([], [], 'm-', linewidth=2, label="Fused Est")
    ax8.set_xlabel("Time (s)")
    ax8.set_ylabel("Y Position (m)")
    ax8.set_title("Y Coordinate vs Time")
    ax8.set_xlim(0, duration)
    ax8.grid(True)
    ax8.legend(fontsize=8)

    # Position Error vs Time
    ax9 = fig.add_subplot(3, 3, 9)
    line_err_r1, = ax9.plot([], [], 'r-', linewidth=2, label="Radar 1 Error")
    line_err_r2, = ax9.plot([], [], 'b-', linewidth=2, label="Radar 2 Error")
    line_err_fused, = ax9.plot([], [], 'm-', linewidth=2, label="Fused Error")
    ax9.set_xlabel("Time (s)")
    ax9.set_ylabel("Position Error (m)")
    ax9.set_title("Estimation Error vs Time")
    ax9.set_xlim(0, duration)
    ax9.grid(True)
    ax9.legend(fontsize=8)

    plt.tight_layout()
    plt.ion()

    eps = 1e-12
    t_adc = np.arange(noADC) / fs

    # --- SIMULATION LOOP ---
    print("Starting dual radar simulation...")
    print(f"Radar 1 Position: {radarPos1}")
    print(f"Radar 2 Position: {radarPos2}")
    print(f"Target Motion: {startPos} -> {endPos}")
    print("")

    for t_idx, t_now in enumerate(np.arange(0.0, duration + UPDATE_RATE_SEC, UPDATE_RATE_SEC)):
        if t_now > duration:
            break

        # Target position at current frame
        objPos = startPos + target_velocity * t_now
        current_x, current_y = objPos[0], objPos[1]

        # Update target
        target_point.set_data([current_x], [current_y])
        target_trajectory.set_data(
            np.append([p[0] for p in target_positions], current_x),
            np.append([p[1] for p in target_positions], current_y)
        )

        # Store position
        timestamps.append(t_now)
        target_positions.append(objPos.copy())

        # ===== RADAR 1 PROCESSING =====
        R1 = np.linalg.norm(objPos - radarPos1)
        az1 = np.arctan2(objPos[1] - radarPos1[1], objPos[0] - radarPos1[0])
        radial_velocity1 = np.dot(target_velocity, (objPos - radarPos1) / (R1 + eps))
        fd_cpi1 = 2.0 * radial_velocity1 / wavelength
        
        tau1 = 2.0 * R1 / c
        fb_term1 = slp * tau1
        Pr_m1 = (Pt * Gt * Gr * (wavelength ** 2) * rcs) / (((4 * np.pi) ** 3) * (R1 ** 4) + eps)
        A_m1 = np.sqrt(np.abs(Pr_m1))
        const_phase_m1 = np.exp(-1j * 2 * np.pi * (f0 * tau1 + 0.5 * slp * tau1 ** 2))

        data1 = np.zeros((noADC, noRx, noChirps), dtype=np.complex128)
        for m in range(noChirps):
            doppler_factor1 = np.exp(1j * 2.0 * np.pi * fd_cpi1 * m * PRI)
            for rx in range(noRx):
                antenna_phase1 = np.exp(1j * 2.0 * np.pi * (antennaSpacing * rx * np.sin(az1)) / wavelength)
                beat1 = A_m1 * const_phase_m1 * antenna_phase1 * doppler_factor1 * np.exp(1j * 2 * np.pi * fb_term1 * t_adc)
                data1[:, rx, m] = beat1

        # Window + Range FFT
        hammingWindow = np.hamming(noADC).reshape(noADC, 1, 1)
        windowed1 = data1 * hammingWindow
        data1_rx1, data1_rx2 = windowed1[:, 0, :], windowed1[:, 1, :]
        range_fft1_rx1 = np.fft.fft(data1_rx1, FFTRNGSIZE, axis=0)
        range_fft1_rx2 = np.fft.fft(data1_rx2, FFTRNGSIZE, axis=0)

        # Remove DC and apply slow-time window
        range_fft1_rx1 = range_fft1_rx1 - np.mean(range_fft1_rx1, axis=1, keepdims=True)
        range_fft1_rx2 = range_fft1_rx2 - np.mean(range_fft1_rx2, axis=1, keepdims=True)
        slow_win = np.hamming(noChirps)
        range_fft1_rx1 = range_fft1_rx1 * slow_win[np.newaxis, :]
        range_fft1_rx2 = range_fft1_rx2 * slow_win[np.newaxis, :]

        rd1_rx1 = np.fft.fftshift(np.fft.fft(range_fft1_rx1, FFTDOPSIZE, axis=1), axes=1)
        RD_dB1 = 20.0 * np.log10(np.abs(rd1_rx1) + eps)

        # Extract measurements from Radar 1
        range_idx1, dop_idx1 = np.unravel_index(np.argmax(RD_dB1), RD_dB1.shape)
        S1_rx1, S1_rx2 = rd1_rx1[range_idx1, dop_idx1], np.fft.fftshift(np.fft.fft(range_fft1_rx2, FFTDOPSIZE, axis=1), axes=1)[range_idx1, dop_idx1]
        delta_phi1 = np.angle(S1_rx2 / (S1_rx1 + eps))
        sin_theta1 = (delta_phi1 * wavelength) / (2.0 * np.pi * antennaSpacing)
        sin_theta1 = np.clip(sin_theta1, -1.0, 1.0)
        measured_aoa1_rad = np.arcsin(sin_theta1)
        measured_aoa1_deg = np.rad2deg(measured_aoa1_rad)
        measured_range1 = rangeAxis[range_idx1]
        measured_velocity1 = velocityAxis[dop_idx1]

        # ===== RADAR 2 PROCESSING =====
        R2 = np.linalg.norm(objPos - radarPos2)
        az2 = np.arctan2(objPos[1] - radarPos2[1], objPos[0] - radarPos2[0])
        radial_velocity2 = np.dot(target_velocity, (objPos - radarPos2) / (R2 + eps))
        fd_cpi2 = 2.0 * radial_velocity2 / wavelength
        
        tau2 = 2.0 * R2 / c
        fb_term2 = slp * tau2
        Pr_m2 = (Pt * Gt * Gr * (wavelength ** 2) * rcs) / (((4 * np.pi) ** 3) * (R2 ** 4) + eps)
        A_m2 = np.sqrt(np.abs(Pr_m2))
        const_phase_m2 = np.exp(-1j * 2 * np.pi * (f0 * tau2 + 0.5 * slp * tau2 ** 2))

        data2 = np.zeros((noADC, noRx, noChirps), dtype=np.complex128)
        for m in range(noChirps):
            doppler_factor2 = np.exp(1j * 2.0 * np.pi * fd_cpi2 * m * PRI)
            for rx in range(noRx):
                antenna_phase2 = np.exp(1j * 2.0 * np.pi * (antennaSpacing * rx * np.sin(az2)) / wavelength)
                beat2 = A_m2 * const_phase_m2 * antenna_phase2 * doppler_factor2 * np.exp(1j * 2 * np.pi * fb_term2 * t_adc)
                data2[:, rx, m] = beat2

        # Window + Range FFT
        windowed2 = data2 * hammingWindow
        data2_rx1, data2_rx2 = windowed2[:, 0, :], windowed2[:, 1, :]
        range_fft2_rx1 = np.fft.fft(data2_rx1, FFTRNGSIZE, axis=0)
        range_fft2_rx2 = np.fft.fft(data2_rx2, FFTRNGSIZE, axis=0)

        # Remove DC and apply slow-time window
        range_fft2_rx1 = range_fft2_rx1 - np.mean(range_fft2_rx1, axis=1, keepdims=True)
        range_fft2_rx2 = range_fft2_rx2 - np.mean(range_fft2_rx2, axis=1, keepdims=True)
        range_fft2_rx1 = range_fft2_rx1 * slow_win[np.newaxis, :]
        range_fft2_rx2 = range_fft2_rx2 * slow_win[np.newaxis, :]

        rd2_rx1 = np.fft.fftshift(np.fft.fft(range_fft2_rx1, FFTDOPSIZE, axis=1), axes=1)
        RD_dB2 = 20.0 * np.log10(np.abs(rd2_rx1) + eps)

        # Extract measurements from Radar 2
        range_idx2, dop_idx2 = np.unravel_index(np.argmax(RD_dB2), RD_dB2.shape)
        S2_rx1, S2_rx2 = rd2_rx1[range_idx2, dop_idx2], np.fft.fftshift(np.fft.fft(range_fft2_rx2, FFTDOPSIZE, axis=1), axes=1)[range_idx2, dop_idx2]
        delta_phi2 = np.angle(S2_rx2 / (S2_rx1 + eps))
        sin_theta2 = (delta_phi2 * wavelength) / (2.0 * np.pi * antennaSpacing)
        sin_theta2 = np.clip(sin_theta2, -1.0, 1.0)
        measured_aoa2_rad = np.arcsin(sin_theta2)
        measured_aoa2_deg = np.rad2deg(measured_aoa2_rad)
        measured_range2 = rangeAxis[range_idx2]
        measured_velocity2 = velocityAxis[dop_idx2]

        # Store time series data
        radar1_ranges.append(measured_range1)
        radar1_velocities.append(measured_velocity1)
        radar1_aoas.append(measured_aoa1_deg)
        radar2_ranges.append(measured_range2)
        radar2_velocities.append(measured_velocity2)
        radar2_aoas.append(measured_aoa2_deg)

        # ===== CARTESIAN COORDINATE CONVERSION =====
        # Convert polar measurements to Cartesian coordinates
        est_pos1 = polar_to_cartesian(measured_range1, measured_aoa1_rad, radarPos1)
        est_pos2 = polar_to_cartesian(measured_range2, measured_aoa2_rad, radarPos2)
        
        # Estimate uncertainty/covariance based on signal strength
        # Higher signal power -> lower uncertainty
        snr1 = np.max(RD_dB1)
        snr2 = np.max(RD_dB2)
        cov1 = 1.0 / (snr1 + 1e-12) if snr1 > -100 else 1.0
        cov2 = 1.0 / (snr2 + 1e-12) if snr2 > -100 else 1.0
        
        # Sensor fusion: combine both estimates
        est_pos_fused = sensor_fusion(est_pos1, est_pos2, cov1, cov2)
        
        # Store Cartesian estimates
        radar1_cartesian.append(est_pos1.copy())
        radar2_cartesian.append(est_pos2.copy())
        fused_cartesian.append(est_pos_fused.copy())
        
        # Calculate position errors
        error1 = np.linalg.norm(est_pos1 - objPos)
        error2 = np.linalg.norm(est_pos2 - objPos)
        error_fused = np.linalg.norm(est_pos_fused - objPos)
        position_errors.append([error1, error2, error_fused])

        # --- PLOTTING UPDATE ---
        im1.set_data(RD_dB1)
        im1.set_clim(vmin=np.max(RD_dB1) - 60, vmax=np.max(RD_dB1))

        im2.set_data(RD_dB2)
        im2.set_clim(vmin=np.max(RD_dB2) - 60, vmax=np.max(RD_dB2))

        # AoA lines
        L1 = R1 * 1.5
        aoa_line1.set_data([radarPos1[0], radarPos1[0] + L1 * np.cos(measured_aoa1_rad)],
                           [radarPos1[1], radarPos1[1] + L1 * np.sin(measured_aoa1_rad)])
        
        L2 = R2 * 1.5
        aoa_line2.set_data([radarPos2[0], radarPos2[0] + L2 * np.cos(measured_aoa2_rad)],
                           [radarPos2[1], radarPos2[1] + L2 * np.sin(measured_aoa2_rad)])

        # Update Cartesian position points
        est_point1.set_data([est_pos1[0]], [est_pos1[1]])
        est_point2.set_data([est_pos2[0]], [est_pos2[1]])
        fused_point.set_data([est_pos_fused[0]], [est_pos_fused[1]])

        ax3.set_title(f"Top-down view (Time: {t_now:.1f}s)")

        # Update time series plots
        line_r1.set_data(timestamps, radar1_ranges)
        line_r2.set_data(timestamps, radar2_ranges)
        ax4.set_ylim(0, max(max(radar1_ranges + [1]), max(radar2_ranges + [1])) * 1.1)

        line_v1.set_data(timestamps, radar1_velocities)
        line_v2.set_data(timestamps, radar2_velocities)
        if len(radar1_velocities) > 0 and len(radar2_velocities) > 0:
            vel_max = max(abs(min(radar1_velocities + radar2_velocities + [0])), 
                         abs(max(radar1_velocities + radar2_velocities + [0]))) * 1.2
            ax5.set_ylim(-vel_max, vel_max)

        line_a1.set_data(timestamps, radar1_aoas)
        line_a2.set_data(timestamps, radar2_aoas)

        # Cartesian coordinate plots
        true_x = [p[0] for p in target_positions]
        true_y = [p[1] for p in target_positions]
        est_x1 = [p[0] for p in radar1_cartesian]
        est_x2 = [p[0] for p in radar2_cartesian]
        est_x_fused = [p[0] for p in fused_cartesian]
        est_y1 = [p[1] for p in radar1_cartesian]
        est_y2 = [p[1] for p in radar2_cartesian]
        est_y_fused = [p[1] for p in fused_cartesian]

        line_x_true.set_data(timestamps, true_x)
        line_x_r1.set_data(timestamps, est_x1)
        line_x_r2.set_data(timestamps, est_x2)
        line_x_fused.set_data(timestamps, est_x_fused)
        ax7.relim()
        ax7.autoscale_view(scalex=False, scaley=True)

        line_y_true.set_data(timestamps, true_y)
        line_y_r1.set_data(timestamps, est_y1)
        line_y_r2.set_data(timestamps, est_y2)
        line_y_fused.set_data(timestamps, est_y_fused)
        ax8.relim()
        ax8.autoscale_view(scalex=False, scaley=True)

        # Position error plot
        errors = np.array(position_errors)
        line_err_r1.set_data(timestamps, errors[:, 0])
        line_err_r2.set_data(timestamps, errors[:, 1])
        line_err_fused.set_data(timestamps, errors[:, 2])
        ax9.relim()
        ax9.autoscale_view(scalex=False, scaley=True)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.01)

        # Print status
        if t_idx % 10 == 0:
            print(f"\n--- TIME: {t_now:.1f}s ---")
            print(f"TRUE Position: ({current_x:.3f}, {current_y:.3f}) m")
            print(f"Radar 1 | Range: {measured_range1:.3f} m | Vel: {measured_velocity1:.3f} m/s | AoA: {measured_aoa1_deg:.2f}°")
            print(f"        | Est Pos: ({est_pos1[0]:.3f}, {est_pos1[1]:.3f}) m | Error: {error1:.3f} m")
            print(f"Radar 2 | Range: {measured_range2:.3f} m | Vel: {measured_velocity2:.3f} m/s | AoA: {measured_aoa2_deg:.2f}°")
            print(f"        | Est Pos: ({est_pos2[0]:.3f}, {est_pos2[1]:.3f}) m | Error: {error2:.3f} m")
            print(f"FUSED   | Est Pos: ({est_pos_fused[0]:.3f}, {est_pos_fused[1]:.3f}) m | Error: {error_fused:.3f} m")

    plt.ioff()
    plt.tight_layout()
    plt.show()

    # Print final statistics
    errors_array = np.array(position_errors)
    print("\n" + "="*70)
    print("FINAL STATISTICS")
    print("="*70)
    print(f"\nPosition Estimation Errors (in meters):")
    print(f"Radar 1 - Mean: {np.mean(errors_array[:, 0]):.4f}m, Std: {np.std(errors_array[:, 0]):.4f}m, Max: {np.max(errors_array[:, 0]):.4f}m")
    print(f"Radar 2 - Mean: {np.mean(errors_array[:, 1]):.4f}m, Std: {np.std(errors_array[:, 1]):.4f}m, Max: {np.max(errors_array[:, 1]):.4f}m")
    print(f"Fused   - Mean: {np.mean(errors_array[:, 2]):.4f}m, Std: {np.std(errors_array[:, 2]):.4f}m, Max: {np.max(errors_array[:, 2]):.4f}m")
    print("\nFusion Improvement:")
    improvement_vs_r1 = (np.mean(errors_array[:, 0]) - np.mean(errors_array[:, 2])) / np.mean(errors_array[:, 0]) * 100
    improvement_vs_r2 = (np.mean(errors_array[:, 1]) - np.mean(errors_array[:, 2])) / np.mean(errors_array[:, 1]) * 100
    print(f"  vs Radar 1: {improvement_vs_r1:.2f}% better")
    print(f"  vs Radar 2: {improvement_vs_r2:.2f}% better")
    print("="*70 + "\n")

    print("Simulation complete!")
    return fig


if __name__ == "__main__":
    fig = run_dual_radar_simulation()
