import numpy as np
import matplotlib.pyplot as plt
import csv

def polar_to_cartesian(range_m, azimuth_rad, radar_pos, elevation_rad=0.0):
    """Convert polar/spherical coordinates to Cartesian coordinates.
    
    Args:
        range_m: Range in meters
        azimuth_rad: Azimuth angle in radians (angle from x-axis in x-y plane)
        radar_pos: Radar position as [x, y, z] or [x, y]
        elevation_rad: Elevation angle in radians (default 0 for 2D case)
    
    Returns:
        Cartesian position [x, y, z] relative to global origin
    """
    radar_pos = np.array(radar_pos)
    if len(radar_pos) == 2:
        radar_pos = np.append(radar_pos, 0.0)  # Add z=0 if 2D
    
    x = radar_pos[0] + range_m * np.cos(elevation_rad) * np.cos(azimuth_rad)
    y = radar_pos[1] + range_m * np.cos(elevation_rad) * np.sin(azimuth_rad)
    z = radar_pos[2] + range_m * np.sin(elevation_rad)
    
    return np.array([x, y, z])


def run_fmcw_radar_simulation():
    """
    Simulate FMCW radar with antenna array to detect a moving point target
    and calculate Cartesian coordinates from range and azimuth measurements.
    """
    # --- RADAR PARAMETERS ---
    noADC = 256              # Number of ADC samples per chirp
    noRx = 2                 # Number of receive antennas (linear array)
    noChirps = 64           # Number of chirps in a CPI
    f0 = 60e9                # Carrier frequency (Hz) - 60 GHz
    fs = 12.5e6              # ADC sampling rate (Hz)
    c = 299792458.0          # Speed of light (m/s)
    wavelength = c / f0      # Wavelength (m)
    
    # Chirp parameters
    chirpTime = noADC / fs   # Chirp duration (s)
    BW = 4e9                 # Bandwidth (Hz)
    slp = BW / chirpTime     # Chirp slope (Hz/s)
    PRI = 300e-6             # Pulse repetition interval (s)
    PRF = 1.0 / PRI          # Pulse repetition frequency (Hz)
    
    # Antenna array parameters
    antennaSpacing = wavelength / 2.0  # Half-wavelength spacing
    
    # Radar hardware parameters
    rcs = 1.0                # Radar cross section (m^2)
    Pt = 0.0158              # Transmit power (W) - 12 dBm
    Gt = 10.0                # Transmit antenna gain
    Gr = 10.0                # Receive antenna gain
    
    # FFT sizes
    FFTRNGSIZE = int(2 ** np.ceil(np.log2(noADC)))
    FFTDOPSIZE = int(2 ** np.ceil(np.log2(noChirps)))
    
    # --- TARGET TRAJECTORY PARAMETERS ---
    radarPos = np.array([0.0, 0.0])  # Radar position [x, y] in meters
    startPos = np.array([3.0, 6.0])  # Initial target position [x, y]
    duration = 12.0                   # Simulation duration (s)
    UPDATE_RATE_SEC = 0.2            # Update rate for visualization (s)
    
    # Initial velocity vector (m/s)
    initial_target_velocity = np.array([-0.3, -0.6])
    
    # Constant acceleration vector (m/s^2)
    target_acceleration = np.array([0.05, -0.07])
    
    # --- AXES DEFINITIONS ---
    # Range axis from beat frequency bin mapping
    f_bins = (np.arange(FFTRNGSIZE) / FFTRNGSIZE) * fs
    rangeAxis = (c * f_bins) / (2.0 * slp)
    
    # Velocity axis from Doppler frequency
    fAx = np.fft.fftshift(np.fft.fftfreq(FFTDOPSIZE, d=PRI))
    velocityAxis = fAx * (wavelength / 2.0)
    
    # --- PLOTTING SETUP ---
    plt.ion()
    fig = plt.figure(figsize=(18, 10))
    
    # 1) Range profile
    ax1 = fig.add_subplot(2, 3, 1)
    line_range_profile, = ax1.plot(rangeAxis, np.zeros(FFTRNGSIZE), linewidth=2)
    ax1.set_xlabel("Range (m)")
    ax1.set_ylabel("Power (dB)")
    ax1.set_title("Range Profile (First Chirp)")
    ax1.set_ylim(-100, 0)
    ax1.grid(True)
    
    # 2) Range-Doppler map
    ax2 = fig.add_subplot(2, 3, 2)
    RD_dB_init = np.full((FFTRNGSIZE, FFTDOPSIZE), -100.0)
    im = ax2.imshow(
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
    ax2.set_title("Range-Doppler Map")
    colorBar = fig.colorbar(im, ax=ax2, orientation='vertical')
    colorBar.set_label("Power (dB)")
    
    # 3) Top-down view with true and estimated positions
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.set_aspect("equal")
    ax3.grid(True)
    
    # Calculate dynamic axis limits based on start position and trajectory
    # Estimate trajectory bounds (consider initial position + motion over duration)
    max_x = max(radarPos[0], startPos[0], startPos[0] + initial_target_velocity[0] * duration + 0.5 * target_acceleration[0] * duration**2)
    min_x = min(radarPos[0], startPos[0], startPos[0] + initial_target_velocity[0] * duration + 0.5 * target_acceleration[0] * duration**2)
    max_y = max(radarPos[1], startPos[1], startPos[1] + initial_target_velocity[1] * duration + 0.5 * target_acceleration[1] * duration**2)
    min_y = min(radarPos[1], startPos[1], startPos[1] + initial_target_velocity[1] * duration + 0.5 * target_acceleration[1] * duration**2)
    
    # Add padding
    padding = 5.0
    ax3.set_xlim(min_x - padding, max_x + padding)
    ax3.set_ylim(min_y - padding, max_y + padding)
    ax3.plot(radarPos[0], radarPos[1], "r*", markersize=15, label="Radar")
    
    # Plot elements
    true_trajectory_line, = ax3.plot([], [], "g-", linewidth=2, alpha=0.5, label="True Trajectory")
    est_trajectory_line, = ax3.plot([], [], "b--", linewidth=2, alpha=0.7, label="Estimated Trajectory")
    true_target_point, = ax3.plot([], [], "go", markersize=12, label="True Position")
    est_target_point, = ax3.plot([], [], "b^", markersize=12, label="Estimated Position")
    aoa_line, = ax3.plot([], [], "m--", linewidth=1.5, alpha=0.6, label="AoA Line")
    error_line, = ax3.plot([], [], "r-", linewidth=1, alpha=0.4, label="Position Error")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("Top-down View")
    ax3.legend(loc="upper right", fontsize=8)
    
    # 4) Position error over time
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position Error (m)")
    ax4.set_title("Position Estimation Error")
    ax4.grid(True)
    error_line_plot, = ax4.plot([], [], "r-", linewidth=2)
    
    # 5) Range and azimuth measurements
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Range (m)")
    ax5.set_title("Measured Range")
    ax5.grid(True)
    range_line, = ax5.plot([], [], "b-", linewidth=2, label="Measured")
    range_true_line, = ax5.plot([], [], "g--", linewidth=2, label="True")
    ax5.legend()
    
    # 6) Azimuth measurements
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Azimuth (degrees)")
    ax6.set_title("Measured Azimuth (AoA)")
    ax6.grid(True)
    azimuth_line, = ax6.plot([], [], "b-", linewidth=2, label="Measured")
    azimuth_true_line, = ax6.plot([], [], "g--", linewidth=2, label="True")
    ax6.legend()
    
    plt.tight_layout()
    
    # --- SIMULATION VARIABLES ---
    eps = 1e-12
    t_adc = np.arange(noADC) / fs  # Time samples within a chirp
    
    # Storage for trajectories and measurements
    time_history = []
    true_positions = []
    est_positions = []
    position_errors = []
    measured_ranges = []
    true_ranges = []
    measured_azimuths = []
    true_azimuths = []
    
    # --- SIMULATION LOOP ---
    print(f"\n=== SIMULATION STARTING ===")
    print(f"Start Position: ({startPos[0]:.3f}, {startPos[1]:.3f}) m")
    print(f"Initial Velocity: ({initial_target_velocity[0]:.3f}, {initial_target_velocity[1]:.3f}) m/s")
    print(f"Acceleration: ({target_acceleration[0]:.3f}, {target_acceleration[1]:.3f}) m/s²")
    print(f"Duration: {duration:.1f} s\n")
    
    for t_now in np.arange(0.0, duration + UPDATE_RATE_SEC, UPDATE_RATE_SEC):
        if t_now > duration:
            break
        
        # --- TARGET POSITION AND VELOCITY ---
        # Position with acceleration: r(t) = r_0 + v_0*t + 0.5*a*t^2
        objPos = startPos + initial_target_velocity * t_now + 0.5 * target_acceleration * (t_now ** 2)
        current_x, current_y = objPos[0], objPos[1]
        
        # Instantaneous velocity: v(t) = v_0 + a*t
        current_target_velocity = initial_target_velocity + target_acceleration * t_now
        
        # Calculate true range and azimuth
        R_true = np.linalg.norm(objPos - radarPos)
        az_true = np.arctan2(objPos[1] - radarPos[1], objPos[0] - radarPos[0])
        
        # Calculate radial velocity (component of instantaneous velocity along line-of-sight)
        radial_velocity = np.dot(current_target_velocity, (objPos - radarPos) / (R_true + eps))
        fd = 2.0 * radial_velocity / wavelength  # Doppler frequency (Hz)
        
        # --- RADAR SIGNAL GENERATION ---
        data = np.zeros((noADC, noRx, noChirps), dtype=np.complex128)
        
        # Signal parameters
        tau = 2.0 * R_true / c  # Round-trip time delay
        fb = slp * tau           # Beat frequency
        
        # Received power (radar equation)
        Pr = (Pt * Gt * Gr * (wavelength**2) * rcs) / (((4*np.pi)**3) * (R_true**4) + eps)
        A = np.sqrt(np.abs(Pr))  # Signal amplitude
        
        # Constant phase term
        const_phase = np.exp(-1j * 2*np.pi * (f0*tau + 0.5 * slp * tau**2))
        
        # Generate signal for each chirp and antenna
        for m in range(noChirps):
            # Doppler phase (slow-time)
            doppler_factor = np.exp(1j * 2.0 * np.pi * fd * m * PRI)
            
            for rx in range(noRx):
                # Antenna phase (phase difference due to array geometry)
                antenna_phase = np.exp(1j * 2.0 * np.pi * (antennaSpacing * rx * np.sin(az_true)) / wavelength)
                
                # Beat signal (fast-time)
                beat = A * const_phase * antenna_phase * doppler_factor * np.exp(1j * 2*np.pi * fb * t_adc)
                data[:, rx, m] = beat
        
        # Add noise (optional - uncomment to add realistic noise)
        SNR_dB = 30  # Signal-to-Noise Ratio in dB
        signal_power = np.mean(np.abs(data) ** 2)
        noise_power = signal_power / (10 ** (SNR_dB / 10.0))
        noise_std = np.sqrt(noise_power / 2.0)
        noise = (noise_std * np.random.randn(*data.shape)) + 1j * (noise_std * np.random.randn(*data.shape))
        data = data + noise
        
        # --- SIGNAL PROCESSING ---
        # Apply Hamming window to fast-time
        hammingWindow = np.hamming(noADC).reshape(noADC, 1, 1)
        windowed = data * hammingWindow
        
        # Separate data from each antenna
        data_rx1 = windowed[:, 0, :]  # Shape: (noADC, noChirps)
        data_rx2 = windowed[:, 1, :]
        
        # Range FFT (along fast-time axis)
        range_fft_rx1 = np.fft.fft(data_rx1, FFTRNGSIZE, axis=0)
        range_fft_rx2 = np.fft.fft(data_rx2, FFTRNGSIZE, axis=0)
        
        # Remove DC/clutter (subtract mean across slow-time)
        range_fft_rx1 = range_fft_rx1 - np.mean(range_fft_rx1, axis=1, keepdims=True)
        range_fft_rx2 = range_fft_rx2 - np.mean(range_fft_rx2, axis=1, keepdims=True)
        
        # Apply window to slow-time
        slow_win = np.hamming(noChirps)
        range_fft_rx1 = range_fft_rx1 * slow_win[np.newaxis, :]
        range_fft_rx2 = range_fft_rx2 * slow_win[np.newaxis, :]
        
        # Doppler FFT (along slow-time axis)
        rd_rx1 = np.fft.fftshift(np.fft.fft(range_fft_rx1, FFTDOPSIZE, axis=1), axes=1)
        rd_rx2 = np.fft.fftshift(np.fft.fft(range_fft_rx2, FFTDOPSIZE, axis=1), axes=1)
        
        # Range-Doppler map (dB scale)
        RD_dB = 20.0 * np.log10(np.abs(rd_rx1) + eps)
        
        # --- TARGET DETECTION ---
        # Find peak in range-Doppler map
        max_val = np.max(RD_dB)
        range_idx, dop_idx = np.unravel_index(np.argmax(RD_dB), RD_dB.shape)
        
        # Apply detection threshold (filter out noise/weak signals)
        # Use absolute threshold: signals below -50 dB are likely noise
        detection_threshold_absolute = -50.0
        
        # Sub-bin peak interpolation for range
        if range_idx > 0 and range_idx < FFTRNGSIZE - 1 and max_val > detection_threshold_absolute:
            RD_mag_col = 10**(RD_dB[:, dop_idx] / 20.0)  # Convert to linear scale
            P0 = RD_mag_col[range_idx]
            P_left = RD_mag_col[range_idx - 1]
            P_right = RD_mag_col[range_idx + 1]
            
            numerator = P_left - P_right
            denominator = 2 * (P_left + P_right - 2 * P0)
            
            if abs(denominator) > 1e-10:
                delta_r = numerator / denominator
                # Limit interpolation to within one bin
                delta_r = np.clip(delta_r, -0.5, 0.5)
            else:
                delta_r = 0
            
            rangeBinSize = rangeAxis[1] - rangeAxis[0]
            measured_range = rangeAxis[range_idx] + (delta_r * rangeBinSize)
        else:
            measured_range = rangeAxis[range_idx]
        
        # Sub-bin peak interpolation for velocity (Doppler)
        if dop_idx > 0 and dop_idx < FFTDOPSIZE - 1 and max_val > detection_threshold_absolute:
            RD_mag_row = 10**(RD_dB[range_idx, :] / 20.0)  # Convert to linear scale
            P0 = RD_mag_row[dop_idx]
            P_left = RD_mag_row[dop_idx - 1]
            P_right = RD_mag_row[dop_idx + 1]
            
            numerator = P_left - P_right
            denominator = 2 * (P_left + P_right - 2 * P0)
            
            if abs(denominator) > 1e-10:
                delta_v = numerator / denominator
                # Limit interpolation to within one bin
                delta_v = np.clip(delta_v, -0.5, 0.5)
            else:
                delta_v = 0
            
            velocityBinSize = velocityAxis[1] - velocityAxis[0]
            measured_velocity = velocityAxis[dop_idx] + (delta_v * velocityBinSize)
        else:
            measured_velocity = velocityAxis[dop_idx]
        
        # --- ANGLE OF ARRIVAL (AoA) ESTIMATION ---
        # Use phase difference between antenna elements at peak location
        S1 = rd_rx1[range_idx, dop_idx]  # Complex value from antenna 1
        S2 = rd_rx2[range_idx, dop_idx]  # Complex value from antenna 2
        
        # Phase difference with more stable calculation
        # Use atan2 for better numerical stability
        phase1 = np.angle(S1)
        phase2 = np.angle(S2)
        delta_phi = phase2 - phase1
        
        # Unwrap phase difference to [-pi, pi] range
        delta_phi = np.arctan2(np.sin(delta_phi), np.cos(delta_phi))
        
        # Calculate AoA from phase difference
        # delta_phi = (2π * d * sin(theta)) / λ
        # => sin(theta) = (delta_phi * λ) / (2π * d)
        d = antennaSpacing
        sin_theta = (delta_phi * wavelength) / (2.0 * np.pi * d)
        sin_theta = np.clip(sin_theta, -1.0, 1.0)  # Prevent numerical errors
        measured_aoa_rad = np.arcsin(sin_theta)
        measured_aoa_deg = np.rad2deg(measured_aoa_rad)
        
        # --- CARTESIAN COORDINATE CALCULATION ---
        # Convert polar (range, azimuth) to Cartesian (x, y)
        est_pos = polar_to_cartesian(measured_range, measured_aoa_rad, radarPos)
        
        # Calculate position error
        position_error = np.linalg.norm(est_pos[:2] - objPos)
        
        # Store data
        time_history.append(t_now)
        true_positions.append(objPos.copy())
        est_positions.append(est_pos[:2].copy())
        position_errors.append(position_error)
        measured_ranges.append(measured_range)
        true_ranges.append(R_true)
        measured_azimuths.append(measured_aoa_deg)
        true_azimuths.append(np.rad2deg(az_true))
        
        # --- UPDATE PLOTS ---
        # Range profile
        range_profile_db = 20.0 * np.log10(np.abs(range_fft_rx1[:, 0]) + eps)
        line_range_profile.set_ydata(range_profile_db)
        ax1.relim()
        ax1.autoscale_view(scaley=True)
        
        # Range-Doppler map
        im.set_data(RD_dB)
        im.set_clim(vmin=np.max(RD_dB) - 60, vmax=np.max(RD_dB))
        
        # Mark peak on range-Doppler map
        ax2.plot(measured_velocity, measured_range, 'w*', markersize=10)
        
        # Top-down view
        if len(true_positions) > 1:
            true_traj = np.array(true_positions)
            est_traj = np.array(est_positions)
            true_trajectory_line.set_data(true_traj[:, 0], true_traj[:, 1])
            est_trajectory_line.set_data(est_traj[:, 0], est_traj[:, 1])
            
            # Error line connecting true and estimated
            error_line.set_data([objPos[0], est_pos[0]], [objPos[1], est_pos[1]])
        
        true_target_point.set_data([current_x], [current_y])
        est_target_point.set_data([est_pos[0]], [est_pos[1]])
        
        # AoA line
        L = R_true * 1.5
        aoa_line.set_data(
            [radarPos[0], radarPos[0] + L * np.cos(measured_aoa_rad)],
            [radarPos[1], radarPos[1] + L * np.sin(measured_aoa_rad)]
        )
        
        # Position error
        error_line_plot.set_data(time_history, position_errors)
        ax4.relim()
        ax4.autoscale_view()
        
        # Range measurements
        range_line.set_data(time_history, measured_ranges)
        range_true_line.set_data(time_history, true_ranges)
        ax5.relim()
        ax5.autoscale_view()
        
        # Azimuth measurements
        azimuth_line.set_data(time_history, measured_azimuths)
        azimuth_true_line.set_data(time_history, true_azimuths)
        ax6.relim()
        ax6.autoscale_view()
        
        # Print status
        print(f"\n--- Time: {t_now:.2f}s ---")
        print(f"True Position: ({current_x:.3f}, {current_y:.3f}) m")
        print(f"True Velocity: ({current_target_velocity[0]:.3f}, {current_target_velocity[1]:.3f}) m/s")
        print(f"Estimated Position: ({est_pos[0]:.3f}, {est_pos[1]:.3f}) m")
        print(f"Position Error: {position_error:.4f} m")
        print(f"True Range: {R_true:.3f} m | Measured Range: {measured_range:.3f} m")
        print(f"True Azimuth: {np.rad2deg(az_true):.2f}° | Measured AoA: {measured_aoa_deg:.2f}°")
        print(f"Radial Velocity: {radial_velocity:.3f} m/s | Measured: {measured_velocity:.3f} m/s")
        
        # Update display
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.05)
    
    plt.ioff()
    plt.show()
    
    # --- FINAL SUMMARY ---
    print("\n=== SIMULATION SUMMARY ===")
    print(f"Average position error: {np.mean(position_errors):.4f} m")
    print(f"Max position error: {np.max(position_errors):.4f} m")
    print(f"Min position error: {np.min(position_errors):.4f} m")
    print(f"RMS position error: {np.sqrt(np.mean(np.array(position_errors)**2)):.4f} m")
    
    return {
        'time': time_history,
        'true_positions': true_positions,
        'est_positions': est_positions,
        'position_errors': position_errors,
        'measured_ranges': measured_ranges,
        'true_ranges': true_ranges,
        'measured_azimuths': measured_azimuths,
        'true_azimuths': true_azimuths
    }

def export_csv(csv_filename):
    # Export detected positions to CSV
    with open(csv_filename, 'w', newline='') as csvfile:
        fieldnames = ['time_s', 'detected_x_m', 'detected_y_m', 'true_x_m', 'true_y_m', 'position_error_m']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for i in range(len(results['time'])):
            writer.writerow({
                'time_s': results['time'][i],
                'detected_x_m': results['est_positions'][i][0],
                'detected_y_m': results['est_positions'][i][1],
                'true_x_m': results['true_positions'][i][0],
                'true_y_m': results['true_positions'][i][1],
                'position_error_m': results['position_errors'][i]
            })
    
    print(f"\nDetected positions exported to: {csv_filename}")

if __name__ == "__main__":
    results = run_fmcw_radar_simulation()
    export_csv("detected_positions2.csv")
    

