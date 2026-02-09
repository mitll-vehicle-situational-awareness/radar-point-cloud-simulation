import numpy as np
import matplotlib.pyplot as plt

def polar_to_cartesian(range_m, azimuth_rad, radar_pos, elevation_rad = 0.0):
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

def run_multi_snr_simulation(snr_levels):
    # PARAMETERS
    num_adc_samples, num_antennas, num_chirps = 256, 4, 128
    f0, fs, c = 60e9, 12.5e6, 299792458.0
    wavelength = c / f0
    slp = 4e9 / (num_adc_samples / fs)
    PRI = 300e-6
    antenna_spacing = wavelength / 2.0
    FFTRNGSIZE = int(2**np.ceil(np.log2(num_adc_samples)))
    FFTDOPSIZE = int(2**np.ceil(np.log2(num_chirps)))
    
    # Trajectory
    radar_position = np.array([0.0, 0.0])
    start_position = np.array([3.0, 6.0])
    initial_target_velocity, target_acceleration = np.array([-0.3, -0.6]), np.array([0.05, -0.07])
    duration, frame_rate = 12.0, 0.25
    
    # Axis Setup
    freq_bins = (np.arange(FFTRNGSIZE) / FFTRNGSIZE) * fs
    range_axis = (c * freq_bins) / (2.0 * slp)
    velocity_bins = np.fft.fftshift(np.fft.fftfreq(FFTDOPSIZE, d=PRI))
    velocity_axis = velocity_bins * (wavelength / 2.0)
    t_adc = np.arange(num_adc_samples) / fs

    # --- PLOT SETUP ---
    plt.ion()
    fig, axes = plt.subplots(len(snr_levels), 6)
    plt.tight_layout(pad=0.75, h_pad=2.0, w_pad=0.3)
    
    # Initialize storage for each SNR row
    histories = []
    for _ in snr_levels:
        histories.append({
            'time': [], 'true_p': [], 'est_p': [], 'err': [], 
            'm_rng': [], 't_rng': [], 'm_az': [], 't_az': []
        })

    # Prepare plot objects for fast updating
    lines = []
    for r in range(3):
        row_lines = {}
        # Col 0: Range Profile
        row_lines['lp'], = axes[r, 0].plot(range_axis, np.zeros(FFTRNGSIZE) - 100)      
        axes[r, 0].set_title(f"Range Profile (SNR: {snr_levels[r]}dB)", fontsize=10)
        axes[r, 0].set_xlabel("Range (m)", fontsize=8)
        axes[r, 0].set_ylabel("Power (dB)", fontsize=8)
        axes[r, 0].set_ylim(-55, 55)
        axes[r, 0].grid(True)
        
        # Col 1: Range-Doppler Map
        range_doppler_init = np.full((FFTRNGSIZE, FFTDOPSIZE), -100.0)
        row_lines['im'] = axes[r, 1].imshow(
            range_doppler_init,
            aspect="auto",
            origin="lower",
            extent=[velocity_axis[0], velocity_axis[-1], range_axis[0], range_axis[-1]],
            cmap="jet",
            vmin=-100.0,
            vmax=0.0,
        )
        
        axes[r, 1].set_title("Range-Doppler Map", fontsize=10)
        axes[r, 1].set_xlabel("Radial velocity (m/s)", fontsize=8)
        axes[r, 1].set_ylabel("Range (m)", fontsize=8)
        cbar = fig.colorbar(row_lines['im'], ax=axes[r, 1])
        cbar.set_label("Power (dB)")
        
        # Col 2: Trajectory
        row_lines['tr_t'], = axes[r, 2].plot([], [], 'g-', alpha=0.5, label="True Trajectory")
        row_lines['tr_e'], = axes[r, 2].plot([], [], 'b.', markersize=3, label="Estimated Trajectory")

        # Col 3: Position error over time
        row_lines['err'], = axes[r, 3].plot([], [], 'r-')
        axes[r, 3].set_title("Position Estimation Error", fontsize=10)
        axes[r, 3].set_xlabel("Time (s)", fontsize=8)
        axes[r, 3].set_ylabel("Position Error (m)", fontsize=8)
        axes[r, 3].grid(True)
        
        # Col 4: Range and azimuth measurements
        row_lines['rng_t'], = axes[r, 4].plot([], [], 'g--', label="True")
        row_lines['rng_m'], = axes[r, 4].plot([], [], 'b.', label="Measured")
        axes[r, 4].set_title("Measured Range", fontsize=10)
        axes[r, 4].set_xlabel("Time (s)", fontsize=8)
        axes[r, 4].set_ylabel("Range (m)", fontsize=8)
        axes[r, 4].grid(True)
        axes[r, 4].legend(loc="lower left")
        
        # Col 5: Azimuth measurements
        row_lines['az_m'], = axes[r, 5].plot([], [], 'm.', label="Measured")
        row_lines['az_t'], = axes[r, 5].plot([], [], 'g--', label="True")
        axes[r, 5].set_title("Measured Azimuth (AoA)", fontsize=10)
        axes[r, 5].set_xlabel("Time (s)", fontsize=8)
        axes[r, 5].set_ylabel("Azimuth (degrees)", fontsize=8)
        axes[r, 5].grid(True)
        axes[r, 5].legend(loc="lower left")
        
        # Calculate dynamic axis limits based on start position and trajectory
        # Estimate trajectory bounds (consider initial position + motion over duration)
        max_x = max(radar_position[0], start_position[0], start_position[0] + initial_target_velocity[0] * duration + 0.5 * target_acceleration[0] * duration**2)
        min_x = min(radar_position[0], start_position[0], start_position[0] + initial_target_velocity[0] * duration + 0.5 * target_acceleration[0] * duration**2)
        max_y = max(radar_position[1], start_position[1], start_position[1] + initial_target_velocity[1] * duration + 0.5 * target_acceleration[1] * duration**2)
        min_y = min(radar_position[1], start_position[1], start_position[1] + initial_target_velocity[1] * duration + 0.5 * target_acceleration[1] * duration**2)
        
        padding = 5.0
        axes[r, 2].grid(True)
        axes[r, 2].set_aspect("equal")
        axes[r, 2].set_title("Top-down View", fontsize=10)
        axes[r, 2].set_xlabel("X (m)", fontsize=8)
        axes[r, 2].set_ylabel("Y (m)", fontsize=8)
        axes[r, 2].set_xlim(min_x - padding, max_x + padding)
        axes[r, 2].set_ylim(min_y - padding, max_y + padding)
        
        print("type: ", axes[r, 2])
        
        # Plot elements
        axes[r, 2].plot([], [], "g-", linewidth=2, alpha=0.5, label="True Trajectory", markersize=3)
        axes[r, 2].plot([], [], "b--", linewidth=2, alpha=0.7, label="Estimated Trajectory", markersize=3)
        axes[r, 2].plot([], [], "go", label="True Position", markersize=3)
        axes[r, 2].plot([], [], "b^", label="Estimated Position", markersize=3)
        axes[r, 2].plot([], [], "m--", linewidth=1.5, alpha=0.6, label="AoA Line", markersize=3)
        axes[r, 2].plot([], [], "r-", linewidth=1, alpha=0.4, label="Position Error", markersize=3)

        axes[r, 2].plot(radar_position[0], radar_position[1], "r*", label="Radar", markersize=3)
        axes[r, 2].legend(bbox_to_anchor=(1.05, 0.0), loc='upper left', fontsize=4)

        lines.append(row_lines)

    # simulation loop
    for t in np.arange(0, duration + frame_rate, frame_rate):
        # general physics
        obj_position = start_position + initial_target_velocity * t + 0.5 * target_acceleration * (t**2)
        true_range = np.linalg.norm(obj_position - radar_position)
        az_true = np.arctan2(obj_position[1], obj_position[0])
        v_rad = np.dot(initial_target_velocity + target_acceleration * t, obj_position / (true_range + 1e-12))
        
        fd = 2.0 * v_rad / wavelength
        tau = 2.0 * true_range / c
        fb = slp * tau

        # process per snr
        for r, snr in enumerate(snr_levels):
            # generate signal
            data = np.zeros((num_adc_samples, num_antennas, num_chirps), dtype=np.complex128)
            for m in range(num_chirps):
                dop = np.exp(1j * 2 * np.pi * fd * m * PRI)
                for rx in range(num_antennas):
                    # phase shifts
                    ap = np.exp(1j * 2 * np.pi * (antenna_spacing * rx * np.sin(az_true)) / wavelength)
                    data[:, rx, m] = ap * dop * np.exp(1j * 2 * np.pi * fb * t_adc)
            
            # noise
            sig_p = np.mean(np.abs(data)**2)
            noise = np.sqrt((sig_p / (10**(snr/10)))/2) * (np.random.randn(*data.shape) + 1j*np.random.randn(*data.shape))
            data += noise

            # fft
            r_fft = np.fft.fft(data[:, 0, :] * np.hamming(num_adc_samples)[:, None], FFTRNGSIZE, axis=0)
            rd_map = np.fft.fftshift(np.fft.fft(r_fft * np.hamming(num_chirps), FFTDOPSIZE, axis=1), axes=1)
            rd_db = 20 * np.log10(np.abs(rd_map) + 1e-12)

            # peak detection
            r_idx, d_idx = np.unravel_index(np.argmax(rd_db), rd_db.shape)
            m_rng = range_axis[r_idx]
            
            # AoA Calculation
            # Using second antenna for phase comparison
            r_fft2 = np.fft.fft(data[:, 1, :] * np.hamming(num_adc_samples)[:, None], FFTRNGSIZE, axis=0)
            rd_map2 = np.fft.fftshift(np.fft.fft(r_fft2 * np.hamming(num_chirps), FFTDOPSIZE, axis=1), axes=1)
            
            phi1, phi2 = np.angle(rd_map[r_idx, d_idx]), np.angle(rd_map2[r_idx, d_idx])
            d_phi = np.arctan2(np.sin(phi2-phi1), np.cos(phi2-phi1))
            m_aoa = np.arcsin(np.clip((d_phi * wavelength)/(2*np.pi*antenna_spacing), -1, 1))
            est_pos = polar_to_cartesian(m_rng, m_aoa, radar_position)

            position_error = np.linalg.norm(est_pos[:2] - obj_position)

            # Update History
            h = histories[r]
            h['time'].append(t)
            h['true_p'].append(obj_position)
            h['est_p'].append(est_pos[:2].copy())
            h['err'].append(position_error)
            h['m_rng'].append(m_rng)
            h['t_rng'].append(true_range)
            h['m_az'].append(np.rad2deg(m_aoa))
            h['t_az'].append(np.rad2deg(az_true))

            # Update Plots
            lines[r]['lp'].set_ydata(20 * np.log10(np.abs(r_fft[:, 0]) + 1e-12))
            lines[r]['im'].set_data(rd_db)
            lines[r]['im'].set_clim(rd_db.max()-50, rd_db.max())
            
            tp_arr, ep_arr = np.array(h['true_p']), np.array(h['est_p'])
            lines[r]['tr_t'].set_data(tp_arr[:,0], tp_arr[:,1])
            lines[r]['tr_e'].set_data(ep_arr[:,0], ep_arr[:,1])
            lines[r]['err'].set_data(h['time'], h['err'])
            lines[r]['rng_t'].set_data(h['time'], h['t_rng'])
            lines[r]['rng_m'].set_data(h['time'], h['m_rng'])
            lines[r]['az_t'].set_data(h['time'], h['t_az'])
            lines[r]['az_m'].set_data(h['time'], h['m_az'])
            
            for c_idx in [3, 4, 5]:
                axes[r, c_idx].relim()
                axes[r, c_idx].autoscale_view()

        plt.pause(0.01)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    run_multi_snr_simulation(snr_levels=[20, 10, 2])