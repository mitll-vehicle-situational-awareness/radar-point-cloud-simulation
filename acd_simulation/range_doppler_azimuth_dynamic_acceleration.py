import numpy as np
import matplotlib.pyplot as plt
# TODO: add another radar/antenna based on EE team setup so we can calculate 2D location

true_values = {'range': [], 'velocity': [], 'aoa':[]}
measured_values = {'range': [], 'velocity': [], 'aoa':[]}

# CHANGE THIS FOR NOISE
SNR_dB = 5  # Signal-to-Noise Ratio in dB (lower = more noise)

def run_radar_simulation():    
    # --- PARAMETERS ---
    noADC = 256
    noRx = 2
    noChirps = 128
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
    
    print(f"FFT Range Size: {FFTRNGSIZE}") # 256
    print(f"FFT Doppler Size: {FFTDOPSIZE}") # 128
    
    # NOTE: Processing gain (the thing that help boost the peak you see in the profile graph) = (10 * log base 10 (FFTRNGSIZE)) + (10 * log base 10 (FFTDOPSIZE)) => 24 + 21 = 45 db (total processing gain from fft)
    # AKA. each time u do a fft, you add gain which makes the target more "visuable"
    # Actual peak difference = SNR + Processing Gain

    # --- MODIFIED TARGET MOTION PARAMETERS (Constant Acceleration) ---
    radarPos = np.array([0.0, 0.0])
    
    # 1. Define Initial Position (r_0)
    startPos = np.array([2.0, 7.0]) 
    
    # 2. Define Initial Velocity (v_0)
    # Using the values that caused the stepped velocity output to clearly show the quantization effect
    initial_target_velocity = np.array([-0.3, -0.6]) # Example: Initial velocity vector (m/s)
    
    # 3. Define Constant Acceleration (a)
    target_acceleration = np.array([0.05, -0.07])      # Example: Constant acceleration vector (m/s^2)
    
    duration = 10.0
    UPDATE_RATE_SEC = 0.2
    # -----------------------------------------------------------------

    # Axes
    f_bins = (np.arange(FFTRNGSIZE) / FFTRNGSIZE) * fs
    rangeAxis = (c * f_bins) / (2.0 * slp)
    fAx = np.fft.fftshift(np.fft.fftfreq(FFTDOPSIZE, d=PRI))
    velocityAxis = fAx * (wavelength / 2.0)
    
    # --- PLOTTING SETUP ---
    plt.ion()
    fig = plt.figure(figsize=(16, 10))

    # Range profile
    ax1 = fig.add_subplot(2, 2, 1)
    line_range_profile, = ax1.plot(rangeAxis, np.zeros(FFTRNGSIZE), linewidth=2)
    ax1.set_xlabel("Range (m)")
    ax1.set_ylabel("Power (dB)")
    ax1.set_title(f"Range Profile (SNR = {SNR_dB} db)")
    ax1.set_ylim(-100, 0)
    ax1.grid(True)

    # Range-Doppler
    ax2 = fig.add_subplot(2, 2, 2)
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
    ax2.set_title("Range-Doppler")
    colorBar = fig.colorbar(im, ax=ax2, orientation='vertical')
    colorBar.set_label("Power (dB)")

    # Top-down geometry
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_aspect("equal")
    ax3.grid(True)
    axis_limit = 6.0
    ax3.set_xlim(min(radarPos[0], startPos[0]) - 1,
                 max(radarPos[0], startPos[0]) + axis_limit)
    ax3.set_ylim(min(radarPos[1], startPos[1]) - 1,
                 max(radarPos[1], startPos[1]) + axis_limit)
    ax3.plot(radarPos[0], radarPos[1], "r*", markersize=12)
    target_point, = ax3.plot(startPos[0], startPos[1], "go", markersize=10)
    aoa_line, = ax3.plot([0, 0], [0, 0], "c--", linewidth=1.5)
    ax3.set_title(f"Top-down view (Time: 0.0s)")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    plt.tight_layout()
    
    eps = 1e-12
    t_adc = np.arange(noADC) / fs


    # --- SIMULATION LOOP ---
    for t_now in np.arange(0.0, duration + UPDATE_RATE_SEC, UPDATE_RATE_SEC):
        if t_now > duration:
            break
        
        # Target position: r(t) = r_0 + v_0*t + 0.5*a*t^2
        objPos = startPos + initial_target_velocity * t_now + 0.5 * target_acceleration * (t_now ** 2)
        current_x, current_y = objPos[0], objPos[1]
        
        # Target INSTANTANEOUS velocity: v(t) = v_0 + a*t
        current_target_velocity = initial_target_velocity + target_acceleration * t_now

        # Update target in plot
        target_point.set_data([current_x], [current_y])

        # --- RADAR SIGNAL GENERATION ---
        data = np.zeros((noADC, noRx, noChirps), dtype=np.complex128)
        
        # Calculate range and angle based on current position (at t_now)
        R_at_start = np.linalg.norm(objPos - radarPos)
        az_at_start = np.arctan2(objPos[1] - radarPos[1], objPos[0] - radarPos[0])
        
        # Radial velocity: Projection of the INSTANTANEOUS velocity vector onto the line of sight
        radial_velocity_continuous = np.dot(current_target_velocity, (objPos - radarPos) / (R_at_start + eps))
        
        # This is the Doppler frequency that is embedded into the signal
        fd_cpi = 2.0 * radial_velocity_continuous / wavelength
        print("Doppler: ", fd_cpi)
        
        # The rest of the signal model assumes this radial velocity is constant throughout the CPI
        tau_start = 2.0 * R_at_start / c
        fb_term = slp * tau_start
        Pr_m = (Pt * Gt * Gr * (wavelength ** 2) * rcs) / (((4 * np.pi) ** 3) * (R_at_start ** 4) + eps)
        A_m = np.sqrt(np.abs(Pr_m))
        const_phase_m = np.exp(-1j * 2 * np.pi * (f0 * tau_start + 0.5 * slp * tau_start ** 2))

        for m in range(noChirps):
            doppler_factor = np.exp(1j * 2.0 * np.pi * fd_cpi * m * PRI)
            for rx in range(noRx):
                antenna_phase = np.exp(1j * 2.0 * np.pi * (antennaSpacing * rx * np.sin(az_at_start)) / wavelength)
                beat = A_m * const_phase_m * antenna_phase * doppler_factor * np.exp(1j * 2 * np.pi * fb_term * t_adc)
                data[:, rx, m] = beat

        # AWGN = Additive White Gaussian Noise
        # TODO: Add a Constant False Alarm Rate - excludes points when SNR is too low       
        
        # Add gaussian white noise to ADC signal
        # TODO: expected result -> shifting due to noise
        # no noise: 2.811, 1.197, 2.923, 1.222

        signal_power = np.mean(np.abs(data) ** 2)
        noise_power = signal_power / (10 ** (SNR_dB / 10.0))
        noise_std = np.sqrt(noise_power / 2.0) # Divide by 2 for real and imaginary components
        noise = (noise_std * np.random.randn(*data.shape)) + 1j * (noise_std * np.random.randn(*data.shape))
        data = data + noise
        print(f"DEBUG: Amplitude of Returning Signal: {A_m}")
        print(f"DEBUG: Old: {np.abs(A_m)**2} | New: {np.mean(np.abs(data) ** 2)}")

        # checking phase error (before fft)
        phi_raw = np.angle(
            (data[:,1,0] + eps) / 
            (data[:,0,0] + eps)
        )
        print("DEBUG: Raw ADC phase std (deg):", np.std(phi_raw) * 180 / np.pi)

        # Window + Range FFT
        hammingWindow = np.hamming(noADC).reshape(noADC, 1, 1)
        windowed = data * hammingWindow
        data_rx1, data_rx2 = windowed[:, 0, :], windowed[:, 1, :]
        range_fft_rx1 = np.fft.fft(data_rx1, FFTRNGSIZE, axis=0)
        range_fft_rx2 = np.fft.fft(data_rx2, FFTRNGSIZE, axis=0)

        # Remove slow-time (per-range) mean to suppress stationary clutter/DC
        range_fft_rx1 = range_fft_rx1 - np.mean(range_fft_rx1, axis=1, keepdims=True)
        range_fft_rx2 = range_fft_rx2 - np.mean(range_fft_rx2, axis=1, keepdims=True)

        # Apply slow-time window across chirps to reduce spectral leakage
        slow_win = np.hamming(noChirps)
        range_fft_rx1 = range_fft_rx1 * slow_win[np.newaxis, :]
        range_fft_rx2 = range_fft_rx2 * slow_win[np.newaxis, :]
        
        # Find peak range bin (no Doppler yet)
        rng_idx = np.argmax(np.mean(np.abs(range_fft_rx1), axis=1))

        # checking phase error (after fft)
        phi_rng = np.angle(
            (range_fft_rx2[rng_idx, :] + eps) /
            (range_fft_rx1[rng_idx, :] + eps)
        )

        print("DEBUG: Post-range FFT phase std (deg):", np.std(phi_rng) * 180 / np.pi)


        rd_rx1 = np.fft.fftshift(np.fft.fft(range_fft_rx1, FFTDOPSIZE, axis=1), axes=1)
        rd_rx2 = np.fft.fftshift(np.fft.fft(range_fft_rx2, FFTDOPSIZE, axis=1), axes=1)
        RD_dB = 20.0 * np.log10(np.abs(rd_rx1) + eps)

        # DEBUG: inspect slow-time sequence at the detected bright range bin
        # try:
            # debug_range_idx = np.argmax(np.max(np.abs(range_fft_rx1), axis=1))
            # slow_seq = range_fft_rx1[debug_range_idx, :]
            # print(f"DEBUG: bright range bin index = {debug_range_idx}")
            # print("DEBUG: slow-time magnitude (per chirp):", np.round(np.abs(slow_seq), 6))
            # print("DEBUG: slow-time phase (rad, per chirp):", np.round(np.angle(slow_seq), 6))
        # except Exception as e:
        #     print("DEBUG: failed to extract slow-time sequence:", e)

        # Find peak bin using db map
        range_idx, dop_idx = np.unravel_index(np.argmax(RD_dB), RD_dB.shape)

        #  QUADRATIC INTERPOLATION 
        # 3 or 5 points
        # sinc / splines
        # depends on the model -> (e.g. spec-limited - not in this case) 
       
        FFTRANGESIZE, FFTDOPSIZE = RD_dB.shape[0], RD_dB.shape[1]
        doppler_profile_mag = 10**(RD_dB[range_idx, :] / 20.0)
        range_profile_mag = 10**(RD_dB[:, dop_idx] / 20.0)

        if dop_idx > 0 and dop_idx < FFTDOPSIZE - 1:
            P0 = doppler_profile_mag[dop_idx]
            # two biggest point next to it
            P0_left = doppler_profile_mag[dop_idx - 1]
            P0_right = doppler_profile_mag[dop_idx + 1]

            numerator = P0_left - P0_right
            denominator = 2 * (P0_left + P0_right - 2 * P0)

            if denominator != 0:
                delta_k = numerator / denominator
            else:
                delta_k = 0 # Assume centered if peak is flat

            # Measured Vel = Center of Peak Bin + (Offset in bins * Bin Size)
            velocityBinSize = velocityAxis[1] - velocityAxis[0] # bin size = resolution
            measured_velocity = velocityAxis[dop_idx] + (delta_k * velocityBinSize)
            
        else:
            # Edge case: fall back to the simple bin center if peak is at index 0 or last index
            measured_velocity = velocityAxis[dop_idx]

        # The subsequent lines for AoA calculation (S1, S2 extraction) should remain the same
        # as they still use the original integer indices (range_idx, dop_idx).
        S1, S2 = rd_rx1[range_idx, dop_idx], rd_rx2[range_idx, dop_idx]
        delta_phi = np.angle(S2 / (S1 + eps))   # phase difference in radians
        d = antennaSpacing
        # delta_phi -> (2π * d * sin(theta))/λ  => sin(theta) = delta_phi * λ / (2π*d)
        sin_theta = (delta_phi * wavelength) / (2.0 * np.pi * d)
        sin_theta = np.clip(sin_theta, -1.0, 1.0)
        measured_aoa_rad = np.arcsin(sin_theta)
        measured_aoa_deg = np.rad2deg(measured_aoa_rad)
        # print("Measure AoA: ", measured_aoa_deg)
        
        # quadratic interpolation on range
        if range_idx > 0 and range_idx < FFTRANGESIZE - 1:
            y0 = range_profile_mag[range_idx]
            y_left = range_profile_mag[range_idx - 1]
            y_right = range_profile_mag[range_idx + 1]

            denom = 2 * (y_left + y_right - 2 * y0)
            if denom != 0:
                delta_r = (y_left - y_right) / denom
            else:
                delta_r = 0
            
            rangeBinSize = rangeAxis[1] - rangeAxis[0]
            measured_range = rangeAxis[range_idx] + (delta_r * rangeBinSize)
        else:
            measured_range = rangeAxis[range_idx]
        
        # measured_range = rangeAxis[range_idx]
        # Measured velocity comes from the center of the detected FFT bin
        # measured_velocity = velocityAxis[dop_idx]

        # --- PLOTTING UPDATE ---
        # range_profile_db = 20.0 * np.log10(np.abs(range_fft_rx1[:, 0]) + eps)
        # line_range_profile.set_ydata(range_profile_db)

        # Normalizing range profile plot so that the peak is at 0
        raw_range_profile = 20.0 * np.log10(np.abs(range_fft_rx1[:, 0]) + eps)
        # normalized_range_profile = raw_range_profile - np.max(raw_range_profile) # normalize so the peak is at 0 dB
        # line_range_profile.set_ydata(normalized_range_profile)
        
        line_range_profile.set_ydata(raw_range_profile)
        ax1.set_ylim(-100, 10)

        ax1.relim()
        ax1.autoscale_view(scaley=True)

        im.set_data(RD_dB) # update rangle-doppler map
        im.set_clim(vmin=np.max(RD_dB) - 60, vmax=np.max(RD_dB)) # adjusts color in noise floor

        # Top-down AoA line
        L = R_at_start * 1.5
        dy = current_y - radarPos[1]
        dx = current_x - radarPos[0]
        true_azimuth_rad = np.arctan2(dy, dx)
        aoa_line.set_data([radarPos[0], radarPos[0] + L * np.cos(measured_aoa_rad)],
                          [radarPos[1], radarPos[1] + L * np.sin(measured_aoa_rad)])
        ax3.set_title(f"Top-down view (Time: {t_now:.1f}s, True Az: {np.rad2deg(true_azimuth_rad):.2f}°)")

        # DEBUG
        print(f"\n--- TIME: {t_now:.1f}s ---")
        print(f"Target Pos: ({current_x:.3f}, {current_y:.3f}) m | Range: {measured_range:.3f} m | "
              f"True Vel: {radial_velocity_continuous:.3f} m/s | Measured Vel: {measured_velocity:.3f} m/s | "
              f"True Az: {np.rad2deg(true_azimuth_rad):.2f}° | Measured AoA: {measured_aoa_deg:.2f}° ")
        
        true_values['velocity'].append(radial_velocity_continuous)
        measured_values['velocity'].append(measured_velocity)

        true_values['range'].append(R_at_start)
        measured_values['range'].append(measured_range)

        true_values['aoa'].append(np.rad2deg(true_azimuth_rad))
        measured_values['aoa'].append(measured_aoa_deg)
        

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)

    plt.ioff()
    plt.show()

# Function to plot differences

def plot_differences(true_values, SNR, measured_values, title):
    range_diff = np.array(true_values['range']) - np.array(measured_values['range'])
    velocity_diff = np.array(true_values['velocity']) - np.array(measured_values['velocity'])
    aoa_diff = np.array(true_values['aoa']) - np.array(measured_values['aoa'])

    # Create subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    # Plot Range Difference
    axs[0].plot(range_diff, label='Range Difference', color='blue')
    axs[0].set_title('Difference in Range')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Difference (m)')
    axs[0].legend()

    # Plot Velocity Difference
    axs[1].plot(velocity_diff, label='Velocity Difference', color='orange')
    axs[1].set_title('Difference in Velocity')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Difference (m/s)')
    axs[1].legend()

    # Plot AoA Difference
    axs[2].plot(aoa_diff, label='AoA Difference', color='green')
    axs[2].set_title('Difference in Angle of Arrival')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Difference (degrees)')
    axs[2].legend()

    plt.tight_layout()
    plt.show()
    
    # --- ACCURACY METRICS ---
    if len(measured_values['range']) > 1:
        keys = ['range', 'velocity', 'aoa']
        units = ['m', 'm/s', 'deg']
        
        print(f"\n{'='*30}")
        print(f"VALIDATION (SNR = {SNR} dB)")
        print(f"{'='*30}")
        
        for key, unit in zip(keys, units):
            err = np.array(true_values[key]) - np.array(measured_values[key])
            
            # Bias (Mean Error) -> should be close to 0
            bias = np.mean(err)
            
            # Jitter (Standard Deviation) -> the fuzziness
            sigma = np.std(err)
            
            # RMSE (Total Accuracy)
            rmse = np.sqrt(np.mean(err**2))
            
            # change m to mm for readability
            scale = 1000 if unit != 'deg' else 1
            s_unit = 'mm' if unit != 'deg' else 'deg'
            
            print(f"{key.capitalize()}:")
            print(f"  Bias (Offset): {bias*scale:+.3f} {s_unit}")
            print(f"  Jitter (Std):  {sigma*scale:.3f} {s_unit}")
            print(f"  Total (RMSE):  {rmse*scale:.3f} {s_unit}")
            print(f"{'-'*30}")


if __name__ == "__main__":
    run_radar_simulation()
    plot_differences(true_values, SNR_dB, measured_values, 'Differences between True and Measured Values')