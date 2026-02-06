import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import convolve

# global
target_logs = {}
true_values = {'range': [], 'velocity': [], 'aoa': [], 'x': [], 'y': []}
measured_values = {'range': [], 'velocity': [], 'aoa': [], 'x': [], 'y': []}
EPSILON = 1e-12

class Target:
    def __init__(self, name, pos, vel, acc, rcs=1.0, color='g'):
        self.name = name
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.acc = np.array(acc, dtype=float)
        self.rcs = rcs
        self.color = color
        self.history = {'x': [], 'y': []}

    def update_physics(self, dt):
        self.pos += self.vel * dt + 0.5 * self.acc * (dt**2)
        self.vel += self.acc * dt
        self.history['x'].append(self.pos[0])
        self.history['y'].append(self.pos[1])

class RadarSensor:
    def __init__(self, snr_db=5):
        self.snr_db = snr_db
        self.f0, self.fs = 60e9, 12.5e6
        self.c = 299792458.0
        self.wavelength = self.c / self.f0
        self.noADC, self.noRx, self.noChirps = 256, 4, 128 # TODO: decrease to 4 later 
        self.pri, self.bw = 300e-6, 4e9
        self.slope = self.bw / (self.noADC / self.fs)
        self.antennaSpacing = self.wavelength / 2.0

        self.rangeAxis = (self.c * (np.arange(self.noADC) / self.noADC) * self.fs) / (2.0 * self.slope)
        self.velocityAxis = np.fft.fftshift(np.fft.fftfreq(self.noChirps, d=self.pri)) * (self.wavelength / 2.0)

        # angle axis corresponding to spatial FFT bins
        spatial_freq = np.fft.fftshift(np.fft.fftfreq(self.noRx, d=1.0))  # normalized element index
        sin_theta = np.clip(spatial_freq * (self.wavelength / self.antennaSpacing), -1.0, 1.0)
        self.angleAxis = np.rad2deg(np.arcsin(sin_theta))  # degrees
    
    def generate_signal(self, targets):
        data = np.zeros((self.noADC, self.noRx, self.noChirps), dtype=np.complex128)
        t_adc = np.arange(self.noADC) / self.fs
        for obj in targets:
            range_val = np.linalg.norm(obj.pos)
            azimuth = np.arctan2(obj.pos[1], obj.pos[0])
            radial_vel = np.dot(obj.vel, obj.pos / (range_val + EPSILON))
            
            tau = 2.0 * range_val / self.c
            fd = 2.0 * radial_vel / self.wavelength
            fb = self.slope * tau
            
            amp = np.sqrt(obj.rcs / (range_val**4 + EPSILON))
            for m in range(self.noChirps):
                doppler_factor = np.exp(1j * 2.0 * np.pi * fd * m * self.pri)
                for rx in range(self.noRx):
                    # assuming gain is perfect
                    ant_phase = np.exp(1j * 2.0 * np.pi * (self.antennaSpacing * rx * np.sin(azimuth)) / self.wavelength)
                    data[:, rx, m] += amp * ant_phase * doppler_factor * np.exp(1j * 2 * np.pi * fb * t_adc)
        
        sig_pwr = np.mean(np.abs(data)**2)
        noise_std = np.sqrt((sig_pwr / (10**(self.snr_db / 10.0))) / 2.0)
        return data + (noise_std * (np.random.randn(*data.shape) + 1j * np.random.randn(*data.shape)))

    def process_radar_cube(self, data):
        """
        data shape: [noADC (fast-time), noRx (antenna), noChirps (slow-time)]
        Returns a 3D cube: [range, angle, doppler]
        """
        # 1) Range FFT
        win_rng = np.hamming(self.noADC).reshape(-1, 1, 1)
        range_fft = np.fft.fft(data * win_rng, axis=0)

        # 2) Doppler FFT
        win_dop = np.hamming(self.noChirps).reshape(1, 1, -1)
        rd_fft = np.fft.fftshift(
            np.fft.fft(range_fft * win_dop, axis=2),
            axes=2
        )  # [range, rx, doppler]

        # 3) Spatial (angle) FFT across antenna array
        win_ang = np.hamming(self.noRx).reshape(1, self.noRx, 1)
        ang_fft = np.fft.fftshift(
            np.fft.fft(rd_fft * win_ang, axis=1),
            axes=1
        )  # [range, angle, doppler]

        return ang_fft

    # CFAR = adjust peak detection based on noise level
    def detect_targets_2d(self, radar_cube):
        """
        radar_cube: [range, angle, doppler]
        Performs 2D CFAR on range-doppler map integrated over angle.
        Returns detections as (range_idx, doppler_idx) pairs.
        Angle estimation is done separately via peak finding.
        """
        # Integrate power over angle dimension
        rd_pwr = np.sum(np.abs(radar_cube)**2, axis=1)  # [range, doppler]

        tr, td, gr, gd = 8, 4, 4, 2
        kernel = np.ones((2*tr+2*gr+1, 2*td+2*gd+1))
        kernel[tr:tr+2*gr+1, td:td+2*gd+1] = 0
        kernel /= (kernel.size - (2*gr+1)*(2*gd+1))

        # finds local average w zero-padding
        # pads border with zeros if kernel is outside of the img edges 
        noise_floor = convolve(rd_pwr, kernel, mode='constant')
        threshold = noise_floor * (10**(15/10.0))
        detections = []
        # check to see if its the max in a 3x3 area (local max suppression)
        for r, d in np.argwhere(rd_pwr > threshold):
            if rd_pwr[r, d] == np.max(rd_pwr[max(0,r-1):r+2, max(0,d-1):d+2]):
                detections.append((r, d))
        return detections

def run_radar_simulation(iteration: int, snr_db: int, targets: list[Target], duration: float, update_rate: float):
    # setup logs
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file_path = os.path.join(log_dir, f"{iteration}.txt")

    # create radar 
    radar = RadarSensor(snr_db=snr_db)
    
    global target_logs
    target_logs.clear() 
    for obj in targets:
        target_logs[obj.name] = {
            'range': [], 'velocity': [], 'aoa': [], 
            'true_r': [], 'true_v': [], 'true_az': []
        }

    # setup plotting
    plt.ion()
    fig = plt.figure(figsize=(16, 10))
    
    ax1 = fig.add_subplot(2, 2, 1)
    line_range_profile, = ax1.plot(radar.rangeAxis, np.zeros(radar.noADC), linewidth=2)
    ax1.set_xlabel("Range (m)")
    ax1.set_ylabel("Power (dB)")
    ax1.set_ylim(-100, 10)
    ax1.grid(True)
    
    ax2 = fig.add_subplot(2, 2, 2)
    im = ax2.imshow(np.full((radar.noADC, radar.noChirps), -100.0), aspect="auto", origin="lower", 
                    extent=[radar.velocityAxis[0], radar.velocityAxis[-1], radar.rangeAxis[0], radar.rangeAxis[-1]], 
                    cmap="jet", vmin=-100.0, vmax=0.0)
    cbar = fig.colorbar(im, ax=ax2, pad=0.02, aspect=20)
    cbar.set_label("Power (dB)")
    ax2.set_xlabel("Radial velocity (m/s)")
    ax2.set_ylabel("Range (m)")

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_aspect("equal")
    ax3.grid(True)
    ax3.set_xlim(-2, 15)
    ax3.set_ylim(-2, 15)
    ax3.plot(0, 0, "k*", markersize=12, label="Radar")
    
    truth_markers = {}
    meas_markers = {}
    trail_lines = {}
    aoa_lines = {}

    for obj in targets:
        # where the color of each target is set
        trail_lines[obj.name], = ax3.plot([], [], color=obj.color, alpha=0.5, linewidth=1, linestyle='dashed')
        truth_markers[obj.name], = ax3.plot([], [], marker="o", color=obj.color, markersize=8, alpha=0.3, linestyle='None', label=f"{obj.name} (True)")
        meas_markers[obj.name], = ax3.plot([], [], marker="x", color=obj.color, markersize=8, markeredgewidth=2, linestyle='None', label=f"{obj.name} (Meas)")
        aoa_lines[obj.name], = ax3.plot([], [], color=obj.color, alpha=1.0, linewidth=1)

    title_text = ax3.set_title("Time: 0.0s")
    ax3.legend(loc='upper right', fontsize='x-small', ncol=2)
    plt.tight_layout()

    # main simulation loop
    with open(log_file_path, "w", encoding="utf-8") as f:
        f.write(f"FFT Range Size: {radar.noADC}\n")
        f.write(f"FFT Doppler Size: {radar.noChirps}\n")
        f.write(f"FFT Angle Size: {radar.noRx}\n")
        f.write(f"SNR Setting: {snr_db} dB\n")
        f.write(f"Detection Method: 2D CFAR + Angle Peak Finding\n")

        for t_now in np.arange(0.0, duration + update_rate, update_rate):
            for obj in targets:
                obj.update_physics(update_rate)

            raw_data = radar.generate_signal(targets)
            rd_map = radar.process_radar_cube(raw_data)  # [range, angle, doppler]
            detections = radar.detect_targets_2d(rd_map)  # Returns (r_idx, d_idx) pairs

            # for visualization: angle-summed magnitude
            rd_amp = np.sqrt(np.sum(np.abs(rd_map)**2, axis=1))  # [range, doppler]
            rd_db = 20.0 * np.log10(rd_amp + EPSILON)
            max_pwr = np.max(rd_db)

            # use zero-Doppler cut for 1D range profile
            line_range_profile.set_ydata(rd_db[:, radar.noChirps//2] - max_pwr)
            im.set_data(rd_db)
            im.set_clim(vmin=max_pwr - 60, vmax=max_pwr)

            f.write(f"\n--- TIME: {t_now:.1f}s ---\n")

            processed_meas = []
            for r_idx, d_idx in detections:
                # had an issue w out of bounds
                r_prev = max(0, r_idx - 1)
                r_next = min(radar.noADC - 1, r_idx + 1)
                d_prev = (d_idx - 1) % radar.noChirps  # doppler is cyclic
                d_next = (d_idx + 1) % radar.noChirps

                # for range
                # takes magnitude (power) from all receivers (:) at that range/doppler bin and sums them up
                y0 = np.sqrt(np.sum(np.abs(rd_map[r_idx, :, d_idx])**2))
                y_l = np.sqrt(np.sum(np.abs(rd_map[r_prev, :, d_idx])**2))
                y_r = np.sqrt(np.sum(np.abs(rd_map[r_next, :, d_idx])**2))

                if r_idx > 0 and r_idx < radar.noADC - 1:
                    interp_r = radar.rangeAxis[r_idx] + (radar.rangeAxis[1] - radar.rangeAxis[0]) * (
                        (y_l - y_r) / (2 * (y_l + y_r - 2 * y0 + EPSILON))
                    )
                else:
                    interp_r = radar.rangeAxis[r_idx]

                # sub-bin interpolation for doppler
                v0 = np.sqrt(np.sum(np.abs(rd_map[r_idx, :, d_idx])**2))
                v_l = np.sqrt(np.sum(np.abs(rd_map[r_idx, :, d_prev])**2))
                v_r = np.sqrt(np.sum(np.abs(rd_map[r_idx, :, d_next])**2))
                interp_v = radar.velocityAxis[d_idx] + (radar.velocityAxis[1] - radar.velocityAxis[0]) * (
                    (v_l - v_r) / (2 * (v_l + v_r - 2 * v0 + EPSILON))
                )

                # using beamforming to get AoA estimate from angle spectrum at detected (range, doppler)
                angle_spectrum = np.abs(rd_map[r_idx, :, d_idx])**2  # [angle bins]
                ang_idx = np.argmax(angle_spectrum) # angular index
                
                # sub-bin interpolation for angle
                a_prev = max(0, ang_idx - 1)
                a_next = min(radar.noRx - 1, ang_idx + 1)
                a0 = angle_spectrum[ang_idx] # current peak
                a_l = angle_spectrum[a_prev] if ang_idx > 0 else a0 # left neighbor
                a_r = angle_spectrum[a_next] if ang_idx < radar.noRx - 1 else a0 # right neighbor
                
                if ang_idx > 0 and ang_idx < radar.noRx - 1:
                    # frac_offset = distance between a signal's true frequency and the nearest discrete Fast Fourier Transform (FFT) bin center
                    frac_offset = (a_l - a_r) / (2 * (a_l + a_r - 2 * a0 + EPSILON))
                    ang_interp_idx = ang_idx + frac_offset
                    
                    # map interpolated bin index to angle
                    ang_bin_width = radar.angleAxis[1] - radar.angleAxis[0] if len(radar.angleAxis) > 1 else 1.0 # angular resolution
                    m_aoa_deg = radar.angleAxis[ang_idx] + (ang_interp_idx - ang_idx) * ang_bin_width # back to degrees
                else:
                    m_aoa_deg = radar.angleAxis[ang_idx]
                
                m_aoa_rad = np.deg2rad(m_aoa_deg)

                mx = interp_r * np.cos(m_aoa_rad)
                my = interp_r * np.sin(m_aoa_rad)

                processed_meas.append({
                    'r': interp_r,
                    'v': interp_v,
                    'aoa': m_aoa_deg,
                    'x': mx,
                    'y': my
                })
                
            for obj in targets:
                truth_markers[obj.name].set_data([obj.pos[0]], [obj.pos[1]])
                trail_lines[obj.name].set_data(obj.history['x'], obj.history['y'])
                
                true_r = np.linalg.norm(obj.pos)
                true_v = np.dot(obj.vel, obj.pos / (true_r + EPSILON))
                true_az_deg = np.rad2deg(np.arctan2(obj.pos[1], obj.pos[0]))
                
                if processed_meas:
                    best = min(processed_meas, key=lambda m: np.sqrt((m['x']-obj.pos[0])**2 + (m['y']-obj.pos[1])**2))
                    
                    if np.sqrt((best['x']-obj.pos[0])**2 + (best['y']-obj.pos[1])**2) < 2.0:
                        meas_markers[obj.name].set_data([best['x']], [best['y']])
                        aoa_lines[obj.name].set_data([0, best['x']], [0, best['y']])

                        log_entry = (f"Target: {obj.name} | "
                                     f"True Pos: ({obj.pos[0]:.3f}, {obj.pos[1]:.3f}) m | Measured Pos: ({best['x']:.3f}, {best['y']:.3f}) m | "
                                     f"True Range: {true_r:.3f} m | Measured Range: {best['r']:.3f} m | "
                                     f"True Vel: {true_v:.3f} m/s | Measured Vel: {best['v']:.3f} m/s | "
                                     f"True Az: {true_az_deg:.2f}° | Measured AoA: {best['aoa']:.2f}°\n")
                        f.write(log_entry)

                        log = target_logs[obj.name]
                        log['range'].append(best['r'])
                        log['velocity'].append(best['v'])
                        log['aoa'].append(best['aoa'])
                        log['true_r'].append(true_r)
                        log['true_v'].append(true_v)
                        log['true_az'].append(true_az_deg)

                        # True vs Measured
                        true_values['velocity'].append(true_v)
                        true_values['range'].append(true_r)
                        true_values['aoa'].append(true_az_deg)
                        true_values['x'].append(obj.pos[0])
                        true_values['y'].append(obj.pos[1])
                        
                        measured_values['velocity'].append(best['v'])
                        measured_values['range'].append(best['r'])
                        measured_values['aoa'].append(best['aoa'])
                        measured_values['x'].append(best['x'])
                        measured_values['y'].append(best['y'])
                    else:
                        meas_markers[obj.name].set_data([], [])
                        aoa_lines[obj.name].set_data([], [])
                else:
                    meas_markers[obj.name].set_data([], [])
                    aoa_lines[obj.name].set_data([], [])

            title_text.set_text(f"Time: {t_now:.1f}s | Detections: {len(detections)}")
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.01)

    plt.ioff()
    plt.show()

def plot_differences(true_values, SNR, measured_values, title):
    range_diff = np.array(true_values['range']) - np.array(measured_values['range'])
    velocity_diff = np.array(true_values['velocity']) - np.array(measured_values['velocity'])
    aoa_diff = np.array(true_values['aoa']) - np.array(measured_values['aoa'])
    x_diff = np.array(true_values['x']) - np.array(measured_values['x'])
    y_diff = np.array(true_values['y']) - np.array(measured_values['y'])

    fig, axs = plt.subplots(4, 1, figsize=(10, 12))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    # range diff
    axs[0].plot(range_diff, label='Range Error', color='blue')
    axs[0].set_ylabel('Diff (m)')
    axs[0].set_title('Range Accuracy')
    axs[0].grid(True)

    # velocity diff
    axs[1].plot(velocity_diff, label='Velocity Error', color='orange')
    axs[1].set_ylabel('Diff (m/s)')
    axs[1].set_title('Velocity Accuracy')
    axs[1].grid(True)

    # aoa diff
    axs[2].plot(aoa_diff, label='AoA Error', color='green')
    axs[2].set_ylabel('Diff (deg)')
    axs[2].set_title('Angle of Arrival Accuracy')
    axs[2].grid(True)

    # patial Position Error (Euclidean distance between True and Measured)
    spatial_error = np.sqrt(x_diff**2 + y_diff**2)
    axs[3].plot(spatial_error, label='Euclidean Error', color='red', linestyle='--')
    axs[3].set_ylabel('Total Error (m)')
    axs[3].set_xlabel('Time Step')
    axs[3].set_title('Total Spatial Error (X/Y Combined)')
    axs[3].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
    
    if len(measured_values['range']) > 1:
        metrics = [
            ('range', 'm'), 
            ('velocity', 'm/s'), 
            ('aoa', 'deg'),
            ('x', 'm'),
            ('y', 'm')
        ]
        
        print(f"\n{'='*45}")
        print(f"VALIDATION SUMMARY (SNR = {SNR} dB)")
        print(f"Target: {title}")
        print(f"{'='*45}")
        
        for key, unit in metrics:
            err = np.array(true_values[key]) - np.array(measured_values[key])
            
            bias = np.mean(err)      # Systematic offset
            sigma = np.std(err)     # Jitter/Noise
            rmse = np.sqrt(np.mean(err**2)) # Overall Accuracy -> 
            
            # formatting (easier to read)
            scale = 1000 if unit == 'm' else 1
            s_unit = 'mm' if unit == 'm' else unit
            
            print(f"{key.upper():<10}:")
            print(f"  Bias (Offset): {bias*scale:+.3f} {s_unit}")
            print(f"  Jitter (Std):  {sigma*scale:.3f} {s_unit}")
            print(f"  Total (RMSE):  {rmse*scale:.3f} {s_unit}")
            print(f"{'-'*45}")

        # Final Euclidean Average
        avg_spatial = np.mean(spatial_error) * 1000
        print(f"MEAN SPATIAL DRIFT: {avg_spatial:.2f} mm")
        print(f"{'='*45}\n")

if __name__ == "__main__":
    # lower number = more noise
    snr_db = 5
    
    # add targets here
    # name, pos, vel, acc, rcs
    targets = [
        Target("Person 1", [15.0, 7.0], [-1.0, -1.0], [0.01, -0.01], rcs=1.0, color='red'),
        Target("Car 1", [12.0, 2.0], [-3.0, 0.5], [0.5, 0.2], rcs=5.0, color='green'),
        Target("Car 2", [5.0, 10.0], [0.5, -2.0], [0, 0.1], rcs=5.0, color='blue'),
        Target("Car 3", [7.0, 10.0], [0.5, -2.0], [0, 0.1], rcs=5.0, color='purple'),
        Target("Car 4", [3.0, 10.0], [0.5, -2.0], [0, 0.1], rcs=5.0, color='orange'),
    ]
    
    run_radar_simulation(iteration="iteration_7", snr_db=snr_db, targets=targets, duration = 10.0, update_rate = 0.2)
    if len(measured_values['range']) > 0:
        plot_differences(
            true_values, 
            snr_db, 
            measured_values, 
            f'Radar Accuracy Analysis (SNR: {snr_db}dB)'
        )
    else:
        print("Simulation ended with no data captured.")
        