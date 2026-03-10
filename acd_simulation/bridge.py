import os
import sys
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
        
        self.pri = 300e-6
        self.bw = 4e9
        self.slope = self.bw / (self.num_adc_samples / self.fs)
        self.antenna_spacing = self.wavelength / 2.0

        self.range_axis = (self.c * (np.arange(self.num_adc_samples) / self.num_adc_samples) * self.fs) / (2.0 * self.slope)
        fd = np.fft.fftshift(np.fft.fftfreq(self.num_chirp_loops, d=self.pri))
        self.velocity_axis = fd * self.wavelength / 2

        self.num_angle_bins = 16
        spatial_freq = np.fft.fftshift(np.fft.fftfreq(self.num_angle_bins, d=1.0))
        sin_theta = np.clip(spatial_freq * (self.wavelength / self.antenna_spacing), -1.0, 1.0)
        self.angle_axis = np.rad2deg(np.arcsin(sin_theta))
    
    def process_radar_cube(self, raw_data):
        raw_data = raw_data - np.mean(raw_data, axis=2, keepdims=True)
        range_win = np.hamming(self.num_adc_samples).reshape(-1, 1, 1)
        doppler_win = np.hamming(self.num_chirp_loops).reshape(1, 1, -1)
        
        range_fft = np.fft.fft(raw_data * range_win, axis=0)
        bin_indices = np.arange(self.num_chirp_loops)
        omega = 2 * np.pi * bin_indices / self.num_chirp_loops

        for tx_idx in range(self.num_tx):
            start_ant = tx_idx * self.num_rx
            end_ant = (tx_idx + 1) * self.num_rx
            phase_corr = np.exp(-1j * omega * (tx_idx / self.num_tx))
            range_fft[:, start_ant:end_ant, :] *= phase_corr.reshape(1,1,-1)

        rd_fft = np.fft.fftshift(
            np.fft.fft(range_fft * doppler_win, axis=2),
            axes=2
        )

        rda_fft = np.fft.fftshift(np.fft.fft(rd_fft, n=self.num_angle_bins, axis=1), axes=1)
        return rda_fft
    
    def detect_targets_2d(self, radar_cube, threshold_db=15.0):
        # Sum across antennas to get power
        range_doppler_power = np.sum(np.abs(radar_cube)**2, axis=1)
        
        # --- DC GUARD ZONE ---
        # Zero out the first 3-5 bins (adjust based on your range resolution)
        # This prevents the "blob" at 0m from being detected
        range_doppler_power[:4, :] = 0
        
        tr, td, gr, gd = 8, 4, 4, 2
        kernel = np.ones((2*tr+2*gr+1, 2*td+2*gd+1))
        kernel[tr:tr+2*gr+1, td:td+2*gd+1] = 0
        kernel /= np.sum(kernel)
        
        noise_floor = convolve(range_doppler_power, kernel, mode='constant')
        threshold = noise_floor * (10**(threshold_db / 10.0))
        
        detections = []
        # Only look for detections where power > threshold AND power > 0
        for r, d in np.argwhere((range_doppler_power > threshold) & (range_doppler_power > 0)):
            # Local Maxima Check
            region = range_doppler_power[max(0,r-1):r+2, max(0,d-1):d+2]
            if range_doppler_power[r, d] == np.max(region):
                detections.append((r, d))
        return detections

def run_radar_simulation():
    ss = stream_data_cs_team.dataStream()
    radar = RadarSensor(ss.data[0])
    
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log_file_path = os.path.join(log_dir, "radar_stream_log.txt")
    
    plt.ion()
    fig = plt.figure(figsize=(15, 8))
    ax_rd = fig.add_subplot(1, 2, 1)
    ax_bev = fig.add_subplot(1, 2, 2)
    
    rd_plot_handle = ax_rd.imshow(
        np.zeros((radar.num_adc_samples, radar.num_chirp_loops)),
        aspect='auto',
        origin='lower',
        extent=[
            radar.velocity_axis[0],
            radar.velocity_axis[-1],
            radar.range_axis[0],
            radar.range_axis[-1]
        ])
    
    with open(log_file_path, "w", encoding="utf-8") as f:
        f.write("RADAR STREAM LOG WITH LATENCY TRACKING\n")
        f.write("="*80 + "\n")

        for frame_idx in range(ss.num_frames):
            start_time = time.time()
            
            frame_raw = ss.data[frame_idx]
            temp = frame_raw.transpose(3, 1, 0, 2)
            virtual_array = temp.reshape(
                radar.num_adc_samples,
                radar.num_chirp_loops,
                radar.num_tx * radar.num_rx
            )
            raw_data = virtual_array.transpose(0, 2, 1)
            
            # print("frame_raw:", frame_raw.shape)
            # print("raw_data:", raw_data.shape)

            radar_cube = radar.process_radar_cube(raw_data)
            detections = radar.detect_targets_2d(radar_cube)

            # RD Plotting - Update data instead of clearing
            rd_amp = np.sqrt(np.sum(np.abs(radar_cube)**2, axis=1))
            rd_db = 20.0 * np.log10(rd_amp + EPSILON)
            rd_plot_handle.set_data(rd_db)
            rd_plot_handle.set_clim(vmin=np.min(rd_db), vmax=np.max(rd_db))
            ax_rd.set_title(f"Range-Doppler Frame: {frame_idx}")
            ax_rd.set_xlabel("Velocity (m/s)")
            ax_rd.set_ylabel("Range (m)")
            
            # BEV Plotting
            ax_bev.cla()
            ax_bev.set_xlim(-10, 10)
            ax_bev.set_ylim(-10, 10)
            ax_bev.grid(True)
            ax_bev.plot(0, 0, 'r^')
            ax_bev.set_title(f"Bird's Eye View - Detections: {len(detections)}")
            ax_bev.set_xlabel("Lateral (m)")
            ax_bev.set_ylabel("Range (m)")

            processing_time = (time.time() - start_time) * 1000 # in ms
            f.write(f"\nFRAME {frame_idx} | Timestamp: {time.time():.4f} | Latency: {processing_time:.2f}ms\n")
            
            for r_idx, d_idx in detections:
                # Interpolation Logic
                r_prev, r_next = max(0, r_idx - 1), min(radar.num_adc_samples - 1, r_idx + 1)
                y0 = np.sqrt(np.sum(np.abs(radar_cube[r_idx, :, d_idx])**2))
                y_l = np.sqrt(np.sum(np.abs(radar_cube[r_prev, :, d_idx])**2))
                y_r = np.sqrt(np.sum(np.abs(radar_cube[r_next, :, d_idx])**2))
                
                # FIXED: Ensure interpolation results are scalars using .item()
                interp_r = (radar.range_axis[r_idx] + (radar.range_axis[1] - radar.range_axis[0]) * ((y_l - y_r) / (2 * (y_l + y_r - 2 * y0 + EPSILON)))).item()

                d_prev, d_next = (d_idx - 1) % radar.num_chirp_loops, (d_idx + 1) % radar.num_chirp_loops
                v_l = np.sqrt(np.sum(np.abs(radar_cube[r_idx, :, d_prev])**2))
                v_r = np.sqrt(np.sum(np.abs(radar_cube[r_idx, :, d_next])**2))
                v0 = np.sqrt(np.sum(np.abs(radar_cube[r_idx, :, d_idx])**2))
                interp_v = (radar.velocity_axis[d_idx] + (radar.velocity_axis[1] - radar.velocity_axis[0]) * ((v_l - v_r) / (2 * (v_l + v_r - 2 * v0 + EPSILON)))).item()

                angle_spectrum = np.abs(radar_cube[r_idx, :, d_idx])**2
                ang_idx = np.argmax(angle_spectrum)
                
                a0 = angle_spectrum[ang_idx] # current peak
                a_l = angle_spectrum[ang_idx - 1] if ang_idx > 0 else a0
                a_r = angle_spectrum[ang_idx + 1] if ang_idx < radar.num_angle_bins - 1 else a0
                
                if 0 < ang_idx < radar.num_angle_bins - 1:
                    frac_offset = (a_l - a_r) / (2 * (a_l + a_r - 2 * a0 + EPSILON))
                    ang_bin_width = radar.angle_axis[1] - radar.angle_axis[0]
                    m_aoa_deg = (radar.angle_axis[ang_idx] + frac_offset * ang_bin_width).item()
                else:
                    m_aoa_deg = radar.angle_axis[ang_idx].item()

                mx, my = interp_r * np.sin(np.deg2rad(m_aoa_deg)), interp_r * np.cos(np.deg2rad(m_aoa_deg))

                ax_bev.plot(mx, my, 'bo')
                f.write(f"  > R:{interp_r:6.3f}m | V:{interp_v:6.3f}m/s | A:{m_aoa_deg:6.2f}deg | X:{mx:6.3f} | Y:{my:6.3f}\n")

            plt.pause(0.01)
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    try:
        run_radar_simulation()
    except KeyboardInterrupt: 
        print("\nLog saved to logs/radar_stream_log.txt")