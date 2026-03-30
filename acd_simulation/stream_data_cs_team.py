import numpy as np
import numpy.fft as fft
import matplotlib.pyplot as plt
from read_dca import read_dca1000
from bin_file_cutting_off import Fix_bin

# FILENAME = "../../radar_camera_sdk/data/adc_data_Raw_0(1).bin"
# FILENAME = "../../radar_camera_sdk/data/iqData_Raw_0.bin"
# FILENAME = "../../radar_camera_sdk/data/adc_data_Raw_0.bin"
# FILENAME = "../../radar_camera_sdk/data/output.bin" # Uses OUTPUT_BIN_CONFIG
# FILENAME = "../../radar_camera_sdk/data/adc_data_new_6min.bin"
# FILENAME = "../../radar_camera_sdk/data/adc_data_Raw_1121.bin"
# FILENAME = "../../radar_camera_sdk/data/combo/stream1R.bin"
FILENAME = "../../radar_camera_sdk/data/aligned_frames_new/output/radar/radar3.bin"
# FILENAME = "../../radar_camera_sdk/data/acrylic_with_hole/stream1R.bin"

CONFIG_FILENAME = "GoodCfg_Matlab.cfg"


class dataStream():
    def __init__(self, config= CONFIG_FILENAME, filename=FILENAME):
        
        fix_bin_file = Fix_bin(1)
        fix_bin_file.read_config(config)

        trans=fix_bin_file.get_no_transmitters()
        receiv=fix_bin_file.get_no_recievers()
        loops=fix_bin_file.get_loops()
        num_adc=fix_bin_file.get_samples()
        chirps=fix_bin_file.get_number_chirps()

        num_chirps_frame=loops*chirps

        self.data, self.num_frames = read_dca1000(filename,
                            reshape_raw=True,
                            num_adc_samples=num_adc,
                            num_chirps_per_frame=num_chirps_frame,
                            num_rx=receiv,
                            num_tx=trans)
        
        self.data = np.flip(self.data, axis=4)
        self.frame_index = 0

    def getNumFrames(self):
        return self.num_frames

    def getFrame(self):
        # curr_frame = self.data[self.frame_index, 0, :, 0 : 2, :]
        # self.frame_index += 1
        # return curr_frame, self.frame_index
        
        curr_frame = self.data[self.frame_index]
        self.frame_index += 1
        return curr_frame, self.frame_index



def main():

    stream_object = dataStream()

    num_frames = stream_object.getNumFrames()

    with open("stream_data_ouput.txt", "w", encoding="utf-8") as file:
        for _ in range(num_frames):
            curr_frame, idx = stream_object.getFrame()
            file.write(f"\n\nINDEX: {str(idx)}\n")
            file.write(str(curr_frame))
            
    print("Finished writing to file: stream_data_output.txt")

if __name__ == '__main__':
    main()