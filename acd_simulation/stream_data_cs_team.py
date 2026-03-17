import string

from read_dca import read_dca1000
# import scipy.fft as fft
import numpy.fft as fft
import matplotlib.pyplot as plt
import numpy as np
import os

# FILENAME = "../data/adc_data_Raw_0(1).bin"
# FILENAME = "../data/iqData_Raw_0.bin"
# FILENAME = "../data/adc_data_Raw_0.bin"
# FILENAME = "../data/output.bin" # Uses OUTPUT_BIN_CONFIG
# FILENAME = "../data/adc_data_new_6min.bin"
# FILENAME = "C:/Users/potat/Downloads/Senior Capstone/radar_camera_sdk/data/adc_data_Raw_1121.bin"
FILENAME = "C:/Users/potat/Downloads/Senior Capstone/radar_camera_sdk/data/combo/stream1R.bin"
# FILENAME = "C:/Users/potat/Downloads/Senior Capstone/radar_camera_sdk/data/acrylic_with_hole/stream1R.bin"
# FILENAME = "../data/adc_data_Raw_1.bin" # uses THE_REAL_CONFIG

RESHAPE_RAW = True;


CONFIG_TYPE = "THE_REAL_CONFIG"
# CONFIG_TYPE = "NEW_CONFIG"
# CONFIG_TYPE = "OUTPUT_BIN_CONFIG"


if (CONFIG_TYPE == "THE_REAL_CONFIG"):
    NUM_ADC_SAMPLES = 256
    NUM_RX = 4
    NUM_TX = 3
    NUM_CHIRPS_PER_FRAME = 96
    NUM_CHIRP_LOOPS = NUM_CHIRPS_PER_FRAME // NUM_TX
    # NUM_FRAMES = 10
    RANGE_RESOLUTION = 0.044 # m per bin
    VELOCITY_RESOLUTION = 1.26 # m/s per bin
    FRAME_TIME_LENGTH = 100 # ms
elif (CONFIG_TYPE == "NEW_CONFIG"):
    NUM_ADC_SAMPLES = 249
    NUM_RX = 4
    NUM_TX = 3
    NUM_CHIRPS_PER_FRAME = 81
    NUM_CHIRP_LOOPS = NUM_CHIRPS_PER_FRAME // NUM_TX
    # NUM_FRAMES = 10
    RANGE_RESOLUTION = 0.044 # m per bin
    VELOCITY_RESOLUTION = 0.13 # m/s per bin
    FRAME_TIME_LENGTH = 100 # ms
elif (CONFIG_TYPE == "OUTPUT_BIN_CONFIG"):
    NUM_ADC_SAMPLES = 256
    NUM_RX = 4
    NUM_TX = 3
    NUM_CHIRPS_PER_FRAME = 48
    NUM_CHIRP_LOOPS = NUM_CHIRPS_PER_FRAME // NUM_TX
    # NUM_FRAMES = 10
    RANGE_RESOLUTION = 0.044 # m per bin
    VELOCITY_RESOLUTION = 1.26 # m/s per bin
    FRAME_TIME_LENGTH = 100 # ms -- THIS DATA IS PROBABLY WRONG FOR THIS CONFIG

class dataStream():
    def __init__(self, filename=FILENAME):
        self.filename = filename # Instance variable
        self.data, self.num_frames = read_dca1000(self.filename,
                                                    reshape_raw=RESHAPE_RAW,
                                                    num_adc_samples=NUM_ADC_SAMPLES,
                                                    num_chirps_per_frame=NUM_CHIRPS_PER_FRAME,
                                                    num_rx=NUM_RX,
                                                    num_tx=NUM_TX)
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
            # print(i)
            # print(curr_frame)
            file.write(f"\n\nINDEX: {str(idx)}\n")
            file.write(str(curr_frame))
            
    print("Finished writing to file: stream_data_output.txt")

if __name__ == '__main__':
    main()