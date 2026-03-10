import numpy as np

def read_dca1000(filename,
                 reshape_raw = True,
                 num_adc_samples=256,
                 num_chirps_per_frame=48,
                 num_rx=4,
                 num_tx=3):
    """
    Read raw DCA1000 binary IQ file and reshape into [frame, tx, rx, samples].
    """

    # ---- Step 1: Read raw bytes ----
    print(f"Reading file: {filename}")
    raw_data = np.fromfile(filename, dtype=np.int16)

    print("RAW DATA: ", raw_data)

    # cut off any extra data so aligned to size 4 (since this is size of IIQQ packet)
    if raw_data.size % 4 != 0:
        data_len_multiple_4 = np.floor(raw_data.size / 4) * 4
        raw_data = raw_data[0 : data_len_multiple_4]

    # Reshape data if it is in IIQQ format
    if reshape_raw:
        I2 = raw_data[1 : : 4]
        Q1 = raw_data[2 : : 4]
        iq_raw = np.copy(raw_data)
        iq_raw[1 : : 4] = Q1
        iq_raw[2 : : 4] = I2
    else:
        iq_raw = raw_data

        


    num_bytes = iq_raw.size * 2
    print(num_bytes)

    # ---- Step 2: Interpret as complex numbers ----
    # Data order: I0, Q0, I1, Q1, ...
    iq_data = iq_raw[0::2] + 1j * iq_raw[1::2]

    # ---- Step 3: Check expected data size ----
    samples_per_chirp = num_adc_samples * num_rx
    # total_expected = num_frames * num_chirps_per_frame * samples_per_chirp
    # if iq_data.size != total_expected:
    #     print(f"[Warning] Data size mismatch:")
    #     print(f"  Expected {total_expected} complex samples, got {iq_data.size}")
    num_frames = iq_data.size // (num_chirps_per_frame * samples_per_chirp)
        # print(f"  Adjusted num_frames = {num_frames}")
    iq_data = iq_data[ : num_frames * num_chirps_per_frame * num_rx * num_adc_samples]

    # ---- Step 4: Reshape to [frame, chirp, rx, samples] ----
    data = iq_data.reshape(num_frames,
                           num_chirps_per_frame,
                           num_rx,
                           num_adc_samples)

    # ---- Step 5: Group chirps by TX antenna ----
    # For TDM-MIMO, chirps repeat TX order: [TX0, TX1, TX2, TX0, TX1, TX2, ...]
    num_chirp_loops = num_chirps_per_frame // num_tx

    data = data.reshape(num_frames,
                        num_chirp_loops,
                        num_tx,
                        num_rx,
                        num_adc_samples)

    # Move TX axis out: [frame, tx, chirp_loop, rx, sample]
    data = np.transpose(data, (0, 2, 1, 3, 4))

    # ---- Step 6: Optionally flatten chirp loops if you just need per-TX stacks ----
    # e.g. data[0, 0] = all chirps from TX0 for frame 0
    # shape now: [frame, tx, chirp_loop, rx, sample]

    return data, num_frames