import os
import numpy as np
import matplotlib.pyplot as plt

def display_npy_images(folder_path):
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    
    index = 0
    while True:
        filename = f"image{index}.npy"
        filepath = os.path.join(folder_path, filename)
        
        if not os.path.exists(filepath):
            break
        
        img = np.load(filepath)
        
        if index == 0:
            # First image: create the imshow object
            im = ax.imshow(img)
            ax.axis('off')
        else:
            # Update the existing image
            im.set_data(img)
        
        fig.canvas.draw()
        plt.pause(0.001)  # Brief pause to update display
        
        index += 1
    
    plt.ioff()
    plt.show()

FOLDER_PATH = "../../radar_camera_sdk/data/aligned_frames_new/output/cam_images"
display_npy_images(FOLDER_PATH)