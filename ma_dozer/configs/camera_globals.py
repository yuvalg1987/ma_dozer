import numpy as np
from threading import Lock

def init_global_vars():

    image_width: int = 1280
    image_height: int = 720
    channel_num: int = 3

    global color_image_global_h
    global depth_image_global_h
    global lock
    global new_data
    global exit_signal

    color_image_global_h = np.zeros([image_width, image_height, channel_num], dtype=np.uint8)
    depth_image_global_h = np.zeros([image_width, image_height], dtype=np.float32)

    lock = Lock()

    exit_signal = False
    new_data = False