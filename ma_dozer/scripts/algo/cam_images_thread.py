from threading import Thread
from time import sleep

import cv2
import numpy as np

from ma_dozer.configs.config import Config
import ma_dozer.utils.helpers.visualization as viz


class CamImagesThread(Thread):
    def __init__(self, config: Config):
        super(CamImagesThread, self).__init__()

        self.config = config

        cv2.namedWindow("Original Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Original Depth", cv2.WINDOW_NORMAL)

    def show_images(self):
        color_image_h, depth_image_h = None, None

        if color_image_h is not None and depth_image_h is not None:

            disp_color_image = np.flipud(color_image_h)
            disp_color_image = np.fliplr(disp_color_image)

            disp_depth_image = np.flipud(depth_image_h)
            disp_depth_image = np.fliplr(disp_depth_image)
            disp_depth_image = viz.colorize_depth_with_bounds(data=disp_depth_image,
                                                              cmap='YlOrBr',
                                                              min_val=self.config.camera.lower_bound_c_h[2],
                                                              max_val=self.config.camera.upper_bound_c_h[2])

            disp_depth_image = cv2.cvtColor(disp_depth_image, cv2.COLOR_RGB2BGR)

            cv2.imshow("Original Image", disp_color_image)
            cv2.imshow("Original Depth", disp_depth_image)
            cv2.waitKey(10)

        else:
            sleep(0.01)
