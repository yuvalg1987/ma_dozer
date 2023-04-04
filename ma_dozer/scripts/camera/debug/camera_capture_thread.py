import cv2
import numpy as np
import pyzed.sl as sl

from threading import Lock, Thread
from time import sleep

from ma_dozer.configs.nodes_config import CameraNodeConfig
from ma_dozer.utils.helpers.logger import init_exp_folder

import ma_dozer.configs.camera_globals as camera_globals


class CameraCaptureThread(Thread):

    def __init__(self, camera_config: CameraNodeConfig):

        super().__init__()
        self.camera_config = camera_config
        self.zed_camera = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        input_type = sl.InputType()
        self.init_params = sl.InitParameters(input_t=input_type)
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 30
        self.init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        self.init_params.coordinate_units = sl.UNIT.CENTIMETER

        # Open the camera
        err = self.zed_camera.open(self.init_params)
        while err != sl.ERROR_CODE.SUCCESS:
            err = self.zed_camera.open(self.init_params)
            print(err)
            print('Failed to open camera')
            sleep(1)

        print('Camera opened successfully')

        self.color_image_sl = sl.Mat()
        self.depth_image_sl = sl.Mat()

        self.runtime_parameters = sl.RuntimeParameters()
        self.image_size = sl.Resolution(camera_config.image_width,
                                        camera_config.image_height)

        if camera_config.record_video:

            folder_location, svo_file_location, _ = init_exp_folder()
            print(f'Saving video to {svo_file_location}')

            recording_param = sl.RecordingParameters(svo_file_location, sl.SVO_COMPRESSION_MODE.H264)
            err = self.zed_camera.enable_recording(recording_param)

            if err != sl.ERROR_CODE.SUCCESS:
                print(repr(err))
                exit(1)

    def run(self):

        while not camera_globals.exit_signal:

            if self.zed_camera.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                self.zed_camera.retrieve_image(self.color_image_sl, sl.VIEW.LEFT, resolution=self.image_size)
                self.zed_camera.retrieve_measure(self.depth_image_sl, sl.MEASURE.DEPTH, resolution=self.image_size)

                camera_globals.lock.acquire()

                camera_globals.color_image_global_h = self.color_image_sl.get_data()
                camera_globals.depth_image_global_h = self.depth_image_sl.get_data()

                camera_globals.color_image_global_h = cv2.cvtColor(camera_globals.color_image_global_h, cv2.COLOR_BGRA2BGR)
                camera_globals.depth_image_global_h[np.isnan(camera_globals.depth_image_global_h)] = 0

                camera_globals.new_data = True
                camera_globals.lock.release()

            sleep(0.01)

        self.zed_camera.disable_recording()
        self.zed_camera.close()

    def read(self):

        if camera_globals.new_data:
            camera_globals.lock.acquire()
            color_image_h = np.copy(camera_globals.color_image_global_h)
            depth_image_h = np.copy(camera_globals.depth_image_global_h)
            camera_globals.new_data = False
            camera_globals.lock.release()

            return color_image_h, depth_image_h

        else:

            return None, None
