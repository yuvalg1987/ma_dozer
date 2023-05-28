from multiprocessing import Pipe, Process, set_start_method
import subprocess
import time

from ma_dozer.configs.config import Config
from ma_dozer.scripts.camera import init_time_script_path
from ma_dozer.utils.camera.utils import zed_capture_func, aruco_position_func, heightmap_proj_func, update_cupy_vars

if __name__ == '__main__':

    subprocess.call([init_time_script_path])
    time.sleep(2)
    subprocess.call(['sudo', 'jetson_clocks'])
    time.sleep(5)

    set_start_method('spawn')

    config = Config()
    # camera_config = CameraConfig()
    camera_config = update_cupy_vars(config.camera)

    color_image_receiver_0, color_image_sender_0 = Pipe(duplex=False)
    color_image_receiver_1, color_image_sender_1 = Pipe(duplex=False)
    depth_image_receiver, depth_image_sender = Pipe(duplex=False)

    zed_capture_proc    = Process(target=zed_capture_func,
                                  args=(camera_config,
                                        color_image_sender_0,
                                        color_image_sender_1,
                                        depth_image_sender,))

    aruco_position_proc = Process(target=aruco_position_func,
                                  args=(config,
                                        camera_config,
                                        color_image_receiver_0,))

    heightmap_proj_proc = Process(target=heightmap_proj_func,
                                  args=(config,
                                        camera_config,
                                        color_image_receiver_1,
                                        depth_image_receiver,))

    zed_capture_proc.start()
    aruco_position_proc.start()
    heightmap_proj_proc.start()

    zed_capture_proc.join()
    aruco_position_proc.join()
    heightmap_proj_proc.join()
