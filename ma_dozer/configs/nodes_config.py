from typing import Union
import numpy as np
import subprocess
from pathlib import Path

try:
    subprocess.check_output('nvidia-smi')
    import cupy as cp
    print('Nvidia GPU detected!')
except Exception:  # this command not being found can raise quite a few different errors depending on the configuration
    import numpy as cp
    print('No Nvidia GPU in system!')

from ma_dozer.configs.controller_config import ControllerConfig
from ma_dozer.configs.navigation_config import NavigationConfig
from ma_dozer.configs.pydantic_config import BaseModel

from ma_dozer.camera_calibration import camera_calibration_dir

from pydantic import validator


class Topics(BaseModel):

    # Dozer Topics:
    topic_dozer_position: str = "dozer_position"
    topic_estimated_dozer_position: str = "estimated_dozer_position"
    topic_kalman_estimated_dozer_position: str = "kalman_estimated_dozer_position"

    topic_dozer_ack_finished: str = "dozer_ack_finished"
    topic_dozer_ack_received: str = "dozer_ack_received"
    topic_dozer_ack_intermediate_state: str = "topic_dozer_ack_intermediate_state"

    # Dumper Topics:
    topic_dumper_position: str = "dumper_position"
    topic_estimated_dumper_position: str = "estimated_dumper_position"
    topic_kalman_estimated_dumper_position: str = "kalman_estimated_dumper_position"

    topic_dumper_ack_finished: str = "dumper_ack_finished"
    topic_dumper_ack_received: str = "dumper_ack_received"
    topic_dumper_ack_intermediate_state: str = "topic_dumper_ack_intermediate_state"

    topic_color_image: str = "color_image"
    topic_depth_image: str = "depth_image"

    topic_algo_dozer_action: str = "algo_dozer_action"
    topic_algo_dumper_action: str = "algo_dumper_action"


class CameraNodeConfig(BaseModel):

    ip: str = '192.168.0.100'
    color_image_port: int = 5555
    depth_image_port: int = 5556

    dozer_position_port: int = 1234
    dozer_estimated_position_port: int = 1235

    dumper_position_port: int = 1236
    dumper_estimated_position_port: int = 1237

    color_image_address: str = None
    depth_image_address: str = None

    image_width: int = 1280
    image_height: int = 720

    fps: int = 30
    channel_num: int = 3
    record_video: bool = False

    intrinsics_h: np.ndarray = np.load((camera_calibration_dir / 'intrinsics_params.npy').as_posix())
    rot_c2w_h: np.ndarray    = np.load((camera_calibration_dir / 'rot_c2w.npy').as_posix())
    t_w2c_w_h: np.ndarray    = np.load((camera_calibration_dir / 't_w2c_w.npy').as_posix())
    dist_coeffs_h: np.ndarray = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    lower_bound_w_h: np.ndarray = np.load((camera_calibration_dir / 'lower_bound_w.npy').as_posix())
    upper_bound_w_h: np.ndarray = np.load((camera_calibration_dir / 'upper_bound_w.npy').as_posix())
    grid_size_w: np.ndarray = np.load((camera_calibration_dir / 'grid_size_w.npy').as_posix())

    lower_bound_c_h: np.ndarray = np.load((camera_calibration_dir / 'lower_bound_c.npy').as_posix())
    upper_bound_c_h: np.ndarray = np.load((camera_calibration_dir / 'upper_bound_c.npy').as_posix())
    grid_size_c: np.ndarray = np.load((camera_calibration_dir / 'grid_size_c.npy').as_posix())

    grid_size_w_path: str = (camera_calibration_dir / 'grid_size_w.npy').as_posix()
    grid_size_w_crop_path: str = (camera_calibration_dir / 'grid_size_w_crop.npy').as_posix()
    grid_size_c_path: str = (camera_calibration_dir / 'grid_size_c.npy').as_posix()

    xaxis_marker_w: np.ndarray = np.load((camera_calibration_dir / 'xaxis_marker_w.npy').as_posix())
    yaxis_marker_w: np.ndarray = np.load((camera_calibration_dir / 'yaxis_marker_w.npy').as_posix())

    dozer_aruco_position_added_noise_start: float = -2
    dozer_aruco_position_added_noise_end: float = 2
    dozer_aruco_rotation_added_noise_start: float = -2
    dozer_aruco_rotation_added_noise_end: float = 2

    dumper_aruco_position_added_noise_start: float = -2
    dumper_aruco_position_added_noise_end: float = 2
    dumper_aruco_rotation_added_noise_start: float = -2
    dumper_aruco_rotation_added_noise_end: float = 2

    pcl_xy_pixel_size: float = 0.5
    marker_length: float = 11.8
    dozer_marker_length = 15.2
    dumper_marker_length = 15.2
    pixel_density: float = np.float32(2.0)
    calibration_altitude_bias: float = 5
    bbox_low_percentile: int = 5
    bbox_high_percentile: int = 95

    inertial_aruco_marker_id: int = 0
    yaxis_aruco_marker_id : int = 1
    xaxis_aruco_marker_id : int = 2
    dozer_aruco_marker_id : int = 3
    dumper_aruco_marker_id: int = 4

    grid_height_w: int = None
    grid_width_w: int = None
    grid_height_w_crop: int = None
    grid_width_w_crop: int = None
    grid_height_c: int = None
    grid_width_c: int = None

    intrinsics_d: Union[cp.ndarray, np.ndarray] = None
    rot_c2w_d: Union[cp.ndarray, np.ndarray] = None
    t_w2c_w_d: Union[cp.ndarray, np.ndarray] = None

    lower_bound_d: Union[cp.ndarray, np.ndarray] = None
    upper_bound_d: Union[cp.ndarray, np.ndarray] = None

    @validator("color_image_address", always=True)
    def init_color_image_address(cls, v, values):
        return f'tcp://{values["ip"]}:{values["color_image_port"]}'

    @validator("depth_image_address", always=True)
    def init_depth_image_address(cls, v, values):
        return f'tcp://{values["ip"]}:{values["depth_image_port"]}'

    @validator("grid_height_w", always=True)
    def init_grid_height_w(cls, v, values):
        grid_size_w = np.load(values['grid_size_w_path'])
        return grid_size_w[0]

    @validator("grid_width_w", always=True)
    def init_grid_width_w(cls, v, values):
        grid_size_w = np.load(values['grid_size_w_path'])
        return grid_size_w[1]

    @validator("grid_height_w_crop", always=True)
    def init_grid_height_w_crop(cls, v, values):
        grid_size_w_crop = np.load(values['grid_size_w_crop_path'])
        return grid_size_w_crop[0]

    @validator("grid_width_w_crop", always=True)
    def init_grid_width_w_crop(cls, v, values):
        grid_size_w_crop = np.load(values['grid_size_w_crop_path'])
        return grid_size_w_crop[1]

    @validator("grid_height_c", always=True)
    def init_grid_height_c(cls, v, values):
        grid_size_c = np.load(values['grid_size_c_path'])
        return grid_size_c[0]

    @validator("grid_width_c", always=True)
    def init_grid_width_c(cls, v, values):
        grid_size_c = np.load(values['grid_size_c_path'])
        return grid_size_c[1]


class DozerNode(BaseModel):

    ip: str = '192.168.0.101'
    ack_port: int = 1235
    path_port: int = 1236
    name: str = 'dozer'

    action_file_path: str = './1_actions.txt'

    imu_port: str = '/dev/ttyUSB0' # 'COM7 or COM8 for Windows
    imu_baud_rate: int = 115200
    kalman_position_port: int = 1237

    controller: ControllerConfig = ControllerConfig()
    navigation: NavigationConfig = NavigationConfig()


class DumperNode(BaseModel):

    ip: str = '192.168.0.102'
    ack_port: int = 1235
    path_port: int = 1236
    name: str = 'dumper'

    action_file_path = './1_actions.txt'

    imu_port = '/dev/ttyUSB0'  # 'COM7 or COM8 for Windows
    imu_baud_rate = 115200
    kalman_position_port: int = 1237

    controller: ControllerConfig = ControllerConfig()
    navigation: NavigationConfig = NavigationConfig()


class AlgoNode(BaseModel):

    ip: str = '192.168.0.103'
    action_port: int = 1236


class NetworkConfig(BaseModel):

    topics: Topics = Topics()

    camera_node: CameraNodeConfig = CameraNodeConfig()
    algo_node: AlgoNode = AlgoNode()
    dozer_node: DozerNode = DozerNode()
    dumper_node: DumperNode = DumperNode()

    dozer_aruco_idx: int = 3
    # use_estimated_aruco_pose = True
