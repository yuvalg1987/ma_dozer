import time
import numpy as np
from typing import Optional

from ma_dozer.configs.navigation_config import NavigationConfig
from ma_dozer.utils.helpers.classes import Pose, IMUData
from ma_dozer.utils.navigation.ekf import EKF
from ma_dozer.utils.navigation.strapdown import StrapDown


class PoseEstimator:

    def __init__(self,
                 navigation_config: NavigationConfig):
        self.config = navigation_config
        self.strap_down: Optional[StrapDown] = None
        self.ekf: Optional[EKF] = None
        self.prev_time = time.time()

    def init(self, initial_pose: Pose):
        rotation_deg = initial_pose.rotation
        rotation_rad = rotation_deg * np.pi / 180

        position_cm = initial_pose.position
        position_m = position_cm / 100

        self.strap_down = StrapDown(config=self.config,
                                    pos=position_m,
                                    vel=initial_pose.velocity,
                                    att=rotation_rad,
                                    time=initial_pose.timestamp)

        self.ekf = EKF(self.config, self.strap_down)  # , init_timestep=initial_pose.timestamp

    def update_imu_measurement(self, curr_measurement: IMUData):
        curr_measurement.delta_theta -= self.strap_down.estimated_gyro_drift * curr_measurement.delta_t  # x,y,z
        curr_measurement.delta_velocity -= self.strap_down.estimated_acc_bias * curr_measurement.delta_t  # x,y,z

        self.strap_down.SD(Qv=curr_measurement.delta_velocity,
                           Qt=curr_measurement.delta_theta,
                           time=curr_measurement.timestamp,
                           dt=curr_measurement.delta_t)

        return self.strap_down.measured_state

    def update_aruco_measurement(self, curr_pose: Pose):
        rotation_deg = curr_pose.rotation
        rotation_rad = rotation_deg * np.pi / 180

        position_cm = curr_pose.position
        position_m = position_cm / 100

        curr_pose_corrected = curr_pose.copy()
        curr_pose_corrected.update_position(position_m)
        curr_pose_corrected.update_rotation(rotation_rad)

        self.ekf.kalman_update(curr_pose_corrected)

        return self.strap_down.measured_state
