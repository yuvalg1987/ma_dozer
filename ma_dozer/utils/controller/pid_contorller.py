import time
from pathlib import Path
from typing import Union, List

import numpy as np

from ma_dozer.configs.controller_config import ControllerConfig
from ma_dozer.utils.controller.roboclaw_wrappers import RoboclawDebugWrapper, RoboclawWrapper, RoboclawBaseWrapper
from ma_dozer.utils.controller.utils import epsilon_close_control
from ma_dozer.utils.helpers.classes import MotorCommand, Pose, Action
from ma_dozer.utils.helpers.logger import Logger


class PIDController:

    def __init__(self,
                 controller_config: ControllerConfig,
                 logger: Logger):

        self.controller_config: ControllerConfig = controller_config
        self.logger: Logger = logger

        if self.controller_config.controller_debug_mode:
            self.roboclaw_controller: RoboclawBaseWrapper = RoboclawDebugWrapper()
        else:
            self.roboclaw_controller: RoboclawBaseWrapper = RoboclawWrapper()

        self.target_pose: Union[Pose, None] = None
        self.curr_motor_command: Union[MotorCommand, None] = None
        self.curr_pose: Union[Pose, None] = None

        self.is_stop: bool = False
        self.prev_delta_xyz: float = 0
        self.prev_delta_pqr: float = 0

        self.path_debug: List = []

        camera_calibration_dir = Path(__file__).parent / '..' / '..' / '..' / 'camera_calibration'
        self.lower_bound_w_h: np.ndarray = np.load((camera_calibration_dir / 'lower_bound_w.npy').as_posix())
        self.upper_bound_w_h: np.ndarray = np.load((camera_calibration_dir / 'upper_bound_w.npy').as_posix())

        self.left_bound: float = self.controller_config.eps_bound_distance
        self.down_bound: float = self.controller_config.eps_bound_distance
        self.right_bound: float = self.upper_bound_w_h[0] - self.controller_config.eps_bound_distance
        self.up_bound: float = self.upper_bound_w_h[1] - self.controller_config.eps_bound_distance

    def update_pose(self, curr_pose: Pose):
        self.curr_pose = curr_pose

    def update_target_pose(self,
                           target_pose: Union[Pose, Action]):  # target_pose_type: MotorCommand

        self.target_pose = target_pose
        self.curr_motor_command = target_pose.motion_type

    def is_inside_bound(self):
        if self.left_bound <= self.curr_pose.position.x <= self.right_bound and \
           self.down_bound <= self.curr_pose.position.y <= self.up_bound:
            return True
        else:
            return False

    def stop(self):
        self.is_stop = True
        time.sleep(0.5)
        self.roboclaw_controller.stop()

    def rotate_left(self):

        self.is_stop = False
        res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                          curr_pose=self.curr_pose,
                                                          target_pose=self.target_pose,
                                                          motor_command=self.curr_motor_command)

        self.prev_delta_pqr = delta_yaw
        while not res and not self.is_stop:

            res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                              curr_pose=self.curr_pose,
                                                              target_pose=self.target_pose,
                                                              motor_command=self.curr_motor_command)

            curr_delta_eps_pqr = self.prev_delta_pqr - delta_yaw
            if curr_delta_eps_pqr < -self.controller_config.eps_delta_yaw:
                break

            if not self.is_inside_bound():
                break

            self.prev_delta_pqr = delta_yaw
            self.roboclaw_controller.rotate_left()

        # self.roboclaw_controller.stop()
        # print('exit left rotation')

    def rotate_right(self):

        self.is_stop = False
        res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                          curr_pose=self.curr_pose,
                                                          target_pose=self.target_pose,
                                                          motor_command=self.curr_motor_command)

        self.prev_delta_pqr = delta_yaw
        while not res and not self.is_stop:

            res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                              curr_pose=self.curr_pose,
                                                              target_pose=self.target_pose,
                                                              motor_command=self.curr_motor_command)

            curr_delta_eps_pqr = self.prev_delta_pqr - delta_yaw
            if curr_delta_eps_pqr < -self.controller_config.eps_delta_yaw:
                break

            if not self.is_inside_bound():
                break

            self.prev_delta_pqr = delta_yaw
            self.roboclaw_controller.rotate_right()

        # self.roboclaw_controller.stop()
        # print('exit right rotation')

    def forward(self):

        self.is_stop = False
        res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                          curr_pose=self.curr_pose,
                                                          target_pose=self.target_pose,
                                                          motor_command=self.curr_motor_command)

        self.prev_delta_xyz = delta_xyz
        while not res and not self.is_stop:

            res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                              curr_pose=self.curr_pose,
                                                              target_pose=self.target_pose,
                                                              motor_command=self.curr_motor_command)

            curr_delta_eps_xyz = self.prev_delta_xyz - delta_xyz

            self.logger.log_controller_step(self.curr_pose, self.target_pose, self.curr_motor_command, curr_delta_eps_xyz)
            if curr_delta_eps_xyz < -self.controller_config.eps_delta_translation:
                break

            if not self.is_inside_bound():
                break

            time.sleep(0.1)
            self.prev_delta_xyz = delta_xyz
            self.roboclaw_controller.forward()

        self.logger.log_controller_finished(self.curr_pose, self.target_pose, self.curr_motor_command)
        # self.roboclaw_controller.stop()
        # print('exit forward translation')

    def backward(self):

        self.is_stop = False
        res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                          curr_pose=self.curr_pose,
                                                          target_pose=self.target_pose,
                                                          motor_command=self.curr_motor_command)

        self.prev_delta_xyz = delta_xyz
        while not res and not self.is_stop:

            res, delta_xyz, delta_yaw = epsilon_close_control(self.controller_config,
                                                              curr_pose=self.curr_pose,
                                                              target_pose=self.target_pose,
                                                              motor_command=self.curr_motor_command)

            curr_delta_eps_xyz = self.prev_delta_xyz - delta_xyz
            self.logger.log_controller_step(self.curr_pose, self.target_pose, self.curr_motor_command, curr_delta_eps_xyz)

            if curr_delta_eps_xyz < -self.controller_config.eps_delta_translation:
                break

            if not self.is_inside_bound():
                break

            time.sleep(0.1)
            self.prev_delta_xyz = delta_xyz
            self.roboclaw_controller.backward()

        self.logger.log_controller_finished(self.curr_pose, self.target_pose, self.curr_motor_command)
        # self.roboclaw_controller.stop()
        # print('exit backward translation')

    def move_to_pose(self):

        if self.curr_motor_command == MotorCommand.ROTATE_LEFT:
            self.rotate_left()
        elif self.curr_motor_command == MotorCommand.ROTATE_RIGHT:
            self.rotate_right()
        elif self.curr_motor_command == MotorCommand.FORWARD:
            self.forward()
        elif self.curr_motor_command == MotorCommand.BACKWARD:
            self.backward()
