import sys
import time
from threading import Thread
from typing import Union
import numpy as np
import copy

from ma_dozer.configs.config import Config
from ma_dozer.utils.controller.pid_contorller import PIDController
from ma_dozer.utils.controller.pose_estimator import PoseEstimator
from ma_dozer.utils.controller.utils import epsilon_close_plan
from ma_dozer.utils.helpers.classes import Pose, Action, CompareType, IMUData
from ma_dozer.utils.helpers.logger import Logger
from ma_dozer.utils.zmq.infrastructure import Publisher


class DozerControlManager(Thread):

    def __init__(self, config: Config, exit_event):

        super(DozerControlManager, self).__init__()

        self.init_pose: Union[Pose, None] = None
        self.curr_pose: Union[Pose, None] = None
        self.target_action: Union[Action, None] = None

        self.action_update_flag: bool = False
        self.is_stop: bool = False
        self.is_finished: bool = False

        self.config = config

        self.dozer_publisher_ack: Publisher = Publisher(ip=self.config.dozer.ip,
                                                        port=self.config.dozer.ack_port)

        self.dozer_position_publisher: Publisher = Publisher(ip=self.config.dozer.ip,
                                                             port=self.config.dozer.kalman_position_port)

        self.logger: Logger = Logger(self.config.dozer.name)
        self.enable_meas_log: bool = False

        self.controller: PIDController = PIDController(controller_config=self.config.dozer.controller,
                                                       logger=self.logger)

        self.pose_init_stage = True
        self.pose_estimator = PoseEstimator(navigation_config=config.dozer.navigation)
        self.imu_message_counter = 0
        self.imu_message_div = 5

        self.init_time = 0
        self.init_step_flag = True

        self.exit_event = exit_event

        self.logger.write_head_to_file()

    def update_action(self, curr_topic: str, curr_data: str):
        if curr_topic != self.config.topics.topic_algo_dozer_action:
            return

        target_action = Action.from_zmq_str(curr_data)
        self.target_action = target_action

        if target_action.is_init_action:
            curr_data = target_action.to_zmq_str()
            self.init_pose = target_action.to_pose(time.time_ns())
            # Aviad
            self.init_time = time.time_ns()
            print(self.init_time)

            self.logger.init_time = self.init_time
            self.curr_pose = target_action.to_pose()
            self.enable_meas_log = True
            self.init_step_flag = False

            self.is_finished = True
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_received, curr_data)
            print(f'Sent init ACK_RECEIVED {self.target_action}')
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
            print(f'Sent init ACK_FINISHED {self.target_action}')
            self.action_update_flag = False
            return

        self.action_update_flag = True
        self.is_finished = False

        curr_data = self.target_action.to_zmq_str()
        self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_received, curr_data)
        print(f'Sent ACK_RECEIVED {target_action}')

        if epsilon_close_plan(self.config.dozer.controller,
                              self.curr_pose,
                              self.target_action,
                              CompareType.ALL):

            self.is_finished = True
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
            print(f'Sent ACK_FINISHED {self.target_action}')
            self.action_update_flag = False
        else:
            print('action ignored epsilon planner')

        if not (self.target_action.position.x - 30 <= self.curr_pose.position.x <= self.target_action.position.x + 30 and
                self.target_action.position.y - 30 <= self.curr_pose.position.y <= self.target_action.position.y + 30):
            print(f'target is too far')
            self.is_finished = True
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
            print(f'Sent ACK_FINISHED {self.target_action}')
            self.action_update_flag = False
        else:
            print('action ignored 30cm bounding box')

    def update_pose_imu(self, curr_topic: str, curr_data: str):

        if self.pose_init_stage:
            return

        curr_imu_measurement = IMUData.from_zmq_str(curr_data)
        # curr_imu_measurement = self.flip_axis(curr_imu_measurement)
        if True:  # self.enable_meas_log:
            self.logger.add_to_imu_buffer(curr_imu_measurement)

        if self.config.dozer.controller.use_ekf:

            strap_down_measurement = self.pose_estimator.update_imu_measurement(curr_imu_measurement)

            rotation_rad = strap_down_measurement.att
            rotation_deg = rotation_rad / np.pi * 180

            position_m = strap_down_measurement.pos
            position_cm = position_m * 100

            curr_dozer_pose = Pose.from_arrays(position=position_cm,
                                               rotation=rotation_deg,
                                               velocity=strap_down_measurement.vel,
                                               timestamp=strap_down_measurement.time)

            self.curr_pose = curr_dozer_pose.copy()
            self.controller.update_pose(curr_dozer_pose)

            self.imu_message_counter += 1

            if self.imu_message_counter % self.imu_message_div == 0:
                pass
                # print(curr_dozer_pose)

        else:
            return

    def update_pose_aruco(self, curr_topic: str, curr_data: str):

        curr_aruco_pose = Pose.from_zmq_str(curr_data)

        if curr_topic == self.config.topics.topic_dozer_position:  # and self.enable_meas_log:
            self.logger.add_to_cam_gt_buffer(curr_aruco_pose)

        elif curr_topic == self.config.topics.topic_estimated_dozer_position:  # and self.enable_meas_log:
            self.logger.add_to_cam_est_buffer(curr_aruco_pose)

        if self.config.use_estimated_aruco_pose and curr_topic == self.config.topics.topic_dozer_position:
            return
        elif not self.config.use_estimated_aruco_pose and curr_topic == self.config.topics.topic_estimated_dozer_position:
            return

        if self.config.dozer.controller.use_ekf:

            if self.pose_init_stage:

                self.curr_pose = curr_aruco_pose.copy()
                self.pose_estimator.init(curr_aruco_pose)
                self.controller.update_pose(curr_aruco_pose)
                self.pose_init_stage = False

            else:
                strap_down_measurement = self.pose_estimator.update_aruco_measurement(curr_aruco_pose)

                rotation_rad = strap_down_measurement.att
                rotation_deg = rotation_rad / np.pi * 180

                position_m = strap_down_measurement.pos
                position_cm = position_m * 100

                curr_dozer_pose = Pose.from_arrays(position=position_cm,
                                                   rotation=rotation_deg,
                                                   velocity=strap_down_measurement.vel,
                                                   timestamp=strap_down_measurement.time)

                curr_data = curr_dozer_pose.to_zmq_str()
                self.dozer_position_publisher.send(self.config.topics.topic_kalman_estimated_dozer_position,
                                                   curr_data)

                # print(f'Aruco Measurement {curr_aruco_pose}')
                self.controller.update_pose(curr_aruco_pose)

        else:
            self.curr_pose = curr_aruco_pose.copy()
            self.controller.update_pose(curr_aruco_pose)
            self.pose_init_stage = False
            # print(f'Aruco Measurement {curr_aruco_pose}')

    def read(self):
        return self.is_finished

    def run(self):

        while not self.is_stop:

            if self.action_update_flag and not self.is_finished and self.curr_pose is not None:

                if self.exit_event.is_set():
                    self.logger.close_logger_files()
                    self.controller.stop()
                    sys.exit(0)

                if self.target_action is not None and not self.init_step_flag:
                    print(f'Before - curr pos: {self.curr_pose.position}, {self.curr_pose.rotation.yaw}\n       '
                          f'target pos: {self.target_action.position}, {self.target_action.rotation.yaw}')

                    self.controller.update_target_pose(self.target_action)  # motion_type
                    self.controller.move_to_pose()

                    print(f'After - curr pos: {self.curr_pose.position}, {self.curr_pose.rotation.yaw}\n      '
                          f'target pos: {self.target_action.position}, {self.target_action.rotation.yaw}')

                    time.sleep(0.01)

                    curr_data = self.target_action.to_zmq_str()
                    self.action_update_flag = False
                    self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
                    print(f'Sent ACK_FINISHED {self.target_action}')

            time.sleep(0.01)

        return

    def stop(self):
        print('Exit control manger')
        self.is_stop = True
        self.logger.close_logger_files()
        time.sleep(1)
        self.controller.stop()

    @staticmethod
    def flip_axis(meas: IMUData):
        meas_flipped = IMUData(
                     timestamp=copy.copy(meas.timestamp),
                     delta_t=copy.copy(meas.delta_t),
                     delta_velocity=meas.delta_velocity.copy(),
                     delta_theta=meas.delta_theta.copy())

        dv_x_old = meas_flipped.delta_velocity[0]
        dv_y_old = meas_flipped.delta_velocity[1]

        meas_flipped.delta_velocity[0] = dv_y_old
        meas_flipped.delta_velocity[1] = dv_x_old

        meas_flipped.delta_theta[0] *= -1

        return meas_flipped
