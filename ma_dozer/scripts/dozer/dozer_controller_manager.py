import time
from threading import Thread
from typing import Union
import numpy as np

from ma_dozer.configs.config import Config
from ma_dozer.utils.controller.pid_contorller import PIDController
from ma_dozer.utils.controller.pose_estimator import PoseEstimator
from ma_dozer.utils.controller.utils import epsilon_close_plan
from ma_dozer.utils.helpers.classes import Pose, Action, CompareType, IMUData
from ma_dozer.utils.helpers.logger import Logger
from ma_dozer.utils.zmq.infrastructure import Publisher


class DozerControlManager(Thread):

    def __init__(self, config: Config):

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

        self.logger: Logger = Logger()
        self.enable_meas_log: bool = False
        # TODO: un-comment
        # self.controller: PIDController = PIDController(controller_config=self.config.dozer.controller,
        #                                                logger=self.logger)

        self.pose_init_stage = True
        self.pose_estimator = PoseEstimator(navigation_config=config.dozer.navigation)
        self.imu_message_counter = 0
        self.imu_message_div = 5

        self.init_imu_time_flag: bool = False
        self.time_sync_imu_camera = 0

    def update_action(self, curr_topic: str, curr_data: str):
        if curr_topic != self.config.topics.topic_algo_dozer_action:
            return

        target_action = Action.from_zmq_str(curr_data)

        if target_action.is_init_action:
            curr_data = target_action.to_zmq_str()
            self.init_pose = target_action.to_pose(time.time_ns())
            self.curr_pose = target_action.to_pose()
            self.enable_meas_log = True
            self.init_imu_time_flag = True
            self.is_finished = True
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_received, curr_data)
            print(f'Sent init ACK_RECEIVED {target_action}')
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
            print(f'Sent init ACK_FINISHED {self.target_action}')
            self.action_update_flag = False
            self.logger.log_camera_gt(self.curr_pose)
            return

        self.target_action = target_action
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
            # curr_data = self.target_action.to_zmq_str()
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
            print(f'Sent ACK_FINISHED {self.target_action}')
            self.action_update_flag = False

    def update_pose_imu(self, curr_topic: str, curr_data: str):

        if self.pose_init_stage:
            return

        if self.config.dozer.controller.use_ekf:

            curr_imu_measurement = IMUData.from_zmq_str(curr_data)
            strap_down_measurement = self.pose_estimator.update_imu_measurement(curr_imu_measurement)

            if self.init_imu_time_flag:
                self.time_sync_imu_camera = curr_imu_measurement.timestamp - self.init_pose.timestamp
                self.init_imu_time_flag = False

            if self.enable_meas_log:
                curr_imu_measurement.timestamp -= self.time_sync_imu_camera
                self.logger.log_imu_readings(curr_imu_measurement)

            rotation_rad = strap_down_measurement.att
            rotation_deg = rotation_rad / np.pi * 180

            position_m = strap_down_measurement.pos
            position_cm = position_m * 100

            curr_dozer_pose = Pose.from_arrays(position=position_cm,
                                               rotation=rotation_deg,
                                               velocity=strap_down_measurement.vel,
                                               timestamp=strap_down_measurement.time)

            self.curr_pose = curr_dozer_pose.copy()
            # TODO
            # self.controller.update_pose(curr_dozer_pose)

            self.imu_message_counter += 1

            if self.imu_message_counter % self.imu_message_div == 0:
                print(curr_dozer_pose)

        else:
            return

    def update_pose_aruco(self, curr_topic: str, curr_data: str):

        curr_aruco_pose = Pose.from_zmq_str(curr_data)

        if curr_topic == self.config.topics.topic_dozer_position and self.enable_meas_log:
            self.logger.log_camera_gt(curr_aruco_pose)
        elif curr_topic == self.config.topics.topic_estimated_dozer_position and self.enable_meas_log:
            self.logger.log_camera_est(curr_aruco_pose)

        if self.config.use_estimated_aruco_pose and curr_topic == self.config.topics.topic_dozer_position:
            return

        if self.config.dozer.controller.use_ekf:

            if self.pose_init_stage:

                self.curr_pose = curr_aruco_pose.copy()
                self.pose_estimator.init(curr_aruco_pose)
                # TODO
                # self.controller.update_pose(curr_aruco_pose)
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
                # self.controller.update_pose(curr_aruco_pose) was in comment before

        else:
            self.curr_pose = curr_aruco_pose.copy()
            # TODO
            # self.controller.update_pose(curr_aruco_pose)
            self.pose_init_stage = False
            print(f'Aruco Measurement {curr_aruco_pose}')

    def read(self):
        return self.is_finished

    def run(self):

        while not self.is_stop:

            if self.action_update_flag and not self.is_finished and self.curr_pose is not None:

                while not self.is_finished:

                    if self.target_action is not None:  # and motion_type is not None
                        # TODO
                        # self.controller.update_target_pose(self.target_action)  # motion_type
                        # self.controller.move_to_pose()

                        self.is_finished = epsilon_close_plan(self.config.dozer.controller,
                                                              self.curr_pose,
                                                              self.target_action,
                                                              CompareType.ALL)

                        if self.is_finished:
                            # TODO
                            # self.controller.stop()
                            break

                if self.is_finished:
                    curr_data = self.target_action.to_zmq_str()
                    self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
                    print(f'Sent ACK_FINISHED {self.target_action}')
                    self.action_update_flag = False

    def stop(self):
        print('Exit control manger')
        self.is_stop = True
        time.sleep(1)
        # TODO
        # self.controller.stop()
