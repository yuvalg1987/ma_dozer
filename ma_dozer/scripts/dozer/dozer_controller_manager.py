import sys
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

        self.logger: Logger = Logger()
        self.enable_meas_log: bool = False

        self.controller: PIDController = PIDController(controller_config=self.config.dozer.controller,
                                                       logger=self.logger)

        self.pose_init_stage = True
        self.pose_estimator = PoseEstimator(navigation_config=config.dozer.navigation)
        self.imu_message_counter = 0
        self.imu_message_div = 5

        self.init_time_flag: bool = False
        self.time_sync_pc_imu = 0
        self.time_sync_pc_camera = 0

        self.init_time = 0
        self.init_step_flag = True

        self.exit_event = exit_event

        self.logger.write_head_to_file()

    def update_action(self, curr_topic: str, curr_data: str):
        if curr_topic != self.config.topics.topic_algo_dozer_action:
            return

        target_action = Action.from_zmq_str(curr_data)
        # yakov
        print(f"target_action.is_init_action = {target_action.is_init_action}")

        if target_action.is_init_action:
            curr_data = target_action.to_zmq_str()
            self.init_pose = target_action.to_pose(time.time_ns())
            # Aviad
            self.init_time = time.time_ns()
            print(self.init_time)
            self.logger.write_init_time(init_time=self.init_time)
            self.curr_pose = target_action.to_pose()
            self.enable_meas_log = True
            self.init_time_flag = True
            self.init_step_flag = False
            self.is_finished = True
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_received, curr_data)
            print(f'Sent init ACK_RECEIVED {target_action}')
            self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
            print(f'Sent init ACK_FINISHED {self.target_action}')
            self.action_update_flag = False
            # self.logger.log_camera_gt(self.curr_pose)
            return

        # if self.init_time == 0:
        #     return

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

            # if self.init_time_flag:
            #     self.time_sync_pc_imu = curr_imu_measurement.timestamp - self.init_pose.timestamp
            #     self.init_time_flag = False

            if True:  # self.enable_meas_log:
                curr_imu_measurement.timestamp -= self.time_sync_pc_imu

                imu_data_np = np.asarray(
                    [
                        curr_imu_measurement.timestamp,
                        curr_imu_measurement.delta_t,
                        curr_imu_measurement.delta_velocity.dv_x,
                        curr_imu_measurement.delta_velocity.dv_y,
                        curr_imu_measurement.delta_velocity.dv_z,
                        curr_imu_measurement.delta_theta.delta_yaw,
                        curr_imu_measurement.delta_theta.delta_pitch,
                        curr_imu_measurement.delta_theta.delta_roll
                    ])

                self.imu_buffer[self.imu_buffer_cnt, :] = imu_data_np
                self.imu_buffer_cnt += 1
                if self.imu_buffer_cnt == self.IMU_BUFFER_CNT_MAX:
                    print("saving IMU Data")
                    print(self.imu_buffer_cnt)
                    self.logger.log_imu_readings_buffer(imu_buffer=self.imu_buffer, buff_len=self.IMU_BUFFER_CNT_MAX)
                    self.imu_buffer_cnt = 0
                    self.imu_buffer *= 0
                    time.sleep(2)
                    sys.exit()

                # self.logger.log_imu_readings(curr_imu_measurement)
                # yakov
                # print('wrote to imu_csv_file')

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
                pass  # print(curr_dozer_pose)

        # else:  # for DEBUG: save IMU when SD and EKF not activated
        #     curr_imu_measurement = IMUData.from_zmq_str(curr_data)
        #
        #     if self.init_imu_time_flag:
        #         self.time_sync_imu_camera = curr_imu_measurement.timestamp - self.init_pose.timestamp
        #         self.init_imu_time_flag = False
        #
        #     if self.enable_meas_log:
        #         curr_imu_measurement.timestamp -= self.time_sync_imu_camera
        #
        #         self.imu_buffer = np.append(self.imu_buffer, curr_imu_measurement)
        #         self.imu_buffer_cnt += 1
        #         if self.imu_buffer_cnt == self.IMU_BUFFER_CNT_MAX:
        #             print(self.imu_buffer_cnt)
        #             self.logger.log_imu_readings_buffer(imu_buffer=self.imu_buffer, buff_len=self.IMU_BUFFER_CNT_MAX)
        #             self.imu_buffer_cnt = 0
        #             self.imu_buffer = []

    def update_pose_aruco(self, curr_topic: str, curr_data: str):
        """
        this method uses the "ESTINATED" pose for ekf.
        if the received pose is of GT, the ekf should not be activated.
        """

        curr_aruco_pose = Pose.from_zmq_str(curr_data)

        # if self.init_time_flag:
        #     self.time_sync_pc_camera = curr_aruco_pose.timestamp - self.init_pose.timestamp
        #     self.init_time_flag = False

        if curr_topic == self.config.topics.topic_dozer_position:  # and self.enable_meas_log:
            curr_aruco_pose.update_timestamp(curr_aruco_pose.timestamp - self.time_sync_pc_camera)

            cam_data_np = np.asarray(
                [
                    curr_aruco_pose.timestamp,
                    curr_aruco_pose.position.x,
                    curr_aruco_pose.position.y,
                    curr_aruco_pose.position.z,
                    curr_aruco_pose.rotation.yaw,
                    curr_aruco_pose.rotation.pitch,
                    curr_aruco_pose.rotation.roll
                ])

            self.cam_gt_buffer[self.cam_gt_buffer_cnt, :] = cam_data_np
            self.cam_gt_buffer_cnt += 1
            if self.cam_gt_buffer_cnt == self.CAM_BUFFER_CNT_MAX:
                print('wrote to camera_gt_file')
                self.logger.log_camera_gt_buffer(cam_buffer=self.cam_gt_buffer, buff_len=self.CAM_BUFFER_CNT_MAX)
                self.cam_gt_buffer_cnt = 0
                self.cam_gt_buffer *= 0

            # self.logger.log_camera_gt(curr_aruco_pose)
            # yakov
            # print('wrote to camera_gt_file')
        elif curr_topic == self.config.topics.topic_estimated_dozer_position:  # and self.enable_meas_log:
            curr_aruco_pose.update_timestamp(curr_aruco_pose.timestamp -self.time_sync_pc_camera)

            cam_data_np = np.asarray(
                [
                    curr_aruco_pose.timestamp,
                    curr_aruco_pose.position.x,
                    curr_aruco_pose.position.y,
                    curr_aruco_pose.position.z,
                    curr_aruco_pose.rotation.yaw,
                    curr_aruco_pose.rotation.pitch,
                    curr_aruco_pose.rotation.roll
                ])

            self.cam_est_buffer[self.cam_est_buffer_cnt, :] = cam_data_np
            self.cam_est_buffer_cnt += 1
            if self.cam_est_buffer_cnt == self.CAM_BUFFER_CNT_MAX:
                print('wrote to camera_noisy_file')
                self.logger.log_camera_est_buffer(cam_buffer=self.cam_est_buffer, buff_len=self.CAM_BUFFER_CNT_MAX)
                self.cam_est_buffer_cnt = 0
                self.cam_est_buffer *= 0

            # self.logger.log_camera_est(curr_aruco_pose)
            # yakov
            # print('wrote to camera_noisy_file')

        if self.config.use_estimated_aruco_pose and curr_topic == self.config.topics.topic_dozer_position:
            return
        elif curr_topic == self.config.topics.topic_estimated_dozer_position:
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
                # print(f'sd position: {self.curr_pose.position}')
                # print(f'sd rotation: {self.curr_pose.rotation}')

        else:
            self.curr_pose = curr_aruco_pose.copy()
            self.controller.update_pose(curr_aruco_pose)
            self.pose_init_stage = False
            print(f'Aruco Measurement {curr_aruco_pose}')

    def read(self):
        return self.is_finished

    def run(self):

        while not self.is_stop:

            if self.action_update_flag and not self.is_finished and self.curr_pose is not None:

                while not self.is_finished:

                    if self.exit_event.is_set():
                        self.logger.close_logger_files()
                        self.controller.stop()
                        sys.exit(0)

                    if self.target_action is not None and not self.init_step_flag:  # and motion_type is not None

                        self.controller.update_target_pose(self.target_action)  # motion_type
                        self.controller.move_to_pose()

                        self.is_finished = epsilon_close_plan(self.config.dozer.controller,
                                                              self.curr_pose,
                                                              self.target_action,
                                                              CompareType.ALL)

                        if self.is_finished:
                            self.controller.stop()
                            break

                        time.sleep(0.01)

                if self.is_finished:
                    curr_data = self.target_action.to_zmq_str()
                    self.dozer_publisher_ack.send(self.config.topics.topic_dozer_ack_finished, curr_data)
                    print(f'Sent ACK_FINISHED {self.target_action}')
                    self.action_update_flag = False

            time.sleep(0.01)

    def stop(self):
        print('Exit control manger')
        self.is_stop = True
        self.logger.close_logger_files()
        time.sleep(1)
        self.controller.stop()
