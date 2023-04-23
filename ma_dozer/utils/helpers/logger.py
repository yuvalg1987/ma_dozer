import os
import time
from pathlib import Path
from typing import List

import numpy as np

from ma_dozer.utils.helpers.classes import IMUData, Pose

dozer_prototype_path = Path(__file__).parent


def init_exp_folder():

    timestamp = time.strftime("%m%d_%H%M%S")
    datestamp = f"exp_{timestamp.split('_')[0][2:]}_{timestamp.split('_')[0][:2]}"
    folder_location = os.path.abspath(f'../data/{datestamp}')
    svo_file_location = os.path.abspath(f'{folder_location}/exp_{timestamp}.svo')
    course_log_file = os.path.abspath(f'{folder_location}/exp_{timestamp}.txt')
    controller_log_location = os.path.abspath(f'{folder_location}/exp_{timestamp}_controller.txt')

    if not (os.path.exists(folder_location)):
        os.makedirs(folder_location)

    return folder_location, svo_file_location, course_log_file, controller_log_location


class Logger:

    def __init__(self):
        super().__init__()

        self.folder_location, \
        self.svo_file_location, \
        self.course_log_file, \
        self.controller_log_location = init_exp_folder()  # TODO fix

        self.controller_log_file = open(self.controller_log_location, "wb")

        self.imu_file = open('./imu_meas.csv', 'w')
        self.camera_gt_file = open('./camera_gt_meas.csv', 'w')
        self.camera_file = open('./camera_meas.csv', 'w')

    def log_controller_step(self, curr_pose, target_pose, curr_motor_command, curr_delta_eps):
        self.controller_log_file.write(f'curr_pose = {curr_pose}'.encode() + '\n'.encode())
        self.controller_log_file.write(f'target_pose = {target_pose}'.encode() + '\n'.encode())
        self.controller_log_file.write(f'curr_motor_command = {curr_motor_command}'.encode() + '\n'.encode())
        self.controller_log_file.write(f'curr_delta_eps = {curr_delta_eps}'.encode() + '\n'.encode())
        self.controller_log_file.write('\n'.encode())

    def log_controller_finished(self, curr_pose, target_pose, curr_motor_command):
        self.controller_log_file.write('finished the following action'.encode())
        self.controller_log_file.write(f'curr_pose = {curr_pose}'.encode() + '\n'.encode())
        self.controller_log_file.write(f'target_pose = {target_pose}'.encode() + '\n'.encode())
        self.controller_log_file.write(f'curr_motor_command = {curr_motor_command}'.encode() + '\n'.encode())
        self.controller_log_file.write('\n'.encode())

    def log_imu_readings(self, imu_sample: IMUData):
        self.imu_file.write(imu_sample.to_log_str() + '\n')
        # self.imu_file.flush()

    def log_imu_readings_buffer(self, imu_buffer: np.ndarray, buff_len: int):
        for k in range(0, buff_len):
            # self.imu_file.write(imu_buffer[k].to_log_str() + '\n')
            self.imu_file.write(self.nparray_to_str(imu_buffer[k]) + '\n')

    def log_camera_gt(self, camera_meas: Pose):
        self.camera_gt_file.write(camera_meas.to_log_str() + '\n')
        # self.camera_gt_file.flush()

    def log_camera_gt_buffer(self, cam_buffer: np.ndarray, buff_len: int):
        for k in range(0, buff_len):
            # self.camera_gt_file.write(cam_buffer[k].to_log_str() + '\n')
            self.camera_gt_file.write(self.nparray_to_str(cam_buffer[k]) + '\n')
        self.camera_gt_file.flush()

    def log_camera_est(self, camera_meas: Pose):
        self.camera_file.write(camera_meas.to_log_str() + '\n')
        # self.camera_file.flush()

    def log_camera_est_buffer(self, cam_buffer: np.ndarray, buff_len: int):
        for k in range(0, buff_len):
            # self.camera_file.write(cam_buffer[k].to_log_str() + '\n')
            self.camera_file.write(self.nparray_to_str(cam_buffer[k]) + '\n')
        self.camera_file.flush()

    def close_logger_files(self):
        self.imu_file.close()
        self.camera_file.close()
        self.camera_gt_file.close()
        self.controller_log_file.close()

    def write_head_to_file(self):
        self.imu_file.write('time,dt,x,y,z,yaw,pitch,roll\n')
        self.camera_gt_file.write('time,x,y,z,yaw,pitch,roll\n')
        self.camera_file.write('time,x,y,z,yaw,pitch,roll\n')

    def write_init_time(self, init_time):
        self.imu_file.write(f'init_time={init_time}\n')

    @staticmethod
    def nparray_to_str(a: np.ndarray) -> str:
        s = ['{:.20f}'.format(x) for x in a]
        s = ','.join(s)
        return s
