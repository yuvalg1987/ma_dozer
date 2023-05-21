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
    meas_log_folder = os.path.abspath(f'{folder_location}/exp_{timestamp}/')
    controller_log_location = os.path.abspath(f'{folder_location}/exp_{timestamp}_controller.txt')

    if not (os.path.exists(folder_location)):
        os.makedirs(folder_location)

    return folder_location, svo_file_location, meas_log_folder, controller_log_location


class Logger:

    def __init__(self):
        super().__init__()

        self.folder_location, \
        self.svo_file_location, \
        self.meas_log_folder, \
        self.controller_log_location = init_exp_folder()  # TODO fix

        self.controller_log_file = open(self.controller_log_location, "wb")

        self.imu_path = self.meas_log_folder + 'imu_meas.csv'
        self.imu_file = open(self.imu_path, 'w')
        self.camera_gt_file = open(self.meas_log_folder + 'camera_gt_meas.csv', 'w')
        self.camera_file = open(self.meas_log_folder + 'camera_meas.csv', 'w')
        self.imu_message_counter = 0

        self.IMU_BUFFER_CNT_MAX = 50000
        L_imu_msg = 8  # time, dt, dx,dy,dz,dyaw,dp,dr
        self.imu_buffer = np.zeros((self.IMU_BUFFER_CNT_MAX, L_imu_msg))
        self.imu_buffer_cnt = 0

        L_cam_msg = 7  # time, x,y,z,y,p,r
        self.CAM_BUFFER_CNT_MAX = 20

        self.cam_gt_buffer = np.zeros((self.CAM_BUFFER_CNT_MAX, L_cam_msg))
        self.cam_gt_buffer_cnt = 0

        self.cam_est_buffer = np.zeros((self.CAM_BUFFER_CNT_MAX, L_cam_msg))
        self.cam_est_buffer_cnt = 0

        self.init_time = 0

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

    def log_imu_readings_buffer(self):
        for k in range(0, self.imu_buffer.shape[0]):
            # self.imu_file.write(imu_buffer[k].to_log_str() + '\n')
            self.imu_file.write(self.nparray_to_str(self.imu_buffer[k]) + '\n')
        self.imu_file.flush()

    def log_camera_gt(self, camera_meas: Pose):
        self.camera_gt_file.write(camera_meas.to_log_str() + '\n')
        # self.camera_gt_file.flush()

    def log_camera_gt_buffer(self):
        for k in range(0, self.cam_gt_buffer.shape[0]):
            # self.camera_gt_file.write(cam_buffer[k].to_log_str() + '\n')
            self.camera_gt_file.write(self.nparray_to_str(self.cam_gt_buffer[k]) + '\n')
        self.camera_gt_file.flush()

    def log_camera_est(self, camera_meas: Pose):
        self.camera_file.write(camera_meas.to_log_str() + '\n')
        # self.camera_file.flush()

    def log_camera_est_buffer(self):
        for k in range(0, self.cam_est_buffer.shape[0]):
            # self.camera_file.write(cam_buffer[k].to_log_str() + '\n')
            self.camera_file.write(self.nparray_to_str(self.cam_est_buffer[k]) + '\n')
        self.camera_file.flush()

    def close_logger_files(self):
        self.log_imu_readings_buffer()
        self.imu_file.close()
        self.write_init_time()
        self.log_camera_est_buffer()
        self.camera_file.close()
        self.log_camera_gt_buffer()
        self.camera_gt_file.close()
        self.controller_log_file.close()

    def write_head_to_file(self):
        self.imu_file.write('time,dt,x,y,z,yaw,pitch,roll\n')
        self.camera_gt_file.write('time,x,y,z,yaw,pitch,roll\n')
        self.camera_file.write('time,x,y,z,yaw,pitch,roll\n')

    def write_init_time(self):
        with open(self.imu_path, 'r+') as f:
            content = f.read()
            f.seek(0, 0)
            f.write(f'init_time={self.init_time}\n' + content)
        print(f'init time successfully added to file')

    @staticmethod
    def nparray_to_str(a: np.ndarray) -> str:
        s = ['{:.20f}'.format(x) for x in a]
        s = ','.join(s)
        return s

    def add_to_imu_buffer(self, imu_meas: IMUData):
        imu_data_np = imu_meas.to_numpy()

        self.imu_buffer[self.imu_buffer_cnt, :] = imu_data_np
        self.imu_buffer_cnt += 1
        if self.imu_buffer_cnt == self.IMU_BUFFER_CNT_MAX:
            print("saving IMU Data")
            # print(self.imu_buffer_cnt)
            self.log_imu_readings_buffer()
            self.imu_buffer_cnt = 0
            self.imu_buffer *= 0
            time.sleep(2)

    def add_to_cam_gt_buffer(self, cam_meas: Pose):
        cam_data_np = cam_meas.to_numpy()

        self.cam_gt_buffer[self.cam_gt_buffer_cnt, :] = cam_data_np
        self.cam_gt_buffer_cnt += 1
        if self.cam_gt_buffer_cnt == self.CAM_BUFFER_CNT_MAX:
            print('wrote to camera_gt_file')
            self.log_camera_gt_buffer()
            self.cam_gt_buffer_cnt = 0
            self.cam_gt_buffer *= 0

    def add_to_cam_est_buffer(self, cam_meas: Pose):
        cam_data_np = cam_meas.to_numpy()

        self.cam_est_buffer[self.cam_est_buffer_cnt, :] = cam_data_np
        self.cam_est_buffer += 1
        if self.cam_est_buffer == self.CAM_BUFFER_CNT_MAX:
            print('wrote to camera_gt_file')
            self.log_camera_est_buffer()
            self.cam_est_buffer_cnt = 0
            self.cam_est_buffer *= 0
