import os
import time
import numpy as np

from ma_dozer.utils.helpers.classes import IMUData, Pose


def init_exp_folder():

    timestamp = time.strftime("%m%d_%H%M%S")
    datestamp = f"exp_{timestamp.split('_')[0][2:]}_{timestamp.split('_')[0][:2]}"
    folder_location = os.path.abspath(f'../data/{datestamp}/{timestamp}')
    svo_file_location = os.path.abspath(f'{folder_location}/exp_{timestamp}.svo')
    controller_log_location = os.path.abspath(f'{folder_location}/exp_{timestamp}_controller.txt')

    if not (os.path.exists(folder_location)):
        os.makedirs(folder_location)

    return folder_location, svo_file_location, controller_log_location


class Logger:

    def __init__(self, node_name):
        super().__init__()

        self.node_name: str = node_name

        self.folder_location, \
        self.svo_file_location, \
        self.controller_log_location = init_exp_folder()  # TODO fix

        self.controller_log_file = open(self.controller_log_location, "wb")

        self.imu_path = self.folder_location + f'/{self.node_name}_imu_meas.csv'
        print(self.imu_path)
        self.imu_file = open(self.imu_path, 'w')
        self.camera_gt_file = open(self.folder_location + f'/{self.node_name}_camera_gt_meas.csv', 'w')
        self.camera_file = open(self.folder_location + f'/{self.node_name}_camera_meas.csv', 'w')
        self.imu_message_counter = 0

        self.IMU_BUFFER_CNT_MAX = 50000
        L_imu_msg = 8  # time, dt, dx,dy,dz,dyaw,dp,dr
        self.imu_buffer = np.zeros((self.IMU_BUFFER_CNT_MAX, L_imu_msg))
        self.imu_buffer_cnt = 0

        L_cam_msg = 7  # time, x,y,z,y,p,r
        self.CAM_BUFFER_CNT_MAX = 500

        self.cam_gt_buffer = np.zeros((self.CAM_BUFFER_CNT_MAX, L_cam_msg))
        self.cam_gt_buffer_cnt = 0

        self.cam_est_buffer = np.zeros((self.CAM_BUFFER_CNT_MAX, L_cam_msg))
        self.cam_est_buffer_cnt = 0

        self.init_time = 0

    def log_controller_step(self, curr_pose, target_pose, curr_motor_command, curr_delta_eps):
        if not self.controller_log_file.closed:
            self.controller_log_file.write(f'curr_pose = {curr_pose}'.encode() + '\n'.encode())
            self.controller_log_file.write(f'target_pose = {target_pose}'.encode() + '\n'.encode())
            self.controller_log_file.write(f'curr_motor_command = {curr_motor_command}'.encode() + '\n'.encode())
            self.controller_log_file.write(f'curr_delta_eps = {curr_delta_eps}'.encode() + '\n'.encode())
            self.controller_log_file.write('\n'.encode())

    def log_controller_finished(self, curr_pose, target_pose, curr_motor_command):
        if not self.controller_log_file.closed:
            self.controller_log_file.write('finished the following action'.encode())
            self.controller_log_file.write(f'curr_pose = {curr_pose}'.encode() + '\n'.encode())
            self.controller_log_file.write(f'target_pose = {target_pose}'.encode() + '\n'.encode())
            self.controller_log_file.write(f'curr_motor_command = {curr_motor_command}'.encode() + '\n'.encode())
            self.controller_log_file.write('\n'.encode())

    def log_imu_readings_buffer(self):
        for k in range(0, self.imu_buffer.shape[0]):
            self.imu_file.write(self.nparray_to_str(self.imu_buffer[k]) + '\n')
        self.imu_file.flush()

    def log_camera_gt_buffer(self):
        for k in range(0, self.cam_gt_buffer.shape[0]):
            self.camera_gt_file.write(self.nparray_to_str(self.cam_gt_buffer[k]) + '\n')
        self.camera_gt_file.flush()

    def log_camera_est_buffer(self):
        for k in range(0, self.cam_est_buffer.shape[0]):
            self.camera_file.write(self.nparray_to_str(self.cam_est_buffer[k]) + '\n')
        self.camera_file.flush()

    def close_logger_files(self):
        self.delete_empty_rows_in_buffers()

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
        s[0] = s[0][:-21]
        s = ','.join(s)
        return s

    def add_to_imu_buffer(self, imu_meas: IMUData):
        imu_data_np = imu_meas.to_numpy()

        if self.imu_buffer_cnt < self.imu_buffer.shape[0]:
            self.imu_buffer[self.imu_buffer_cnt, :] = imu_data_np
            self.imu_buffer_cnt += 1
        if self.imu_buffer_cnt == self.IMU_BUFFER_CNT_MAX and not self.imu_file.closed:
            print("saving IMU Data")
            # print(self.imu_buffer_cnt)
            self.log_imu_readings_buffer()
            self.imu_buffer_cnt = 0
            self.imu_buffer *= 0
            time.sleep(2)

    def add_to_cam_gt_buffer(self, cam_meas: Pose):
        cam_data_np = cam_meas.to_numpy()

        if self.cam_gt_buffer_cnt < self.cam_gt_buffer.shape[0]:
            self.cam_gt_buffer[self.cam_gt_buffer_cnt, :] = cam_data_np
            self.cam_gt_buffer_cnt += 1
        if self.cam_gt_buffer_cnt == self.CAM_BUFFER_CNT_MAX and not self.camera_gt_file.closed:
            print('wrote to camera_gt_file')
            self.log_camera_gt_buffer()
            self.cam_gt_buffer_cnt = 0
            self.cam_gt_buffer *= 0

    def add_to_cam_est_buffer(self, cam_meas: Pose):
        cam_data_np = cam_meas.to_numpy()

        if self.cam_est_buffer_cnt < self.cam_est_buffer.shape[0]:
            self.cam_est_buffer[self.cam_est_buffer_cnt, :] = cam_data_np
            self.cam_est_buffer_cnt += 1
        if self.cam_est_buffer_cnt == self.CAM_BUFFER_CNT_MAX and not self.camera_file.closed:
            print('wrote to camera_est_file')
            self.log_camera_est_buffer()
            self.cam_est_buffer_cnt = 0
            self.cam_est_buffer *= 0

    def delete_empty_rows_in_buffers(self):
        self.imu_buffer = self.imu_buffer[~np.all(self.imu_buffer == 0, axis=1)]
        self.cam_gt_buffer = self.cam_gt_buffer[~np.all(self.cam_gt_buffer == 0, axis=1)]
        self.cam_est_buffer = self.cam_est_buffer[~np.all(self.cam_est_buffer == 0, axis=1)]
        

