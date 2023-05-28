import numpy as np
import signal
import sys

from ma_dozer.utils.helpers.logger import Logger
from ma_dozer.utils.imu.imu_subscriber import IMUSubscriber
from ma_dozer.utils.helpers.classes import IMUData
from ma_dozer.configs.config import Config

import time


class LogIMU:
    def __init__(self):
        self.imu_buffer = []
        self.imu_buffer_cnt = 0
        self.IMU_BUFFER_CNT_MAX = 100

        self.logger = Logger('test')

    def imu_read(self, curr_topic: str, curr_data: str):
        curr_imu_measurement = IMUData.from_zmq_str(curr_data)

        self.logger.IMU_BUFFER_CNT_MAX = self.IMU_BUFFER_CNT_MAX

        print(f'{time.time_ns()} write to file...')
        self.logger.add_to_imu_buffer(curr_imu_measurement)

    def print_data(self):
        if len(self.logger.imu_buffer) > 0:
            print(f'{self.logger.imu_buffer[-1]}')


if __name__ == '__main__':

    # def signal_handler(sig, frame):
    #     print(f'user stopped the program')
    #     time.sleep(1.)
    #     log_imu.logger.log_imu_readings_buffer(imu_buffer=log_imu.imu_buffer, buff_len=log_imu.IMU_BUFFER_CNT_MAX)
    #     log_imu.logger.close_logger_files()
    #     thread_imu.end_thread()
    #     sys.exit(0)
    #
    #
    # signal.signal(signal.SIGINT, signal_handler)

    print('Program started')

    config = Config()
    log_imu = LogIMU()

    thread_imu = IMUSubscriber(imu_config=config.dozer, callback_func=log_imu.imu_read)

    thread_imu.start()

    count_sec = 0
    while count_sec < 30:
        time.sleep(1.)
        log_imu.print_data()
        count_sec += 1

    time.sleep(1.)
    thread_imu.end_thread()
    thread_imu.join()
    log_imu.logger.close_logger_files()

    print('Program closed')
