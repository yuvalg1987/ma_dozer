import signal
import subprocess
import sys
import threading
import time
import platform
from PyQt5.QtWidgets import QApplication

from ma_dozer.configs.config import Config
from ma_dozer.scripts.dozer.dozer_controller_manager import DozerControlManager
from ma_dozer.utils.gui.gui_utils import Window
from ma_dozer.utils.imu.imu_subscriber import IMUSubscriber
from ma_dozer.utils.zmq.infrastructure import ThreadedSubscriber


def main():

    config: Config = Config()

    exit_event = threading.Event()

    # print(platform.system())
    # if platform.system() == 'Windows':
    #     subprocess.run(["C:\\Program Files\\Git\\bin\\bash.exe", '-l', 'init_time.sh'],
    #                    cwd='C:\\Users\\Dozer\\Documents\\Python Scripts\\ma_dozer\\ma_dozer\\scripts')

    # curr_time = time.time()
    # command = f'ssh Dozer@{config.camera.ip} date --set {curr_time}'
    # command = f''
    #
    # res = subprocess.run(ssh_command)

    # navigation_file_location = dozer_prototype_path / 'configs' / 'real_navigation_config.yaml'
    # navigation_config = NavigationConfig.from_file(navigation_file_location)

    control_manager = DozerControlManager(config=config, exit_event=exit_event)

    def signal_handler(sig,frame):
        print('you have stopped')
        exit_event.set()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # try:
    aruco_est_position_subscriber = ThreadedSubscriber(ip=config.camera.ip,
                                                       port=config.camera.dozer_estimated_position_port,
                                                       topics=[config.topics.topic_estimated_dozer_position],
                                                       callback_func=control_manager.update_pose_aruco)

    aruco_gt_position_subscriber = ThreadedSubscriber(ip=config.camera.ip,
                                                      port=config.camera.dozer_position_port,
                                                      topics=[config.topics.topic_dozer_position],
                                                      callback_func=control_manager.update_pose_aruco)

    action_subscriber = ThreadedSubscriber(ip=config.algo.ip,
                                           port=config.algo.action_port,
                                           topics=[config.topics.topic_algo_dozer_action],
                                           callback_func=control_manager.update_action)

    imu_measurement_subscriber = IMUSubscriber(imu_config=config.dozer,
                                               callback_func=control_manager.update_pose_imu)

    aruco_est_position_subscriber.start()
    aruco_gt_position_subscriber.start()
    action_subscriber.start()
    imu_measurement_subscriber.start()

    control_manager.start()

    app = QApplication(sys.argv)
    win = Window(control_manager)

    win.show()
    app.exec()

    aruco_est_position_subscriber.stop()
    aruco_gt_position_subscriber.stop()
    imu_measurement_subscriber.end_thread()
    action_subscriber.stop()
    control_manager.stop()

    control_manager.join()
    print('control manager finished')
    aruco_est_position_subscriber.join()
    print('aruco est finished')
    aruco_gt_position_subscriber.join()
    print('aruco gt finished')
    imu_measurement_subscriber.join()
    print('imu finished')
    action_subscriber.join()
    print('action finished')

    sys.exit()

    # except KeyboardInterrupt:
    #     control_manager.logger.imu_file.close()
    #     control_manager.logger.camera_file.close()
    #     control_manager.logger.camera_gt_file.close()
    #     sys.exit()


if __name__ == '__main__':
    main()
