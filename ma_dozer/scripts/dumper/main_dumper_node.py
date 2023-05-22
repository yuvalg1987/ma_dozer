import signal
import sys
import threading
from PyQt5.QtWidgets import QApplication

from ma_dozer.configs.config import Config
from ma_dozer.scripts.dumper.dumper_controller_manager import DumperControlManager
from ma_dozer.utils.gui.gui_utils import Window
from ma_dozer.utils.imu.imu_subscriber import IMUSubscriber
from ma_dozer.utils.zmq.infrastructure import ThreadedSubscriber


def main():

    config: Config = Config()

    exit_event = threading.Event()

    control_manager = DumperControlManager(config=config, exit_event=exit_event)

    def signal_handler(sig, frame):
        print('you have stopped')
        exit_event.set()
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
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    aruco_est_position_subscriber = ThreadedSubscriber(ip=config.camera.ip,
                                                       port=config.camera.dumper_estimated_position_port,
                                                       topics=[config.topics.topic_estimated_dumper_position],
                                                       callback_func=control_manager.update_pose_aruco)

    aruco_gt_position_subscriber = ThreadedSubscriber(ip=config.camera.ip,
                                                      port=config.camera.dumper_position_port,
                                                      topics=[config.topics.topic_dumper_position],
                                                      callback_func=control_manager.update_pose_aruco)

    action_subscriber = ThreadedSubscriber(ip=config.algo.ip,
                                           port=config.algo.action_port,
                                           topics=[config.topics.topic_algo_dumper_action],
                                           callback_func=control_manager.update_action)

    imu_measurement_subscriber = IMUSubscriber(imu_config=config.dumper,
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


if __name__ == '__main__':
    main()
