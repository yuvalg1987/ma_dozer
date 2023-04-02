import sys

from PyQt5.QtWidgets import QApplication

from ma_dozer.configs.config import Config
from ma_dozer.scripts.dumper.dumper_controller_manager import DumperControlManager
from ma_dozer.utils.gui.gui_utils import Window
from ma_dozer.utils.imu.imu_subscriber import IMUSubscriber
from ma_dozer.utils.zmq.infrastructure import ThreadedSubscriber


def main():

    config: Config = Config()

    # navigation_file_location = dozer_prototype_path / 'configs' / 'real_navigation_config.yaml'
    # navigation_config = NavigationConfig.from_file(navigation_file_location)

    control_manager = DumperControlManager(config=config)

    aruco_est_position_subscriber = ThreadedSubscriber(ip=config.camera.ip,
                                                       port=config.camera.estimated_position_port,
                                                       topics=[config.topics.topic_estimated_dumper_position],
                                                       callback_func=control_manager.update_pose_aruco)

    aruco_gt_position_subscriber = ThreadedSubscriber(ip=config.camera.ip,
                                                      port=config.camera.position_port,
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
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
