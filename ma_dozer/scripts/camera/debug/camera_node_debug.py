import numpy as np
import simplejpeg
import imagezmq

from time import sleep

from ma_dozer.configs.config import Config
from ma_dozer.utils.camera.utils import calc_poses, update_cupy_vars, crop_bounds
from ma_dozer.utils.zmq.infrastructure import Publisher
from ma_dozer.scripts.camera.debug.camera_capture_thread import CameraCaptureThread

import ma_dozer.utils.camera.pcl_utils as pcl_utils
import ma_dozer.configs.camera_globals as camera_globals
import ma_dozer.utils.helpers.visualization as viz
from ma_dozer.utils.camera.aruco_utils import ArucoDetector

import cv2


def main():

    print("Starting the ZED")

    camera_globals.init_global_vars()

    config = Config()
    camera_config = update_cupy_vars(config.camera)

    capture_thread = CameraCaptureThread(camera_config=camera_config)
    capture_thread.start()

    intrinsics_d = camera_config.intrinsics_d
    rot_c2w_d = camera_config.rot_c2w_d
    t_w2c_w_d = camera_config.t_w2c_w_d
    lower_bound_d = camera_config.lower_bound_d
    upper_bound_d = camera_config.upper_bound_d

    aruco_detector = ArucoDetector(marker_length=camera_config.dozer_marker_length)

    ##############
    # Publishers #
    ##############

    dozer_position_publisher = Publisher(ip=config.camera.ip, port=config.camera.dozer_position_port)
    dumper_position_publisher = Publisher(ip=config.camera.ip, port=config.camera.dumper_position_port)

    color_camera_publisher = imagezmq.ImageSender(connect_to=config.camera.color_image_address, REQ_REP=False)
    depth_camera_publisher = imagezmq.ImageSender(connect_to=config.camera.depth_image_address, REQ_REP=False)

    cv2.namedWindow("Original Image", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Original Depth", cv2.WINDOW_NORMAL)

    while not camera_globals.exit_signal:

        color_image_h, depth_image_h = capture_thread.read()

        if color_image_h is not None and depth_image_h is not None:

            dozer_pose, dumper_pose = calc_poses(camera_config=camera_config,
                                    aruco_detector=aruco_detector,
                                    color_image_h=color_image_h)

            if dozer_pose is None or dumper_pose is None:
                print("Aruco Detection Missed")
                #continue-----------------------------

            dozer_position_publisher.send(config.topics.topic_dozer_position, dozer_pose.to_zmq_str())
            dumper_position_publisher.send(config.topics.topic_dumper_position, dumper_pose.to_zmq_str())

            print(f'Dozer Pose {dozer_pose}')
            print(f'Dumper Pose {dumper_pose}')

            if cv2.waitKey(10) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                camera_globals.exit_signal = True

            height_map_h, topview_image_h = pcl_utils.rgbd_to_top(color_image_h, depth_image_h,
                                                                  intrinsics_d, rot_c2w_d, t_w2c_w_d,
                                                                  lower_bound_d, upper_bound_d,
                                                                  camera_config.pixel_density,
                                                                  camera_config.grid_width_w,
                                                                  camera_config.grid_height_w)

            height_map_final, topview_image_final = crop_bounds(camera_config,
                                                                height_map_h,
                                                                topview_image_h)

            heightmap_corrected = height_map_final + camera_config.calibration_altitude_bias

            ##################
            # Publish Images #
            ##################

            jpeg_buffer = simplejpeg.encode_jpeg(topview_image_final, quality=95, colorspace='BGR')

            depth_camera_publisher.send_image(config.topics.topic_depth_image, heightmap_corrected)
            color_camera_publisher.send_jpg(config.topics.topic_color_image, jpeg_buffer)

            jpeg_buffer = simplejpeg.encode_jpeg(topview_image_h, quality=95, colorspace='BGR')
            depth_camera_publisher.send_image(config.topics.topic_depth_image, heightmap_corrected)
            color_camera_publisher.send_jpg(config.topics.topic_color_image, jpeg_buffer)

            jpeg_buffer = simplejpeg.encode_jpeg(color_image_h, quality=95, colorspace='BGR')
            depth_camera_publisher.send_image(config.topics.topic_depth_image, depth_image_h)
            color_camera_publisher.send_jpg(config.topics.topic_color_image, jpeg_buffer)

            disp_color_image = np.flipud(color_image_h)
            disp_color_image = np.fliplr(disp_color_image)

            disp_depth_image = np.flipud(depth_image_h)
            disp_depth_image = np.fliplr(disp_depth_image)
            disp_depth_image = viz.colorize_depth_with_bounds(data=disp_depth_image,
                                                               cmap='YlOrBr',
                                                               min_val=camera_config.lower_bound_c_h[2],
                                                               max_val=camera_config.upper_bound_c_h[2])

            disp_depth_image = cv2.cvtColor(disp_depth_image, cv2.COLOR_RGB2BGR)

            cv2.imshow("Original Image", disp_color_image)
            cv2.imshow("Original Depth", disp_depth_image)
            cv2.waitKey(10)

        else:
            sleep(0.01)

    camera_globals.exit_signal = True
    capture_thread.join()


if __name__ == '__main__':

    main()
