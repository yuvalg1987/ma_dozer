import cv2
import sys
import math
import time
import argparse
import numpy as np
import cupy as cp
import pyzed.sl as sl
from time import sleep

import imagezmq
import simplejpeg
from scipy.stats import uniform

from ma_dozer.configs.config import Config
from ma_dozer.configs.nodes_config import CameraNode
from ma_dozer.utils.helpers.classes import Pose, Rotation, Position
from ma_dozer.utils.zmq.infrastructure import Publisher


def calc_dozer_pose(camera_config: CameraNode,
                    aruco_detector,
                    color_image_h: np.ndarray):
    ids, \
    corners, \
    rotation_vecs_i2c, \
    translation_vecs_c2i_c = aruco_detector.find_markers(color_image_h,
                                                         camera_config.intrinsics_h,
                                                         camera_config.dist_coeffs_h)

    curr_timestamp = time.time_ns()

    if ids is None or \
            rotation_vecs_i2c is None or \
            translation_vecs_c2i_c is None or \
            translation_vecs_c2i_c is None:
        return None

    if camera_config.dozer_aruco_marker_id in ids:
        aruco_detector.draw_markers(color_image_h,
                                    corners, ids,
                                    camera_config.intrinsics_h,
                                    camera_config.dist_coeffs_h,
                                    rotation_vecs_i2c,
                                    translation_vecs_c2i_c)

        dozer_idx = np.where(ids == camera_config.dozer_aruco_marker_id)[0].item()
        dozer_marker_c = Position.from_array(translation_vecs_c2i_c[dozer_idx, :].T)
        dozer_marker_w = Position.from_array(
            camera_config.rot_c2w_h @ dozer_marker_c + camera_config.t_w2c_w_h.squeeze())

        rot_dozer2c, _ = cv2.Rodrigues(rotation_vecs_i2c[dozer_idx, :])
        rot_dozer2w = Rotation.from_dcm((camera_config.rot_c2w_h @ rot_dozer2c).T)
        dozer_pose = Pose.from_position(camera_config.dozer_aruco_marker_id,
                                        dozer_marker_w,
                                        rot_dozer2w,
                                        curr_timestamp)

        return dozer_pose


def crop_bounds(camera_config: CameraNode,
                height_map_h: np.ndarray,
                topview_image_h: np.ndarray):
    lower_bound_crop_x = abs(int(camera_config.lower_bound_w_h[0] * camera_config.pixel_density))
    lower_bound_crop_y = abs(int(camera_config.lower_bound_w_h[1] * camera_config.pixel_density))

    upper_bound_crop_x = min(
        abs(int(camera_config.xaxis_marker_w[0] * camera_config.pixel_density)) + lower_bound_crop_x,
        camera_config.grid_width_w)
    upper_bound_crop_y = min(
        abs(int(camera_config.yaxis_marker_w[1] * camera_config.pixel_density)) + lower_bound_crop_y,
        camera_config.grid_height_w)

    upper_bound_crop_x -= (upper_bound_crop_x - lower_bound_crop_x) % int(camera_config.pixel_density)
    upper_bound_crop_y -= (upper_bound_crop_y - lower_bound_crop_y) % int(camera_config.pixel_density)

    height_map_final = height_map_h[lower_bound_crop_y:upper_bound_crop_y, lower_bound_crop_x:upper_bound_crop_x].copy()
    topview_image_final = topview_image_h[lower_bound_crop_y:upper_bound_crop_y,
                          lower_bound_crop_x:upper_bound_crop_x].copy()

    return height_map_final, topview_image_final


def update_cupy_vars(camera_config: CameraNode):
    camera_config.intrinsics_d = cp.asarray(camera_config.intrinsics_h, dtype=cp.float32)
    camera_config.rot_c2w_d = cp.asarray(camera_config.rot_c2w_h, dtype=cp.float32)
    camera_config.t_w2c_w_d = cp.asarray(camera_config.t_w2c_w_h, dtype=cp.float32)

    camera_config.lower_bound_d = cp.asarray(camera_config.lower_bound_w_h, dtype=cp.float32)
    camera_config.upper_bound_d = cp.asarray(camera_config.upper_bound_w_h, dtype=cp.float32)
    return camera_config


def zed_capture_func(camera_config, color_image_sender_0, color_image_sender_1, depth_image_sender):
    zed_camera = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    input_type = sl.InputType()
    init_params = sl.InitParameters(input_t=input_type)
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.CENTIMETER

    # Open the camera
    err = zed_camera.open(init_params)
    while err != sl.ERROR_CODE.SUCCESS:
        err = zed_camera.open(init_params)
        print(err)
        print('Failed to open camera')
        sleep(1)

    print('Camera opened successfully')

    color_image_sl = sl.Mat()
    depth_image_sl = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()
    image_size = sl.Resolution(camera_config.image_width,
                               camera_config.image_height)

    if camera_config.record_video:

        folder_location, svo_file_location, _ = init_exp_folder()
        print(f'Saving video to {svo_file_location}')

        recording_param = sl.RecordingParameters(svo_file_location, sl.SVO_COMPRESSION_MODE.H264)
        err = zed_camera.enable_recording(recording_param)

        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            exit(1)

    counter = 0
    while True:

        if zed_camera.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed_camera.retrieve_image(color_image_sl, sl.VIEW.LEFT, resolution=image_size)
            zed_camera.retrieve_measure(depth_image_sl, sl.MEASURE.DEPTH, resolution=image_size)

            color_image_h = color_image_sl.get_data()
            depth_image_h = depth_image_sl.get_data()

            color_image_h = cv2.cvtColor(color_image_h, cv2.COLOR_BGRA2BGR)
            depth_image_h[np.isnan(depth_image_h)] = 0

            color_image_sender_0.send(color_image_h)
            color_image_sender_1.send(color_image_h)
            depth_image_sender.send(depth_image_h)

            counter += 1

        sleep(0.01)

    zed_camera.disable_recording()
    zed_camera.close()


def aruco_position_func(config: Config, color_image_receiver):
    aruco_detector = ArucoDetector(marker_length=config.camera.dozer_marker_length)

    ##############
    # Publishers #
    ##############

    aruco_position_noise_rvs = uniform(loc=config.camera.aruco_position_added_noise_start,
                                       scale=config.camera.aruco_position_added_noise_end -
                                             config.camera.aruco_position_added_noise_start)
    aruco_rotation_noise_rvs = uniform(loc=config.camera.aruco_rotation_added_noise_start,
                                       scale=config.camera.aruco_rotation_added_noise_end -
                                             config.camera.aruco_rotation_added_noise_start)

    position_clean_publisher = Publisher(ip=config.camera.ip, port=config.camera.position_port)
    position_estimated_publisher = Publisher(ip=config.camera.ip,
                                             port=config.camera.estimated_position_port)

    while True:

        curr_color_image = color_image_receiver.recv()

        dozer_pose = calc_dozer_pose(camera_config=config.camera,
                                     aruco_detector=aruco_detector,
                                     color_image_h=curr_color_image)

        if dozer_pose is None:
            print("Aruco Detection Missed")
            continue

        dozer_estimated_pose = dozer_pose.copy()
        estimated_position = dozer_estimated_pose.position + aruco_position_noise_rvs.rvs(3)
        estimated_rotation = dozer_estimated_pose.rotation + aruco_rotation_noise_rvs.rvs(3)
        dozer_estimated_pose.update_position(estimated_position)
        dozer_estimated_pose.update_rotation(estimated_rotation)

        position_clean_publisher.send(config.topics.topic_dozer_position, dozer_pose.to_zmq_str())
        position_estimated_publisher.send(config.topics.topic_dozer_estimated_position,
                                          dozer_estimated_pose.to_zmq_str())


def heightmap_proj_func(zmq_config, camera_config, color_image_receiver, depth_image_receiver):
    intrinsics_d = camera_config.intrinsics_d
    rot_c2w_d = camera_config.rot_c2w_d
    t_w2c_w_d = camera_config.t_w2c_w_d
    lower_bound_d = camera_config.lower_bound_d
    upper_bound_d = camera_config.upper_bound_d

    ##############
    # Publishers #
    ##############

    color_camera_publisher = imagezmq.ImageSender(connect_to=zmq_config.camera_node.color_image_address, REQ_REP=False)
    depth_camera_publisher = imagezmq.ImageSender(connect_to=zmq_config.camera_node.depth_image_address, REQ_REP=False)

    # color_image_h = np.zeros((720, 1280, 3), dtype=np.uint8)

    while True:
        curr_color_image = color_image_receiver.recv()
        curr_depth_image = depth_image_receiver.recv()

        height_map_h, topview_image_h = pcl_utils.rgbd_to_top(curr_color_image,
                                                              curr_depth_image,
                                                              intrinsics_d, rot_c2w_d, t_w2c_w_d,
                                                              lower_bound_d, upper_bound_d,
                                                              camera_config.pixel_density,
                                                              camera_config.grid_width_w,
                                                              camera_config.grid_height_w)

        height_map_final, topview_image_final = crop_bounds(camera_config,
                                                            height_map_h,
                                                            topview_image_h)

        height_map_corrected = height_map_final + camera_config.calibration_altitude_bias

        ##################
        # Publish Images #
        ##################

        jpeg_buffer = simplejpeg.encode_jpeg(topview_image_final, quality=95, colorspace='BGR')

        depth_camera_publisher.send_image(zmq_config.topics.topic_depth_image, height_map_corrected)
        color_camera_publisher.send_jpg(zmq_config.topics.topic_color_image, jpeg_buffer)
