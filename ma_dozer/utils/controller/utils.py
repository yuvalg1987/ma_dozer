import math

from typing import Union
import numpy as np

from ma_dozer.configs.controller_config import ControllerConfig
from ma_dozer.utils.helpers.classes import Pose, Action, MotorCommand, CompareType

print_control_counter: int = 0


def calc_pose_distance(curr_pose: Union[Pose, Action],
                       target_pose: Union[Pose, Action]):
    delta_xyz = np.linalg.norm(np.array([curr_pose.position.x - target_pose.position.x,
                                         curr_pose.position.y - target_pose.position.y]))

    rotation_dist_mat = curr_pose.rotation.dcm @ target_pose.rotation.dcm.T * 180 / np.pi
    delta_yaw = abs(rotation_dist_mat[0, 1])
    # delta_pitch = abs(rotation_dist_mat[0, 2])
    # delta_roll = abs(rotation_dist_mat[1, 2])

    return delta_xyz, delta_yaw


def epsilon_close_control(controller_config: ControllerConfig(),
                          curr_pose: Union[Pose, Action],
                          target_pose: Union[Pose, Action],
                          motor_command: MotorCommand):
    global print_control_counter

    delta_xyz, delta_yaw = calc_pose_distance(curr_pose, target_pose)

    if motor_command == MotorCommand.FORWARD or motor_command == MotorCommand.BACKWARD:

        res = (delta_xyz < controller_config.eps_delta_translation)
        if print_control_counter % controller_config.print_control_mod == 0:
            print(f'delta_translation = {delta_xyz}')
            print_control_counter += 1

    elif motor_command == MotorCommand.ROTATE_LEFT or motor_command == MotorCommand.ROTATE_RIGHT:
        res = (delta_yaw < controller_config.eps_delta_yaw)
        if print_control_counter % controller_config.print_control_mod == 0:
            print(f'delta_yaw = {delta_yaw}')
            print_control_counter += 1

    else:
        res = False

    return res, delta_xyz, delta_yaw


def epsilon_close_plan(controller_config: ControllerConfig(),
                       curr_pose: Union[Pose, Action],
                       target_pose: Union[Pose, Action],
                       motion_type: CompareType):
    delta_xyz, delta_yaw = calc_pose_distance(curr_pose, target_pose)

    if motion_type == CompareType.TRANSLATION:
        res = (delta_xyz < controller_config.eps_delta_translation * controller_config.eps_delta_planner_xyz)
        print(f'delta_translation = {delta_xyz}')

    elif motion_type == CompareType.ROTATION:
        res = (delta_yaw < controller_config.eps_delta_yaw * controller_config.eps_delta_planner_pqr)
        print(f'delta_yaw = {delta_yaw}')

    else:
        res = (delta_xyz < controller_config.eps_delta_translation * controller_config.eps_delta_planner_xyz) and \
              (delta_yaw < controller_config.eps_delta_yaw * controller_config.eps_delta_planner_pqr)

    return res


def check_turn_direction(rotation_angle: float):
    if rotation_angle >= 0:
        return MotorCommand.ROTATE_LEFT
    else:
        return MotorCommand.ROTATE_RIGHT


def calc_path_angle(start_pose: Union[Pose, Action],
                    end_pose: Action):
    """
    :param motion_direction:
    :param start_pose:
    :param end_pose:
    :return angle direction in degrees:
    """
    path_yaw = np.arctan2(end_pose.position.y - start_pose.position.y,
                          end_pose.position.x - start_pose.position.x) * 180 / np.pi

    if end_pose.forward_movement:
        delta_yaw = path_yaw - start_pose.rotation.yaw
    else:
        delta_yaw = path_yaw - (start_pose.rotation.yaw - 180)

    delta_yaw = np.arctan2(np.sin(delta_yaw / 180 * np.pi), np.cos(delta_yaw / 180 * np.pi)) * 180 / np.pi

    return delta_yaw


def estimate_rotation_sequence(controller_config: ControllerConfig(),
                               start_pose: Pose,
                               end_pose: Action):
    curr_sequence = []

    if epsilon_close_plan(controller_config, start_pose, end_pose, CompareType.TRANSLATION):
        delta_yaw = end_pose.rotation.yaw - start_pose.rotation.yaw
    else:
        delta_yaw = calc_path_angle(start_pose, end_pose)

    if abs(delta_yaw) >= controller_config.eps_delta_yaw * controller_config.eps_delta_planner_pqr:

        rotation_steps = math.ceil(abs(delta_yaw) / controller_config.max_angle_per_step) + 1
        next_motor_command = check_turn_direction(delta_yaw)

        for idx in range(0, rotation_steps - 1):

            if next_motor_command == MotorCommand.ROTATE_LEFT:
                curr_yaw_increment = min(delta_yaw, (idx + 1) * controller_config.max_angle_per_step)
            elif next_motor_command == MotorCommand.ROTATE_RIGHT:
                curr_yaw_increment = max(delta_yaw, - (idx + 1) * controller_config.max_angle_per_step)
            else:  # should not get to here !!
                curr_yaw_increment = 0

            next_yaw = start_pose.rotation.yaw + curr_yaw_increment

            next_pose = Pose(x=start_pose.position.x,
                             y=start_pose.position.y,
                             z=start_pose.position.z,
                             yaw=next_yaw,
                             pitch=end_pose.rotation.pitch,
                             roll=end_pose.rotation.roll,
                             v_x=0, v_y=0, v_z=0, timestamp=0,
                             vehicle_id=start_pose.vehicle_id)

            pose_tuple = (next_pose, next_motor_command)
            curr_sequence.append(pose_tuple)

    return curr_sequence


def estimate_translation_sequence(controller_config: ControllerConfig(),
                                  start_pose: Union[Pose, Action],
                                  end_pose: Action):
    if epsilon_close_plan(controller_config, start_pose, end_pose, CompareType.TRANSLATION):
        return

    delta_xyz, _ = calc_pose_distance(start_pose, end_pose)

    curr_sequence = []
    distance_steps = math.floor(delta_xyz / controller_config.max_distance_per_step) + 1

    if end_pose.forward_movement:
        curr_yaw = start_pose.rotation.yaw
        motor_command = MotorCommand.FORWARD
    else:
        curr_yaw = start_pose.rotation.yaw - 180
        motor_command = MotorCommand.BACKWARD

    for idx in range(0, distance_steps - 1):
        curr_x_step = start_pose.position.x + (idx + 1) * controller_config.max_distance_per_step * math.cos(
            curr_yaw / 180 * np.pi)
        curr_y_step = start_pose.position.y + (idx + 1) * controller_config.max_distance_per_step * math.sin(
            curr_yaw / 180 * np.pi)

        curr_pose = Pose(x=curr_x_step,
                         y=curr_y_step,
                         z=start_pose.position.z,
                         yaw=start_pose.rotation.yaw,
                         pitch=start_pose.rotation.pitch,
                         roll=start_pose.rotation.roll,
                         v_x=0, v_y=0, v_z=0, timestamp=0,
                         vehicle_id=start_pose.vehicle_id)

        pose_tuple = (curr_pose, motor_command)
        curr_sequence.append(pose_tuple)

    final_pose = Pose(x=end_pose.position.x,
                      y=end_pose.position.y,
                      z=end_pose.position.z,
                      yaw=start_pose.rotation.yaw,
                      pitch=start_pose.rotation.pitch,
                      roll=start_pose.rotation.roll,
                      v_x=0, v_y=0, v_z=0, timestamp=0,
                      vehicle_id=start_pose.vehicle_id)

    pose_tuple = (final_pose, motor_command)
    curr_sequence.append(pose_tuple)

    return curr_sequence
