from enum import Enum
import numpy as np
import matplotlib.pyplot as plt


def add_str_action(action_file, id_agent, pose_x, pose_y, yaw_ang, motor_command, init_command):
    id_str = str(id_agent)
    x_str = str(round(pose_x, 4))
    y_str = str(round(pose_y, 4))
    z_str = '0'
    yaw_str = str(round(np.degrees(yaw_ang), 4))
    pitch_str = '0'
    roll_str = '0'
    action_type_str = str(motor_command)
    is_init_action = 'True' if init_command == 0 else 'False'

    action_str = '#'.join((id_str, x_str, y_str, z_str, yaw_str, pitch_str, roll_str, action_type_str, is_init_action))
    action_file.write(action_str + '\n')


def main():
    traj_action_file = open('../../scripts/dozer/1_actions.txt', 'w')
    agent_id = 3

    MotorCommand = Enum('MotorCommand', 'FORWARD BACKWARD ROTATE_LEFT ROTATE_RIGHT')

    max_distance_per_step = 0.05
    max_rot_per_step = 5
    eps = 1e-4

    commands = [[MotorCommand.FORWARD, 0.5],
                [MotorCommand.ROTATE_LEFT, 30],
                [MotorCommand.FORWARD, 0.5],
                [MotorCommand.ROTATE_LEFT, 30],
                [MotorCommand.FORWARD, 0.5],
                [MotorCommand.ROTATE_LEFT, 30],
                [MotorCommand.FORWARD, 0.5],
                [MotorCommand.BACKWARD, 1]]

    anchor_coord = np.array([[0, 0]])
    yaw = 0
    add_str_action(traj_action_file, agent_id, anchor_coord[0, 0], anchor_coord[0, 1], yaw, MotorCommand.FORWARD, 0)
    i = 1
    for command in commands:
        if command[0] == MotorCommand.FORWARD:
            end_pose_x = anchor_coord[i - 1, 0] + command[1] * np.cos(yaw)
            end_pose_y = anchor_coord[i - 1, 1] + command[1] * np.sin(yaw)
            while np.abs(anchor_coord[-1, 0] - end_pose_x) > eps or np.abs(anchor_coord[-1, 1] - end_pose_y) > eps:
                anchor_coord = np.append(anchor_coord, [[anchor_coord[i - 1, 0] + max_distance_per_step * np.cos(yaw),
                                                         anchor_coord[i - 1, 1] + max_distance_per_step * np.sin(yaw)]],
                                         axis=0)
                add_str_action(traj_action_file, agent_id, anchor_coord[i, 0], anchor_coord[i, 1], yaw, command[0], i)
                i += 1
        elif command[0] == MotorCommand.BACKWARD:
            end_pose_x = anchor_coord[i - 1, 0] - command[1] * np.cos(yaw)
            end_pose_y = anchor_coord[i - 1, 1] - command[1] * np.sin(yaw)
            while np.abs(anchor_coord[-1, 0] - end_pose_x) > eps or np.abs(anchor_coord[-1, 1] - end_pose_y) > eps:
                anchor_coord = np.append(anchor_coord, [[anchor_coord[i - 1, 0] - max_distance_per_step * np.cos(yaw),
                                                         anchor_coord[i - 1, 1] - max_distance_per_step * np.sin(yaw)]],
                                         axis=0)
                add_str_action(traj_action_file, agent_id, anchor_coord[i, 0], anchor_coord[i, 1], yaw, command[0], i)
                i += 1
        elif command[0] == MotorCommand.ROTATE_LEFT:
            end_yaw = yaw + np.radians(command[1])
            while np.abs(yaw - end_yaw) > eps:
                yaw = yaw + np.radians(max_rot_per_step)
                add_str_action(traj_action_file, agent_id, anchor_coord[i-1, 0], anchor_coord[i-1, 1], yaw, command[0], i)
        elif command[0] == MotorCommand.ROTATE_RIGHT:
            end_yaw = yaw + np.radians(command[1])
            while np.abs(yaw - end_yaw) > eps:
                yaw = yaw + np.radians(-max_rot_per_step)
                add_str_action(traj_action_file, agent_id, anchor_coord[i-1, 0], anchor_coord[i-1, 1], yaw, command[0], i)

    traj_action_file.close()
    plt.figure()
    plt.plot(anchor_coord[:, 0], anchor_coord[:, 1])
    plt.axis('equal')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
