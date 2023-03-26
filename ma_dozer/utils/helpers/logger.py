import os
import time


def init_exp_folder():

    timestamp = time.strftime("%m%d_%H%M%S")
    datestamp = f"exp_{timestamp.split('_')[0][2:]}_{timestamp.split('_')[0][:2]}"
    folder_location = os.path.abspath(f'../data/{datestamp}')
    svo_file_location = os.path.abspath(f'{folder_location}/exp_{timestamp}.svo')
    course_log_file = os.path.abspath(f'{folder_location}/exp_{timestamp}.txt')

    if not (os.path.exists(folder_location)):
        os.makedirs(folder_location)

    return folder_location, svo_file_location, course_log_file


class Logger:

    def __init__(self):
        super().__init__()

        self.folder_location, \
        self.planner_log_location, \
        self.controller_log_location, \
        self.svo_file_location = init_exp_folder() # TODO fix

        self.planner_log_file = open(self.planner_log_location, "wb")
        self.controller_log_file = open(self.controller_log_location, "wb")

    def log_course(self, curr_course, curr_pose, target_action):

        self.planner_log_file.write(f'curr_pose = {curr_pose}'.encode() + '\n'.encode() )
        self.planner_log_file.write(f'target_action = {target_action}'.encode() + '\n'.encode())

        self.planner_log_file.write('current_course:'.encode() + '\n'.encode())

        for curr_pose_tmp in curr_course:
            self.planner_log_file.write(
                f'curr_pose = {curr_pose_tmp[0]}'.encode() + f' curr_action = {curr_pose_tmp[1]}'.encode() + '\n'.encode())

        self.planner_log_file.write('end of course:'.encode() + '\n\n'.encode())
        return

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

