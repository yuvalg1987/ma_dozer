import sys
import time
from threading import Thread
from typing import Optional

from ma_dozer.configs.config import Config
from ma_dozer.scripts.algo.algo_messaging_thread import AlgoMessagingThread
from ma_dozer.utils.helpers.classes import Action, Pose


class Env(Thread):
    def __init__(self,
                 config: Config,
                 algo_messaging_thread: AlgoMessagingThread = None,
                 name: str = None,
                 init_pose: Pose = None):

        super(Env, self).__init__()

        self.config = config
        self.algo_messaging_thread = algo_messaging_thread
        self.name = name
        self.init_pose = init_pose

        if self.name == self.config.dozer.name:
            self.action_file = open(self.config.dozer.action_file_path, 'r')
        elif self.name == self.config.dumper.name:
            self.action_file = open(self.config.dumper.action_file_path, 'r')
        self.action = None

    def step(self, action: Optional[Action]):

        self.algo_messaging_thread.send_action(action, self.name)
        print(f'{self.name.upper()} - Action was sent: {action}')

        while not self.algo_messaging_thread.check_ack_received(action, self.name):
            time.sleep(0.005)

        print(f"action removed from {self.name.upper()} received dictionary for action:{action}")
        time_start = time.time()

        while not self.algo_messaging_thread.check_ack_finished(action, self.name):
            time.sleep(0.005)

        print(f"action removed from {self.name.upper()} finished dictionary for action:{action}")

        time_end = time.time()
        action_time = time_end - time_start

        is_done = self.is_done()

        return is_done

    def run(self):
        if self.init_pose is not None:
            is_done = False

            # get actions from files
            self.action = Action.from_zmq_str(self.action_file.readline())
            self.action = self.action + self.init_pose
            print(f'{self.name.upper()} Send start Action: {self.action}')
            while not is_done:
                print(f"    {self.name.upper()}: sending next action {self.action}")

                # run action
                is_done = self.step(self.action)

            sys.exit(0)
        else:
            print(f'{self.name.upper()}: init position is None')

    def is_done(self):
        next_action = self.action_file.readline()
        if next_action == '':
            print(f'{self.name} is finished its trajectory')
            return True
        else:
            self.action = Action.from_zmq_str(next_action)
            self.action = self.action + self.init_pose
        return False
