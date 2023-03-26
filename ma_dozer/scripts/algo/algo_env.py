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

        while not self.algo_messaging_thread.check_ack_received(action, self.name):
            time.sleep(0.005)

        print(f"real env received acknowledgment from dozer for action:{action}")
        time_start = time.time()

        while not self.algo_messaging_thread.check_ack_finished(action, self.name):
            time.sleep(0.005)

        print(f"real env received finished from dozer for action:{action}")

        time_end = time.time()
        action_time = time_end - time_start

        is_done = self.is_done()

        return is_done

    def run(self):
        is_done = False

        # get actions from files
        self.action = Action.from_zmq_str(self.action_file.readline())
        self.action = self.action + self.init_pose
        while not is_done:
            print("sending next action")

            # run action
            is_done = self.step(self.action)

    def is_done(self):
        next_action = self.action_file.readline()
        if next_action == '':
            return True
        else:
            self.action = Action.from_zmq_str(next_action)
        return False
