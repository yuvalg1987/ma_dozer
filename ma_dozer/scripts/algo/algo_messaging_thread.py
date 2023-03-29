import time
from copy import deepcopy
from threading import Thread

import zmq

from ma_dozer.configs.config import Config
from ma_dozer.utils.helpers.classes import Pose, Action
from ma_dozer.utils.zmq.infrastructure import Publisher, Subscriber


class AlgoMessagingThread(Thread):

    def __init__(self, config: Config):

        super(AlgoMessagingThread, self).__init__()

        self.config = config

        self.dozer_curr_pose = None
        self.dozer_curr_action = None

        self.dozer_ack_finished = {}
        self.dozer_ack_intermediate_state = []
        self.dozer_ack_received = {}

        self.dumper_curr_pose = None
        self.dumper_curr_action = None

        self.dumper_ack_finished = {}
        self.dumper_ack_intermediate_state = []
        self.dumper_ack_received = {}

        # publisher, algo -> dozer / dumper send action
        self.algo_action_publisher = Publisher(ip=config.algo.ip, port=config.algo.action_port)

        # subscriber, dozer / dumper -> algo, dozer received action, dozer finished action
        self.ack_subscriber = Subscriber(ip=config.dozer.ip,
                                         port=config.dozer.ack_port,
                                         topics=[config.topics.topic_dozer_ack_received,
                                                 config.topics.topic_dozer_ack_finished,
                                                 config.topics.topic_dumper_ack_received,
                                                 config.topics.topic_dumper_ack_finished])

        # subscriber, camera -> algo, dozer current position
        self.position_subscriber = Subscriber(ip=config.camera.ip,
                                              port=config.camera.position_port,
                                              topics=[config.topics.topic_dozer_position,
                                                      config.topics.topic_dumper_position])

        self.subscriber_poller = zmq.Poller()
        self.subscriber_poller.register(self.ack_subscriber.get_socket(), zmq.POLLIN)
        self.subscriber_poller.register(self.position_subscriber.get_socket(), zmq.POLLIN)

        self.init_connection()

    def init_connection(self):

        curr_dozer_action = None

        flag_init_dozer_pos = False
        flag_init_dumper_pos = False
        while True:
            socks = dict(self.subscriber_poller.poll(20))  # 20ms
            if socks:
                if (self.position_subscriber.get_socket() in socks and
                        socks[self.position_subscriber.get_socket()] == zmq.POLLIN):
                    curr_topic, curr_data = self.position_subscriber.receive()
                    if curr_topic == self.config.topics.topic_dozer_position and not flag_init_dozer_pos:
                        self.dozer_curr_pose = Pose.from_zmq_str(curr_data)
                        flag_init_dozer_pos = True
                    elif curr_topic == self.config.topics.topic_dumper_position and not flag_init_dumper_pos:
                        self.dumper_curr_pose = Pose.from_zmq_str(curr_data)
                        flag_init_dumper_pos = True

                    if flag_init_dozer_pos and flag_init_dumper_pos:
                        break

        init_dozer_action = Action(x=self.dozer_curr_pose.position.x,
                                   y=self.dozer_curr_pose.position.y,
                                   z=self.dozer_curr_pose.position.z,
                                   yaw=self.dozer_curr_pose.rotation.yaw,
                                   pitch=self.dozer_curr_pose.rotation.pitch,
                                   roll=self.dozer_curr_pose.rotation.roll,
                                   forward_movement=True,
                                   vehicle_id=self.dozer_curr_pose.vehicle_id)

        init_dumper_action = Action(x=self.dumper_curr_pose.position.x,
                                    y=self.dumper_curr_pose.position.y,
                                    z=self.dumper_curr_pose.position.z,
                                    yaw=self.dumper_curr_pose.rotation.yaw,
                                    pitch=self.dumper_curr_pose.rotation.pitch,
                                    roll=self.dumper_curr_pose.rotation.roll,
                                    forward_movement=True,
                                    vehicle_id=self.dumper_curr_pose.vehicle_id)

        flag_init_dozer_ack = False
        flag_init_dumper_ack = False

        dumper_attempts = 10
        while True:

            socks = dict(self.subscriber_poller.poll(20))  # 20ms

            if socks:
                if (self.ack_subscriber.get_socket() in socks and
                        socks[self.ack_subscriber.get_socket()] == zmq.POLLIN):
                    curr_topic, curr_data = self.ack_subscriber.receive()

                    if curr_topic == self.config.topics.topic_dozer_ack_received and not flag_init_dozer_ack:
                        curr_dozer_action = Action.from_zmq_str(curr_data)
                        print(f'Received ACK for Init dozer action {curr_dozer_action}')
                        flag_init_dozer_ack = True
                    elif curr_topic == self.config.topics.topic_dumper_ack_received and not flag_init_dumper_ack:
                        curr_dumper_action = Action.from_zmq_str(curr_data)
                        print(f'Received ACK for Init dumper action {curr_dumper_action}')
                        flag_init_dumper_ack = True

                    if flag_init_dozer_ack and flag_init_dumper_ack:
                        break

                    if not flag_init_dozer_ack:
                        print(f'Send init dozer action = {init_dozer_action}')
                        self.algo_action_publisher.send(self.config.topics.topic_algo_dozer_action,
                                                        init_dozer_action.to_zmq_str())
                        time.sleep(0.5)
                    if not flag_init_dumper_ack and dumper_attempts > 0:
                        print(f'Send init dozer action = {init_dumper_action}')
                        self.algo_action_publisher.send(self.config.topics.topic_algo_dumper_action,
                                                        init_dumper_action.to_zmq_str())
                        time.sleep(0.5)
                        dumper_attempts -= 1

                        if dumper_attempts == 0:
                            flag_init_dumper_ack = True

                else:
                    print(f'Send init dozer action = {init_dozer_action}')
                    self.algo_action_publisher.send(self.config.topics.topic_algo_dozer_action,
                                                    init_dozer_action.to_zmq_str())
                    time.sleep(0.5)

                    if dumper_attempts > 0:
                        print(f'Send init dumper action = {init_dumper_action}')
                        self.algo_action_publisher.send(self.config.topics.topic_algo_dumper_action,
                                                        init_dumper_action.to_zmq_str())
                        time.sleep(0.5)
                        dumper_attempts -= 1
                        if dumper_attempts == 0:
                            flag_init_dumper_ack = True

            else:
                print(f'Send init dozer action = {init_dozer_action}')
                self.algo_action_publisher.send(self.config.topics.topic_algo_dozer_action,
                                                init_dozer_action.to_zmq_str())
                time.sleep(0.5)

                if dumper_attempts > 0:
                    print(f'Send init dumper action = {init_dumper_action}')
                    self.algo_action_publisher.send(self.config.topics.topic_algo_dumper_action,
                                                    init_dumper_action.to_zmq_str())
                    time.sleep(0.5)
                    dumper_attempts -= 1
                    if dumper_attempts == 0:
                        flag_init_dumper_ack = True

    def run(self):
        while True:
            socks = dict(self.subscriber_poller.poll(20))  # 20ms

            if socks:
                if (self.ack_subscriber.get_socket() in socks and
                        socks[self.ack_subscriber.get_socket()] == zmq.POLLIN):

                    curr_topic, curr_data = self.ack_subscriber.receive()
                    curr_algo_action = Action.from_zmq_str(curr_data)

                    if curr_topic == self.config.topics.topic_dozer_ack_received:
                        print(f'ACK received from dozer for action {curr_algo_action}')
                        self.dozer_ack_received[curr_algo_action] = True

                    if curr_topic == self.config.topics.topic_dozer_ack_finished:
                        print(f'ACK finished from dozer for action = {curr_algo_action}')
                        self.dozer_ack_finished[curr_algo_action] = True

                    if curr_topic == self.config.topics.topic_dozer_ack_intermediate_state:
                        print(f'ACK intermediate state from dozer for position = {curr_algo_action}')
                        self.dozer_ack_intermediate_state.append(curr_algo_action)

                    if curr_topic == self.config.topics.topic_dumper_ack_received:
                        print(f'ACK received from dumper for action {curr_algo_action}')
                        self.dumper_ack_received[curr_algo_action] = True

                    if curr_topic == self.config.topics.topic_dumper_ack_finished:
                        print(f'ACK finished from dumper for action = {curr_algo_action}')
                        self.dumper_ack_finished[curr_algo_action] = True

                    if curr_topic == self.config.topics.topic_dumper_ack_intermediate_state:
                        print(f'ACK intermediate state from dumper for position = {curr_algo_action}')
                        self.dumper_ack_intermediate_state.append(curr_algo_action)

                if (self.position_subscriber.get_socket() in socks and
                        socks[self.position_subscriber.get_socket()] == zmq.POLLIN):

                    curr_topic, curr_data = self.position_subscriber.receive()
                    if curr_topic == self.config.topics.topic_dozer_position:
                        self.dozer_curr_pose = Pose.from_zmq_str(curr_data)
                        print(f'Current dozer position {self.dozer_curr_pose}')
                    elif curr_topic == self.config.topics.topic_dumper_position:
                        self.dumper_curr_pose = Pose.from_zmq_str(curr_data)
                        print(f'Current dumper position {self.dumper_curr_pose}')

                else:
                    pass
                    # print('Unknown Socket')

            else:
                pass
                # print("error: message timeout")

    def read_pose(self, node: str):
        if node == self.config.dozer.name:
            curr_position_in_map = deepcopy(self.dozer_curr_pose)
        elif node == self.config.dumper.name:
            curr_position_in_map = deepcopy(self.dumper_curr_pose)
        else:
            curr_position_in_map = None

        return curr_position_in_map

    def check_ack_finished(self, action: Action, node: str):
        is_finished = False
        if node == self.config.dozer.name:
            if action in self.dozer_ack_finished:
                is_finished = self.dozer_ack_finished[action]
                if is_finished:
                    self.dozer_ack_finished.pop(action)
        elif node == self.config.dumper.name:
            if action in self.dumper_ack_finished:
                is_finished = self.dumper_ack_finished[action]
                if is_finished:
                    self.dumper_ack_finished.pop(action)

        return is_finished

    def check_ack_intermediate_state(self, node: str):
        if node == self.config.dozer.name:
            if self.dozer_ack_intermediate_state:
                intermediate_pose = self.dozer_ack_intermediate_state.pop()
                return intermediate_pose
        if node == self.config.dumper.name:
            if self.dumper_ack_intermediate_state:
                intermediate_pose = self.dumper_ack_intermediate_state.pop()
                return intermediate_pose
        else:
            return None

    def check_ack_received(self, action: Action, node: str):
        is_finished = False

        if node == self.config.dozer.name:
            if action in self.dozer_ack_received:
                is_finished = self.dozer_ack_received[action]
                if is_finished:
                    self.dozer_ack_received.pop(action)
        elif node == self.config.dumper.name:
            if action in self.dumper_ack_received:
                is_finished = self.dumper_ack_received[action]
                if is_finished:
                    self.dumper_ack_received.pop(action)

        return is_finished

    def send_action(self, action: Action, node: str):
        if node == self.config.dozer.name:
            self.algo_action_publisher.send(self.config.topics.topic_algo_dozer_action, action.to_zmq_str())
        elif node == self.config.dumper.name:
            self.algo_action_publisher.send(self.config.topics.topic_algo_dumper_action, action.to_zmq_str())
