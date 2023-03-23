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

        curr_dozer_position = None
        curr_dumper_position = None

        flag_init_dozer_pos = False
        flag_init_dumper_pos = False
        while True:
            socks = dict(self.subscriber_poller.poll(20))  # 20ms
            if socks:
                if (self.position_subscriber.get_socket() in socks and
                        socks[self.position_subscriber.get_socket()] == zmq.POLLIN):
                    curr_topic, curr_data = self.position_subscriber.receive()
                    if curr_topic == self.config.topics.topic_dozer_position and not flag_init_dozer_pos:
                        curr_dozer_position = Pose.from_zmq_str(curr_data)
                        flag_init_dozer_pos = True
                    elif curr_topic == self.config.topics.topic_dumper_position and not flag_init_dumper_pos:
                        curr_dumper_position = Pose.from_zmq_str(curr_data)
                        flag_init_dumper_pos = True

                    if flag_init_dozer_pos and flag_init_dumper_pos:
                        break

        init_dozer_action = Action(x=curr_dozer_position.position.x,
                                   y=curr_dozer_position.position.y,
                                   z=curr_dozer_position.position.z,
                                   yaw=curr_dozer_position.rotation.yaw,
                                   pitch=curr_dozer_position.rotation.pitch,
                                   roll=curr_dozer_position.rotation.roll,
                                   forward_movement=True,
                                   vehicle_id=curr_dozer_position.vehicle_id)

        init_dumper_action = Action(x=curr_dumper_position.position.x,
                                    y=curr_dumper_position.position.y,
                                    z=curr_dumper_position.position.z,
                                    yaw=curr_dumper_position.rotation.yaw,
                                    pitch=curr_dumper_position.rotation.pitch,
                                    roll=curr_dumper_position.rotation.roll,
                                    forward_movement=True,
                                    vehicle_id=curr_dumper_position.vehicle_id)

        flag_init_dozer_ack = False
        flag_init_dumper_ack = False
        while True:

            socks = dict(self.subscriber_poller.poll(20))  # 20ms

            if socks:
                if (self.ack_subscriber.get_socket() in socks and
                        socks[self.ack_subscriber.get_socket()] == zmq.POLLIN):
                    curr_topic, curr_data = self.ack_subscriber.receive()

                    if curr_topic == self.config.topics.topic_dozer_ack_received:
                        curr_action = Action.from_zmq_str(curr_data)
                        print(f'Received ACK for Init action {curr_action}')
                        break

                    else:
                        print(f'Send init action = {init_action}')
                        self.algo_action_publisher.send(self.config.topics.topic_algo_action,
                                                        init_action.to_zmq_str())
                        time.sleep(0.5)

                else:
                    print(f'Send init action = {init_action}')
                    self.algo_action_publisher.send(self.config.topics.topic_algo_action,
                                                    init_action.to_zmq_str())
                    time.sleep(0.5)

            else:
                print(f'Send init action = {init_action}')
                self.algo_action_publisher.send(self.config.topics.topic_algo_action, init_action.to_zmq_str())
                time.sleep(0.5)

    def run(self):
        while True:
            socks = dict(self.subscriber_poller.poll(20))  # 20ms

            if socks:
                if (self.dozer_ack_subscriber.get_socket() in socks and
                        socks[self.dozer_ack_subscriber.get_socket()] == zmq.POLLIN):

                    curr_topic, curr_data = self.dozer_ack_subscriber.receive()
                    curr_algo_action = RealDozerAction.from_zmq_str(curr_data)

                    if curr_topic == self.config.topics.topic_dozer_ack_received:
                        print(f'ACK received from dozer for action {curr_algo_action}')
                        self.ack_received[curr_algo_action] = True

                    if curr_topic == self.config.topics.topic_dozer_ack_finished:
                        print(f'ACK finished from dozer for action = {curr_algo_action}')
                        self.ack_finished[curr_algo_action] = True

                    if curr_topic == self.config.topics.topic_dozer_ack_intermediate_state:
                        print(f'ACK intermediate state from dozer for position = {curr_algo_action}')
                        self.ack_intermediate_state.append(curr_algo_action)

                if (self.dozer_position_subscriber.get_socket() in socks and
                        socks[self.dozer_position_subscriber.get_socket()] == zmq.POLLIN):

                    curr_topic, curr_data = self.dozer_position_subscriber.receive()
                    self.curr_pose = RealDozerPose.from_zmq_str(curr_data)
                    print(f'Current dozer position {self.curr_pose}')

                else:
                    pass
                    # print('Unknown Socket')

            else:
                pass
                # print("error: message timeout")

    def read_pose(self):
        curr_dozer_position_in_map = deepcopy(self.curr_pose)
        # curr_dozer_position_in_map.X += 55 / 2.1
        # curr_dozer_position_in_map.Y += 12 / 2.1

        return curr_dozer_position_in_map

    def check_ack_finished(self, action: RealDozerAction):
        if action in self.ack_finished:
            is_finished = self.ack_finished[action]
            if is_finished:
                self.ack_finished.pop(action)
        else:
            is_finished = False
        return is_finished

    def check_ack_intermediate_state(self):
        if self.ack_intermediate_state:
            intermediate_pose = self.ack_intermediate_state.pop()
            return intermediate_pose
        else:
            return None

    def check_ack_received(self, action: RealDozerAction):
        if action in self.ack_received:
            is_finished = self.ack_received[action]
            if is_finished:
                self.ack_received.pop(action)
        else:
            is_finished = False
        return is_finished

    def send_action(self, action: RealDozerAction):
        self.algo_action_publisher.send(self.config.topics.topic_algo_action, action.to_zmq_str())
