from typing import Callable, Union, Optional
from threading import Thread

import zmq
from typing import List


class Publisher:

    def __init__(self, ip: str = '127.0.0.1', port: int = 1234) -> None:

        self.context: zmq.Context = zmq.Context()
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://%s:%d" % (ip, port))

    def send(self, topic_name: str, message_data) -> None:
        self.pub_socket.send_string(f'{topic_name} {message_data}')


class Subscriber:

    def __init__(self, ip: str = '127.0.0.1', port: int = 1234, topics: List = []) -> None:
        self.context: zmq.Context = zmq.Context()
        self.sub_zmq = self.context.socket(zmq.SUB)
        self.sub_zmq.connect("tcp://%s:%d" % (ip, port))

        for topic in topics:
            self.sub_zmq.setsockopt_string(zmq.SUBSCRIBE, topic)

    def receive(self) -> [str, str]:
        curr_string = self.sub_zmq.recv_string()
        curr_topic, curr_data = curr_string.split()
        return curr_topic, curr_data

    def get_socket(self):
        return self.sub_zmq


class ThreadedSubscriber(Thread):

    def __init__(self,
                 ip: str = '127.0.0.1',
                 port: int = 1234,
                 topics: List = [],
                 callback_func: Optional[Callable[[str, str], None]] = None):

        super().__init__()

        self.context: zmq.Context = zmq.Context()
        self.sub_zmq = self.context.socket(zmq.SUB)
        self.sub_zmq.connect("tcp://%s:%d" % (ip, port))

        for topic in topics:
            self.sub_zmq.setsockopt_string(zmq.SUBSCRIBE, topic)

        self.callback_func = callback_func

    def receive(self) -> [str, str]:
        curr_string = self.sub_zmq.recv_string()
        curr_topic, curr_data = curr_string.split()
        return curr_topic, curr_data

    def get_socket(self):
        return self.sub_zmq

    def run(self) -> None:

        while True:
            curr_topic, curr_data = self.receive()
            self.callback_func(curr_topic, curr_data)



