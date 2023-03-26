from abc import abstractmethod

from ma_dozer.scripts.roboclaw.roboclaw_3 import Roboclaw


class RoboclawBaseWrapper:

    @abstractmethod
    def __init__(self):
        # print('Init Roboclaw Debugger')
        pass

    @abstractmethod
    def forward(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Forward')
        pass

    @abstractmethod
    def backward(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Backward')
        pass

    @abstractmethod
    def rotate_right(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Rotate Right')
        pass

    @abstractmethod
    def rotate_left(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Rotate Left')
        pass

    @abstractmethod
    def stop(self):
        # print('Motor Command Stop')
        pass


class RoboclawDebugWrapper(RoboclawBaseWrapper):

    def __init__(self):
        # print('Init Roboclaw Debugger')
        pass

    def forward(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Forward')
        pass

    def backward(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Backward')
        pass

    def rotate_right(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Rotate Right')
        pass

    def rotate_left(self, speed: int = 30): # Max speed is 128
        # print('Motor Command Rotate Left')
        pass

    def stop(self):
        # print('Motor Command Stop')
        pass


class RoboclawWrapper(RoboclawBaseWrapper):

    def __init__(self):
        self.address: int = 0x80
        self.roboclaw: Roboclaw = Roboclaw("/dev/ttyS0", 38400)
        self.roboclaw.Open()

    def forward(self, speed: int = 50): # Max speed is 128
        self.roboclaw.ForwardM1(self.address, speed)
        self.roboclaw.ForwardM2(self.address, speed)

    def backward(self, speed: int = 50): # Max speed is 128
        self.roboclaw.BackwardM1(self.address, speed)
        self.roboclaw.BackwardM2(self.address, speed)

    def rotate_right(self, speed: int = 30): # Max speed is 128
        self.roboclaw.BackwardM1(self.address, speed)
        self.roboclaw.ForwardM2(self.address, speed)

    def rotate_left(self, speed: int = 30): # Max speed is 128
        self.roboclaw.ForwardM1(self.address, speed)
        self.roboclaw.BackwardM2(self.address, speed)

    def stop(self):
        self.roboclaw.ForwardM1(self.address, 0)
        self.roboclaw.ForwardM2(self.address, 0)
