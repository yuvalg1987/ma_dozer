import pathlib
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum
from typing import Union

import numpy as np
import navpy as nav
import yaml
from scipy.stats import rv_frozen, rv_continuous

MotorCommand = Enum('MotorCommand', 'FORWARD BACKWARD ROTATE_LEFT ROTATE_RIGHT')
CompareType = Enum('CompareType', 'TRANSLATION ROTATION ALL')
basic_types = [float, int, str, np.ndarray, list, pathlib.WindowsPath, pathlib.PosixPath,
               bool, type(None), rv_frozen, rv_continuous]


def str2bool(v):
    return v.lower() == "true"


class Position(np.ndarray):

    def __new__(cls, x, y, z):
        return np.asarray([x, y, z]).view(cls)

    @property
    def x(self):
        return self[0].item()

    @property
    def y(self):
        return self[1].item()

    @property
    def z(self):
        return self[2].item()

    @classmethod
    def from_array(cls, position: np.ndarray):
        flatten = position.flatten()
        return cls(flatten[0], flatten[1], flatten[2])

    def update(self, position: np.ndarray):
        assert self.shape == position.shape
        self[:] = position.copy()


class Rotation(np.ndarray):

    def __new__(cls, yaw, pitch, roll):
        return np.asarray([yaw, pitch, roll]).view(cls)

    @property
    def yaw(self):
        return self[0].item()

    @property
    def pitch(self):
        return self[1].item()

    @property
    def roll(self):
        return self[2].item()

    @property
    def euler(self):
        return self

    @property
    def dcm(self):
        rotation_dcm = nav.angle2dcm(self[0], self[1], self[2], input_unit='deg')
        return rotation_dcm

    @classmethod
    def from_array(cls, euler_angles: np.ndarray):
        return cls(euler_angles[0], euler_angles[1], euler_angles[2])

    @classmethod
    def from_dcm(cls, rotation_dcm):
        rotation_euler = nav.dcm2angle(rotation_dcm, output_unit='deg')
        return cls(rotation_euler[0], rotation_euler[1], rotation_euler[2])

    def update_rotation(self, rotation: np.ndarray):
        assert self.shape == rotation.shape
        self[:] = rotation.copy()

    def update_rotation_dcm(self, rotation_dcm: np.ndarray):
        rotation_euler = nav.dcm2angle(rotation_dcm, output_unit='deg')
        self[:] = rotation_euler


class Velocity(np.ndarray):

    def __new__(cls, v_x, v_y, v_z):
        return np.asarray([v_x, v_y, v_z]).view(cls)

    @property
    def v_x(self):
        return self[0].item()

    @property
    def v_y(self):
        return self[1].item()

    @property
    def v_z(self):
        return self[2].item()

    @classmethod
    def from_array(cls, velocity):
        return cls(velocity[0], velocity[1], velocity[2])

    def update_velocity(self, velocity: np.ndarray):
        assert self.shape == velocity.shape
        self[:] = velocity.copy()


class DeltaVelocity(np.ndarray):

    def __new__(cls, dv_x, dv_y, dv_z):
        return np.asarray([dv_x, dv_y, dv_z]).view(cls)

    @property
    def dv_x(self):
        return self[0].item()

    @property
    def dv_y(self):
        return self[1].item()

    @property
    def dv_z(self):
        return self[2].item()

    @classmethod
    def from_array(cls, delta_velocity):
        return cls(delta_velocity[0], delta_velocity[1], delta_velocity[2])

    def update_velocity(self, delta_velocity: np.ndarray):
        assert self.shape == delta_velocity.shape
        self[:] = delta_velocity.copy()


class DeltaTheta(np.ndarray):
    """
    All properties in this class are given in Radians

    """

    def __new__(cls, delta_yaw, delta_pitch, delta_roll):
        return np.asarray([delta_yaw, delta_pitch, delta_roll]).view(cls)

    @property
    def delta_yaw(self):
        return self[0].item()

    @property
    def delta_pitch(self):
        return self[1].item()

    @property
    def delta_roll(self):
        return self[2].item()

    @classmethod
    def from_array(cls, delta_theta: np.ndarray):
        return cls(delta_theta[0], delta_theta[1], delta_theta[2])


class Pose:

    def __init__(self,
                 x: float = 0,
                 y: float = 0,
                 z: float = 0,
                 yaw: float = 0,
                 pitch: float = 0,
                 roll: float = 0,
                 v_x: float = 0,
                 v_y: float = 0,
                 v_z: float = 0,
                 timestamp: int = 0,
                 vehicle_id: int = 3):

        self._position: Position = Position(x, y, z)
        self._rotation: Rotation = Rotation(yaw, pitch, roll)
        self._velocity: Velocity = Velocity(v_x, v_y, v_z)
        self._timestamp: int = timestamp
        self._vehicle_id: int = vehicle_id

    def copy(self):
        return deepcopy(self)

    @classmethod
    def from_arrays(cls,
                    position: np.ndarray,
                    rotation: np.ndarray,
                    velocity: np.ndarray,
                    timestamp: int,
                    vehicle_id: int = 3):

        return cls(x=position[0],
                   y=position[1],
                   z=position[2],
                   yaw=rotation[0],
                   pitch=rotation[1],
                   roll=rotation[2],
                   v_x=velocity[0],
                   v_y=velocity[1],
                   v_z=velocity[2],
                   timestamp=timestamp,
                   vehicle_id=vehicle_id)

    @classmethod
    def from_classes(cls, position: Position,
                     rotation: Rotation,
                     velocity: Velocity,
                     timestamp: int,
                     vehicle_id: int = 3):

        return cls(x=position.x, y=position.y, z=position.z,
                   yaw=rotation.yaw, pitch=rotation.pitch, roll=rotation.roll,
                   v_x=velocity.v_x, v_y=velocity.v_y, v_z=velocity.v_z,
                   timestamp=timestamp, vehicle_id=vehicle_id)

    @property
    def timestamp(self):
        return self._timestamp

    @property
    def position(self):
        return self._position

    @property
    def velocity(self):
        return self._velocity

    @property
    def vehicle_id(self):
        return self._vehicle_id

    @property
    def rotation(self):
        return self._rotation.euler

    # @property
    # def rotation_dcm(self):
    #     return self._rotation.dcm

    def update_pose(self,
                    new_position: Union[np.ndarray, Position],
                    new_rotation: Union[np.ndarray, Rotation],
                    new_velocity: Union[np.ndarray, Velocity],
                    new_timestamp: float):

        self.update_position(new_position)
        self.update_rotation(new_rotation)
        self.update_velocity(new_velocity)
        self.update_timestamp(new_timestamp)

    def update_timestamp(self, new_timestamp: float):
        self._timestamp = new_timestamp

    def update_position(self, new_position: Union[np.ndarray, Position]):
        if isinstance(new_position, Position):
            self._position = deepcopy(new_position)
        elif isinstance(new_position, np.ndarray):
            self._position = Position.from_array(new_position)

    def update_rotation(self, new_rotation: Union[np.ndarray, Rotation]):
        if isinstance(new_rotation, Rotation):
            self._rotation = deepcopy(new_rotation)
        elif isinstance(new_rotation, np.ndarray):
            self._rotation = Rotation.from_array(new_rotation)

    def update_velocity(self, new_velocity: Union[np.ndarray, Velocity]):
        if isinstance(new_velocity, Velocity):
            self._velocity = deepcopy(new_velocity)
        elif isinstance(new_velocity, np.ndarray):
            self._velocity = Velocity.from_array(new_velocity)

    def to_zmq_str(self):
        # return f'{self.vehicle_id}#{self._position.x}#{self._position.y}#{self._position.z}' + \
        #        f'#{self._rotation.yaw}#{self._rotation.pitch}#{self._rotation.roll}' + \
        #        f'#{self.velocity.v_x}#{self.velocity.v_y}#{self.velocity.v_z}#{self.timestamp}' \

        return f'{self.vehicle_id}#{self._position.x}#{self._position.y}#{self._position.z}' + \
               f'#{self._rotation.yaw}#{0.0}#{0.0}' + \
               f'#{self.velocity.v_x}#{self.velocity.v_y}#{self.velocity.v_z}#{self.timestamp}' \


    def __str__(self):
        return f'id = {self.vehicle_id}, ' \
               f'X = {self._position.x:.3f}, Y = {self._position.y:.3f}, Z = {self._position.z:.3f}, ' \
               f'Yaw = {self._rotation.yaw:.3f}, Pitch = {self._rotation.pitch:.3f}, Roll = {self._rotation.roll:.3f}, ' \
               f'v_x = {self.velocity.v_z:.3f}, v_y={self.velocity.v_y:.3f}, v_z={self.velocity.v_z:.3f}, timestamp={self.timestamp}'

    def __eq__(self, other):
        if self._position.x == other._position.x and \
                self._position.y == other._position.y and \
                self._position.z == other._position.z and \
                self._rotation.yaw == other._rotation.yaw and \
                self._rotation.pitch == other._rotation.pitch and \
                self._rotation.roll == other._rotation.roll:

            return True
        else:
            return False

    @classmethod
    def from_zmq_str(cls, pos_str: str):

        id_str, x_str, y_str, z_str, \
        yaw_str, pitch_str, roll_str, \
        v_x_str, v_y_str, v_z_str, timestamp_str = pos_str.split('#')

        return cls(x=float(x_str),
                   y=float(y_str),
                   z=float(z_str),
                   yaw=float(yaw_str),
                   pitch=float(pitch_str),
                   roll=float(roll_str),
                   v_x=float(v_x_str),
                   v_y=float(v_y_str),
                   v_z=float(v_z_str),
                   timestamp=int(timestamp_str),
                   vehicle_id=int(id_str))

    @classmethod
    def from_position(cls,
                      marker_id: int,
                      position: Position,
                      rotation: Rotation,
                      timestamp: int):

        dozer_pose = cls.from_classes(position=position,
                                      rotation=rotation,
                                      velocity=Velocity(0, 0, 0),
                                      timestamp=timestamp, vehicle_id=marker_id)

        return dozer_pose

    @classmethod
    def from_action(cls, action: Action):
        pos = cls(action.position.x,
                  action.position.y,
                  action.position.z,
                  action.rotation.euler.yaw,
                  action.rotation.euler.pitch,
                  action.rotation.euler.roll,
                  v_x=0,
                  v_y=0,
                  v_z=0,
                  timestamp=0,
                  vehicle_id=action.vehicle_id)

        return pos

    def to_log_str(self):
        return ','.join((self.timestamp, self.position.x, self.position.y, self.position.z,
                         self.rotation.yaw, self.rotation.pitch, self.rotation.roll))


class Action:

    def __init__(self,
                 x: float = 0,
                 y: float = 0,
                 z: float = 0,
                 yaw: float = 0,
                 pitch: float = 0,
                 roll: float = 0,
                 forward_movement: bool = True,
                 velocity_gear: int = 1,
                 vehicle_id: int = 3,
                 is_init_action: bool = False,
                 motion_type: MotorCommand = MotorCommand.FORWARD):

        # this is given in meters and distance relative to dozer current location
        self._position: Position = Position(x, y, z)
        # this is given in degrees and is the angle compared to current angle
        self._rotation: Rotation = Rotation(yaw, pitch, roll)
        self._forward_movement: bool = forward_movement
        self.motion_type: MotorCommand = motion_type
        self._velocity_gear: int = velocity_gear
        self.vehicle_id: int = vehicle_id
        self.is_init_action = is_init_action

    @classmethod
    def from_arrays(cls,
                    position: np.ndarray,
                    rotation: np.ndarray,
                    forward_movement: bool,
                    velocity_gear: int = 1,
                    vehicle_id: int = 3):

        position_flat = position.flatten()
        rotation_flat = rotation.flatten()

        return cls(x=position_flat[0],
                   y=position_flat[1],
                   z=position_flat[2],
                   yaw=rotation_flat[0],
                   pitch=rotation_flat[1],
                   roll=rotation_flat[2],
                   forward_movement=forward_movement,
                   velocity_gear=velocity_gear,
                   vehicle_id=vehicle_id)

    @classmethod
    def from_classes(cls, position: Position,
                     rotation: Rotation,
                     forward_movement: bool,
                     velocity_gear: int = 1,
                     vehicle_id: int = 3):

        return cls.from_arrays(position=position,
                               rotation=rotation,
                               forward_movement=forward_movement,
                               velocity_gear=velocity_gear,
                               vehicle_id=vehicle_id)

    @property
    def position(self):
        return self._position

    @property
    def rotation(self):
        return self._rotation

    @property
    def forward_movement(self):
        return self._forward_movement

    @property
    def velocity_gear(self):
        return self._velocity_gear

    def __eq__(self, other):
        # if isinstance(other, Action):
        #     return self.__key() == other.__key()
        # else:
        #     return False
        x_close = np.isclose(self.position.x, other.position.x, atol=1e-3)
        y_close = np.isclose(self.position.y, other.position.y, atol=1e-3)
        z_close = np.isclose(self.position.z, other.position.z, atol=1e-3)
        pitch_close = np.isclose(self.rotation.euler.pitch, other.rotation.euler.pitch, atol=1e-3)
        # yaw_close = np.isclose(self.yaw, other.yaw, atol=1e-3)
        roll_close = np.isclose(self.rotation.euler.roll, other.rotation.euler.roll, atol=1e-3)
        action_close = self.forward_movement == other.forward_movement
        # if x_close and y_close and z_close and pitch_close and yaw_close and roll_close and action_close:
        if x_close and y_close and z_close and pitch_close and roll_close and action_close:
            return True
        else:
            return False

    def __hash__(self):
        return hash((self.position.x,
                     self.position.y,
                     self.position.z,
                     self.rotation.euler.pitch,
                     self.rotation.euler.roll,
                     self.forward_movement))

    def __str__(self):
        return f'id = {self.vehicle_id}, ' + \
               f'X = {self.position.x:.3f}, ' + \
               f'Y = {self.position.y:.3f}, ' + \
               f'Z = {self.position.z:.3f}, ' + \
               f'Yaw = {self.rotation.euler.yaw:.3f}, ' + \
               f'Pitch = {self.rotation.euler.pitch:.3f}, ' + \
               f'Roll = {self.rotation.euler.roll:.3f}'

    def to_zmq_str(self):

        if self.forward_movement:
            action_type = MotorCommand.FORWARD
        else:
            action_type = MotorCommand.BACKWARD

        return f'{self.vehicle_id}#' + \
               f'{self.position.x}#' + \
               f'{self.position.y}#' + \
               f'{self.position.z}#' + \
               f'{self.rotation.euler.yaw}#' + \
               f'{self.rotation.euler.pitch}#' + \
               f'{self.rotation.euler.roll}#' + \
               f'{action_type}#' + \
               f'{self.is_init_action}'

    @classmethod
    def from_zmq_str(cls, pos_str: str):

        id_str, x_str, y_str, z_str, yaw_str, pitch_str, roll_str, action_type_str, is_init_action_str = pos_str.split(
            '#')
        curr_action_type = action_type_str.split('.')[1]

        if curr_action_type == 'FORWARD':
            forward_movement = True
        else:
            forward_movement = False

        return cls(x=float(x_str), y=float(y_str), z=float(z_str),
                   yaw=float(yaw_str), pitch=float(pitch_str), roll=float(roll_str),
                   forward_movement=forward_movement, vehicle_id=int(id_str),
                   is_init_action=str2bool(is_init_action_str))

    @classmethod
    def from_pose(cls, pose: Pose):
        action = cls(pose.position.x,
                     pose.position.y,
                     pose.position.z,
                     pose.rotation.euler.yaw,
                     pose.rotation.euler.pitch,
                     pose.rotation.euler.roll,
                     forward_movement=False,
                     is_init_action=False,
                     vehicle_id=pose.vehicle_id)

        return action

    def __add__(self, other):
        pos_x = self.position.x + other.position.x
        pos_y = self.position.y + other.position.y
        pos_z = self.position.z + other.position.z
        rot_yaw = self.rotation.yaw + other.rotation.yaw
        rot_pitch = self.rotation.pitch + other.rotation.pitch
        rot_roll = self.rotation.roll + other.rotation.roll

        return Action(pos_x, pos_y, pos_z,
                      rot_yaw, rot_pitch, rot_roll,
                      forward_movement=self.forward_movement,
                      vehicle_id=self.vehicle_id,
                      is_init_action=self.is_init_action,
                      motion_type=self.motion_type)


class IMUData:
    def __init__(self,
                 timestamp: int,
                 delta_t: float,
                 delta_velocity: DeltaVelocity,
                 delta_theta: DeltaTheta):
        self.timestamp: int = timestamp
        self.delta_t: float = delta_t
        self.delta_velocity: DeltaVelocity = delta_velocity
        self.delta_theta: DeltaTheta = delta_theta

    def __str__(self):
        return f'timestamp = {self.timestamp}, dt = {self.delta_t}, ' \
               f'dv_x = {self.delta_velocity.dv_x:.3f}, dv_y = {self.delta_velocity.dv_y:.3f}, dv_z = {self.delta_velocity.dv_z:.3f}, ' \
               f'd_yaw = {self.delta_theta.delta_yaw:.3f}, d_pitch = {self.delta_theta.delta_pitch:.3f}, Roll = {self.delta_theta.delta_roll:.3f}, ' \

    @classmethod
    def from_array(cls, timestamp: int,
                   delta_t: float,
                   delta_velocity: DeltaVelocity,
                   delta_theta: DeltaTheta):
        return cls(timestamp=timestamp,
                   delta_velocity=DeltaVelocity(dv_x=delta_velocity.dv_x,
                                                dv_y=delta_velocity.dv_y,
                                                dv_z=delta_velocity.dv_z),
                   delta_theta=DeltaTheta(delta_yaw=delta_theta.delta_yaw,
                                          delta_pitch=delta_theta.delta_pitch,
                                          delta_roll=delta_theta.delta_roll),
                   delta_t=delta_t)

    @classmethod
    def from_zmq_str(cls, measurement_str: str):
        timestamp_str, \
        delta_t_str, \
        delta_velocity_x_str, \
        delta_velocity_y_str, \
        delta_velocity_z_str, \
        delta_theta_yaw_str, \
        delta_theta_pitch_str, \
        delta_theta_roll_str = measurement_str.split('#')

        return cls(timestamp=int(timestamp_str),
                   delta_t=float(delta_t_str),
                   delta_velocity=DeltaVelocity(dv_x=float(delta_velocity_x_str),
                                                dv_y=float(delta_velocity_y_str),
                                                dv_z=float(delta_velocity_z_str)),
                   delta_theta=DeltaTheta(delta_yaw=float(delta_theta_yaw_str),
                                          delta_pitch=float(delta_theta_pitch_str),
                                          delta_roll=float(delta_theta_roll_str)))

    def to_zmq_str(self):
        return f'{self.timestamp}#{self.delta_t}#' \
               f'{self.delta_velocity.dv_x}#{self.delta_velocity.dv_y}#{self.delta_velocity.dv_z}#' \
               f'{self.delta_theta.delta_yaw}#{self.delta_theta.delta_pitch}#{self.delta_theta.delta_roll}'

    def to_log_str(self):
        return ','.join((self.timestamp,
                        self.delta_t,
                        self.delta_velocity.dv_x,
                        self.delta_velocity.dv_y,
                        self.delta_velocity.dv_z,
                        self.delta_theta.delta_yaw,
                        self.delta_theta.delta_pitch,
                        self.delta_theta.delta_roll))


@dataclass
class BaseClass(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader

    @classmethod
    def from_yaml(cls, loader, node):
        values = loader.construct_mapping(node, deep=True)
        return cls(**values)

    @classmethod
    def from_str(cls, yaml_str):
        data = yaml.safe_load(yaml_str)
        return cls(**data)

    @classmethod
    def to_yaml(cls, dumper, data):
        return dumper.represent_yaml_object(cls.yaml_tag, data, cls, flow_style=cls.yaml_flow_style)

    def update(self, other):
        class_keys = self.__dict__.keys()

        for key in class_keys:
            if type(self.__dict__[key]) in basic_types:
                self.__dict__[key] = getattr(other, key)
            else:
                self.__dict__[key].update(getattr(other, key))


class NavState(BaseClass):

    def __init__(self, time, pos, vel, att):
        self.time: float = time
        self.pos: np.ndarray = pos
        self.vel: np.ndarray = vel
        self.att: np.ndarray = att
