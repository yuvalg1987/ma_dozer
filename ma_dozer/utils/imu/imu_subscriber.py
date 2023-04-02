import numpy as np
import serial
import struct
import threading
import time

from array import array
from typing import Optional, Union

from ma_dozer.configs.nodes_config import DozerNode, DumperNode
from ma_dozer.utils.helpers.classes import DeltaVelocity, DeltaTheta, IMUData

"""
To configure the IMU to collect delta_time, delta_velocity and delta_theta use the following line ins the terminal.

$VNASY,0*XX                 // stop async message printing
$VNWRG,06,0*XX              // stop ASCII message outputs
$VNWRG,75,1,8,01,0800*XX    // output binary message (see below for details)
$VNCMD*XX                   // enter command mode
system save                 // save settings to flash memory
exit                        // exit command mode
$VNASY,1*XX                 // resume async message printing

important stuff:
    - set the binary serial number to 1
    - check the number of bytes the message contains and update the variable self._len_payload
      to be (num bytes - 1)

"""


class IMUSubscriber(threading.Thread):

    def __init__(self,
                 imu_config: Union[DozerNode, DumperNode],
                 callback_func):

        threading.Thread.__init__(self)

        self._lock = threading.Lock()

        self._on: bool = True
        self._port: str = imu_config.imu_port
        self._baud: int = imu_config.imu_baud_rate
        self._imu_rate: int = 100  # if changed it needs to also be changed in the IMU registers
        self._timestamp: int = 0
        self._delta_t: float = 0.0
        self._delta_velocity: Optional[DeltaVelocity] = None
        self._delta_theta: Optional[DeltaTheta] = None
        self._curr_measurement: Optional[IMUData] = None

        # You should update this to match your configuration.
        # should be the (number of bytes stated in vector nav) - 1
        self._len_payload = 33

        self.callback_func = callback_func
        print('IMU: initialized')

    def init_connection(self):
        print('IMU: reading from {} at {}'.format(self._port, self._baud))

        # In case the port is not properly closed,
        try:
            temp = serial.Serial(self._port, self._baud)
            temp.close()
        except:
            print('\033[91m' + 'Unable to open IMU port at ' + self._port
                  + ':' + str(self._baud) + '\033[0m')
            return

    def run(self):
        """
        Start the thread.
        """

        self.init_connection()

        # Open the serial port and start reading.
        with serial.Serial(self._port, self._baud, timeout=1) as s:

            # Clear the buffer first.
            print('IMU: clearing buffer')
            num_bytes = s.in_waiting
            s.read(num_bytes)

            print('IMU: starting main loop')

            self._timestamp = time.time_ns()
            while self._on:

                # Check if there are bytes waiting in the buffer.
                num_bytes = s.in_waiting
                if num_bytes == 0:
                    time.sleep(0.005)  # Reduce/delete this sleep time if you are reading data at a faster rate.
                    continue

                # IMU sends 0xFA (int 250) as the first byte. This marks the begining of the message.
                imu_sync_detected = self.check_sync_byte(s)
                if not imu_sync_detected:
                    continue

                # If the sync byte us detected, read the rest of the message.
                success = self.read_imu_data(s)

                self.callback_func('imu_measurement', self._curr_measurement.to_zmq_str())

                if not success:
                    continue

        print('IMU: thread closed')

    @staticmethod
    def check_sync_byte(s):
        """
        Check if the sync byte is detected.

        IMU sends 0xFA (int 250) as the first byte. This marks the beginning of
        the message.

        Args:
        s: (serial object) - Already open serial port of the IMU.

        Return:
        bool - True if the sync byte is detected in the current buffer.
        """

        # Iterate over all the bytes in the current buffer.
        for _ in range(s.in_waiting):
            byte_in = s.read(1)

            # Check if the sync byte 0xFA (int 250) is detected.
            int_in = int.from_bytes(byte_in, 'little')
            if int_in == 250:
                return True

        return False

    def read_imu_data(self, s):
        """
        Read and parse the payload of the IMU message.

        Args:
        s: (serial object) - Already open serial port of the IMU.

        Return:
        bool - True if the operation is successful
        """

        # Read data.
        N = self._len_payload
        data = s.read(N)

        # Check if there are unexpected errors in the message.
        # Last two bytes of the payload is the checksum bytes.
        checksum_array = array('B', [data[N - 1], data[N - 2]])
        checksum = struct.unpack('H', checksum_array)[0]

        # Compare the received checksum value against the calculated checksum.
        crc = self.calculate_imu_crc(data[:N - 2])
        if not crc == checksum:
            print('IMU CRC error')
            return False

        # If the checksum is valid, parse the data.
        return self.parse_data(data)

    def parse_data(self, data):
        """
        Parse the bytes of the sensor measurements

        Args:
        data: (byte array) - data read from the serial port

        Return:
        bool - True if the operation is successful
        """

        try:
            with self._lock:

                # header = struct.unpack('c', data[0:3])[0]

                self._delta_t = struct.unpack('f', data[3:7])[0]

                delta_roll_deg  = struct.unpack('f', data[7:11])[0]
                delta_pitch_deg = struct.unpack('f', data[11:15])[0]
                delta_yaw_deg   = struct.unpack('f', data[15:19])[0]

                self._delta_theta = DeltaTheta(delta_roll=delta_roll_deg * np.pi / 180,   # rotation around x
                                               delta_pitch=delta_pitch_deg * np.pi / 180,  # rotation around y
                                               delta_yaw=delta_yaw_deg * np.pi / 180)   # rotation around z

                self._delta_velocity = DeltaVelocity(dv_x=struct.unpack('f', data[19:23])[0],
                                                     dv_y=struct.unpack('f', data[23:27])[0],
                                                     dv_z=struct.unpack('f', data[27:31])[0])

                # self._delta_theta = self._delta_theta + 0.2*self._delta_t # TODO - remove manual bias

                self._timestamp += int(self._delta_t * 1e9)
                self._curr_measurement = IMUData.from_array(timestamp=self._timestamp,
                                                            delta_theta=self._delta_theta,
                                                            delta_velocity=self._delta_velocity,
                                                            delta_t=self._delta_t)

                # crc_byte = struct.unpack('c', data[32:35])[0]

        except:
            print('IMU: error parsing data')
            return False

        return True

    @staticmethod
    def calculate_imu_crc(data):
        """
        Calculate the 16-bit CRC for the given message.

        Args:
        data: (byte array) - data read from the serial port

        Return:
        unsigned short - CRC checksum value
        """
        data = bytearray(data)

        crc = np.array([0], dtype=np.ushort)
        for i in range(len(data)):
            crc[0] = (crc[0] >> 8) | (crc[0] << 8)
            crc[0] ^= data[i]
            crc[0] ^= (crc[0] & 0xff) >> 4
            crc[0] ^= crc[0] << 12
            crc[0] ^= (crc[0] & 0x00ff) << 5

        return crc[0]

    # def read_data(self):
    #     '''Output the current measurements.
    #
    #     Return:
    #     ImuData - current IMU data
    #     '''
    #
    #     with self._lock:
    #         data = ImuData(
    #             self._delta_t,
    #             self._delta_theta,
    #             self._delta_vel)
    #     return data

    def end_thread(self):
        """
        Call to end the IMU thread.
        """

        self._on = False
        print('IMU: thread close signal received')
