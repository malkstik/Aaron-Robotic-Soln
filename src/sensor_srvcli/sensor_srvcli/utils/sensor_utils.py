import socket
import sys
import numpy as np
import time

from typing import Union
from collections import deque
from sensor_interfaces.msg import SensorData, SensorDataArray
from std_msgs.msg import String
from rclpy.clock import Time

class SensorReader:
    '''
    Reads sensor data at provided ip address and port
    '''
    def __init__(self, server_address: str = '127.0.0.3', port: int = 10000):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((server_address, port))

        self._num_samples = "10"

    @property
    def num_samples(self) -> str:
        '''
        Getter for self._num_samples
        '''
        return self._num_samples
    
    @num_samples.setter
    def num_samples(self, n: Union[str, int]) -> None:
        '''
        Ensure that self._num_samples is a string
        :param n: Number of samples get when calling self.getData, can be of type str or int
        '''
        if isinstance(n, int):
            n = str(n)
        self._num_samples = n

    def getData(self) -> np.ndarray:
        message = self._num_samples.encode()
        self._sock.sendall(message)

        byte_data = self._sock.recv(10000)        
        return np.frombuffer(byte_data)

    def __del__(self):
        self._sock.close()

class SensorDataArrayMsg:
    '''
    Reformats data as a SensorData msg type and packs it into a SensorDataArray.msg
    '''
    def __init__(self, data: np.ndarray, timestamps: list[Time], sensor_id: int, logger = None):
        self.data = data
        self.timestamps = timestamps
        self.logger = logger
        self.sensor_id = String(data = sensor_id)

    def getMsg(self):
        '''
        Encapsulates each data point as a SensorData.msg and returns an array of them
        :return: SensorDataArray.msg
        '''
        data_deque = deque()
        for i in range(len(self.timestamps)):
            # Front of queue is on the right so iterate backwards through indices
            data_point = self.data[-(i+1), :].tolist()
            time = self.timestamps[-(i+1)].to_msg()

            data_msg = SensorData(data = data_point,
                                  timestamp = time,
                                  sensor_id = self.sensor_id)
            data_deque.appendleft(data_msg)


        msg_array = SensorDataArray()
        msg_array.sensor_id = self.sensor_id
        msg_array.oldest_timestamp = self.timestamps[-1].to_msg()
        msg_array.data = list(data_deque)

        return msg_array

if __name__ == '__main__':
    pass