## A simple example of how to call sensor.py

# socket_echo_client.py
import socket
import sys
import numpy as np
import time

from typing import Union

class SensorReader:
    '''
    Reads sensor data at provided ip address and port
    '''
    def __init__(self, server_address: str = '127.0.0.3', port: int = 10000):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect(server_address, port)

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

    def getData(self) -> np.ndarray[float]:
        message = self._num_samples.encode()
        self._sock.sendall(message)

        byte_data = self._sock.recv(10000)        
        return np.frombuffer(byte_data)

    def __del__(self):
        self._sock.close()

if __name__ == '__main__':
    pass