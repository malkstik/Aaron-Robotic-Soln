from sensor_interfaces.srv import GetLoad

import rclpy
from rclpy.node import Node

import socket
import sys
import numpy as np
import time

from .utils.sensor_utils import SensorReader
from collections import deque 



class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self._srv = self.create_service(GetLoad, 'get_load_cell_data', self.getLoadCellData)
        timer_period = 0.01 # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

        #Declare default parameters
        self.declare_parameter('address', '127.0.0.3')
        self.declare_parameter('port', 10000)
        self.declare_parameter('num_samples', '10')

        #Get parameters
        address = self.get_parameter('address').value
        port = self.get_parameter('port').value
        num_samples = self.get_parmeter('num_samples').value

        #Setup reader
        self._reader = SensorReader(address, port) 
        self._reader.num_samples = num_samples

        #Init deques
        self.sensor_deque = deque()
        self.sensor_timestamps = deque()

        self.time_array = np.arange(num_samples+1, 1, step =-1)/2000

    def timer_callback(self):
        times = time.time() + self.time_array
        data = self._reader.getData()
        self.sensor_deque.appendleft(reversed(data)) #assume that most recent data point in data is at data[-1]
        self.sensor_timestamps.appendleft(times)
        self.get_logger().info('Got sensor data')

    def getLoadCellData(self, request, response):
        i = 0
        while request.req_time < self.sensor_timestamps[i][0]:
            i += 1
        
        num_pop = len(deque) - i + 1

        data = None
        timestamps = None
        for _ in range(num_pop):
            popped_data = self.sensor_deque.pop() 
            popped_time = self.sensor_timestamps.pop() 
            data = popped_data if data is None else np.hstack((popped_data,data))
            timestamps = popped_time if timestamps is None else np.hstack((popped_time,timestamps))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
