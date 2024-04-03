from sensor_srv.srv import GetLoad

import rclpy
from rclpy.node import Node

import socket
import sys
import numpy as np
import time

from utils.sensor_utils import SensorReader

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(GetLoad, 'get_load_cell_data', self.add_two_ints_callback)
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('address', '127.0.0.3')
        self.declare_parameter('port', 10000)
        self.declare_parameter('num_samples', '10')


        address = self.get_parameter('address').value
        port = self.get_parameter('port').value

        self._reader = SensorReader(address, port) 
        self._reader.num_samples = self.get_parmeter('num_samples').value

    def timer_callback(self):
        self._reader.getData()
        #TODO: Handle data from reader
        self.get_logger().info('Got sensor data')

    def getLoadCellData(self, request, response):
        #TODO: Handle data from reader
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
