import numpy as np

from sensor_interfaces.srv import GetLoad

from .utils.sensor_utils import SensorReader, SensorDataArrayMsg
from collections import deque 

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.time import Time, Duration

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        timer_period = 0.01 # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)
        self._clock = ROSClock()

        #Declare default parameters
        self.declare_parameter('address', '127.0.0.3')
        self.declare_parameter('port', 10000)
        self.declare_parameter('num_samples', 10)
        self.declare_parameter('polling_rate', 2000)
        self.declare_parameter('dof', 3)
        self.declare_parameter('sensor_id', "sensor1")

        #Get parameters
        self._address = self.get_parameter('address').value
        self._port = self.get_parameter('port').value
        self._num_samples = self.get_parameter('num_samples').value
        self._rate = self.get_parameter('polling_rate').value
        self._dof = self.get_parameter('dof').value
        self._sensor_id = self.get_parameter('sensor_id').value

        #Setup server
        self._srv = self.create_service(GetLoad, f'get_{self._sensor_id}_data', self.getLoadCellData)

        #Setup reader
        self._reader = SensorReader(self._address, self._port) 
        self._reader.num_samples = self._num_samples

        #Init deques
        self._sensor_deque = deque()
        self._sensor_timestamps = deque()

        #Useful values
        self._dt_array = np.arange(int(self._num_samples)+1, 1, step =-1)/self._rate
        self._dt_array = [Duration(seconds = 0, nanoseconds = dt*1e9) for dt in self._dt_array]


    def timer_callback(self):        
        now = self._clock.now()
        times = [now + dt for dt in self._dt_array]
        data = self._reader.getData()

        self._sensor_deque.appendleft(data) 
        self._sensor_timestamps.appendleft(times)

    def getLoadCellData(self, request, response):
        '''
        Retrieves all load cell data from times BEFORE request timestamp to send
        '''

        req_time = Time.from_msg(request.req_time)

        # If no data or all data is newer than the request
        if not self._sensor_timestamps or req_time < self._sensor_timestamps[-1][0]: 
            self.get_logger().debug(f"No data, sending out empty service response")
            return response

        # Search for what portion of deque is older than the request
        i = 0
        while req_time < self._sensor_timestamps[i][0]:
            i += 1
        
        num_pop = len(self._sensor_timestamps) - i + 1

        data = None
        timestamps = []
        for _ in range(num_pop):
            popped_data = self._sensor_deque.pop() 
            popped_time = self._sensor_timestamps.pop() 
            data = popped_data if data is None else np.hstack((popped_data,data))
            timestamps = popped_time + timestamps


        data, timestamps = self.filter(data, timestamps)

        response.data = SensorDataArrayMsg(data, timestamps, self._sensor_id, self.get_logger()).getMsg()
        self.get_logger().debug(f"Sending out service response")
        return response

    def filter(self, data: np.ndarray, time: list[Time]) -> tuple[np.ndarray, deque]:
        '''
        filter and unflatten data
        '''
        # For now just unflatten raw data
        filtered_data = data.reshape((-1, self._dof))
        filtered_time = time
        return filtered_data, filtered_time

def main():
    rclpy.init()

    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
