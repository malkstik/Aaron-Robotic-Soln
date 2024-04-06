import sys

from sensor_interfaces.srv import GetLoad
from sensor_interfaces.msg import SensorData, SensorDataArray

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from time import time

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self._cli = self.create_client(GetLoad, 'get_load_cell_data')
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self._req = GetLoad.Request()
        timer_period = 0.002 # seconds
        self._servicetimer = self.create_timer(timer_period, self.service_timer_callback)
        self._publishtimer = self.create_timer(timer_period, self.publish_timer_callback)
        
        self._publisher = self.create_publisher(SensorData, 'sensor', 10)

        self.msg = SensorData()


    def service_timer_callback(self):
        now = time.time()
        service_response = self.send_request(now)
        
        # Make new message
        msg = SensorData()
        msg.data = service_response.data
        msg.timestamps = service_response.timestamps

        # Update message
        self.msg = msg

        self.get_logger().info('Got sensor data')

    def publish_timer_callback(self):
        self._publisher.publish(self.msg)

    def send_request(self, time: float):
        self._req.req_time = time
        self.future = self._cli.call_async(self._req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    executor = MultiThreadedExecutor()
    
    
    executor.add_node(minimal_client)


    try:
        executor.spin()
    finally:
        executor.shutdown()
        minimal_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()