import sys

from sensor_interfaces.srv import GetLoad
from sensor_interfaces.msg import SensorDataArray
from builtin_interfaces.msg import Time as TimeMsg

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import ROSClock
from rclpy.time import Time, Duration

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
        self._publisher = self.create_publisher(SensorDataArray, 'sensor', 10)
        self._clock = ROSClock()

        self.msg = SensorDataArray()

        self.wait_for_response = False

        self.get_logger().set_level(LoggingSeverity.DEBUG)


    def service_timer_callback(self):
        sec, nanosec = self._clock.now().seconds_nanoseconds()
        now = TimeMsg()
        now.sec = sec
        now.nanosec = nanosec

        # service_response = self.send_request(now)
        self.send_request(now)
        
        # # Make new message
        # msg = SensorDataArray()
        # msg.data = service_response.data
        # msg.timestamps = service_response.timestamps

        # # Update message
        # self.msg = msg

    def publish_timer_callback(self):
        # self.get_logger().debug(f"Published message")
        self._publisher.publish(self.msg)

    def send_request(self, time: TimeMsg):
        if self.wait_for_response:
            return
        self._req.req_time = time

        # self.get_logger().debug(f"Sending out service request")
        self.future = self._cli.call_async(self._req)
        self.wait_for_response = True
        # self.get_logger().debug(f"Waiting for service response")

        rclpy.task.Future.add_done_callback(self.future, self.handle_service_response)
        # rclpy.spin_until_future_complete(self, self.future)
        # self.get_logger().debug(f"Received response")
        # return self.future.result()

    def handle_service_response(self, future):
        try:
            self.get_logger().debug('Received service response')
            # Make sure the future completed successfully
            if future.result() is not None:
                service_response = future.result()

                # Make new message only if service call was successful
                msg = service_response.data

                # Update message if theres new data
                if len(msg.data):
                    self.msg = msg
                self.wait_for_response = False
            else:
                self.get_logger().error('Service call failed %r' % (future.exception(),))

        except Exception as e:
            self.get_logger().error('An error occurred in the service response handler: %r' % (e,))


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