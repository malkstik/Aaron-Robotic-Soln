import sys

from sensor_interfaces.srv import GetLoad
from sensor_interfaces.msg import SensorDataArray, MultipleSensorData
from builtin_interfaces.msg import Time as TimeMsg
from .utils.sensor_config import SENSOR_IDX

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import ROSClock
from rclpy.time import Time, Duration
from rclpy.client import Client

class MinimalSensorClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Declare default parameters
        self.declare_parameter('sensors', ['sensor1'])
        
        # Get parameters
        self._sensors = self.get_parameter('sensors').value

        # Setup timers, publishers, clients, etc...
        self._cli = self.init_clients(self._sensors)
        self._req = GetLoad.Request()
        timer_period = 0.002 # seconds
        self._servicetimer = self.create_timer(timer_period, self.service_timer_callback)
        self._publishtimer = self.create_timer(timer_period, self.publish_timer_callback)
        self._publisher = self.create_publisher(MultipleSensorData, 'sensor', 10)
        self._clock = ROSClock()

        # Init msg for sensoor topic
        self.msg = MultipleSensorData(dataset = [SensorDataArray() for sensor_id in self._sensors])


    def init_clients(self, sensors: list[str]) -> list[Client]:
        clients = [self.create_client(GetLoad, f'get_{sensor_id}_data')for sensor_id in sensors]
        for idx, client in enumerate(clients):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'get_{sensors[idx]}_data not available, waiting again...')
        
        self.wait_for_response ={}
        for sensor_id in sensors:
            self.wait_for_response[sensor_id] =  False

        return clients

    def service_timer_callback(self):
        sec, nanosec = self._clock.now().seconds_nanoseconds()
        now = TimeMsg()
        now.sec = sec
        now.nanosec = nanosec

        for client, sensor_id in zip(self._cli, self._sensors):
            self.send_request(now, client, sensor_id)
        

    def publish_timer_callback(self):
        self._publisher.publish(self.msg)

    def send_request(self, time: TimeMsg, client: Client, sensor_id: str):
        if self.wait_for_response[sensor_id]:
            return
        
        self._req.req_time = time
        self.get_logger().debug(f'Sent request for {sensor_id}')
        future = client.call_async(self._req)
        self.wait_for_response[sensor_id] = True

        future_callback = lambda future: self.handle_service_response(future, sensor_id=sensor_id)
        rclpy.task.Future.add_done_callback(future, future_callback)

    def handle_service_response(self, future, sensor_id):
        try:
            self.get_logger().debug('Received service response')
            # Make sure the future completed successfully
            if future.result() is not None:
                service_response = future.result()

                # Make new message only if service call was successful
                msg = service_response.data

                # Skip if data is empty or dataset is the same
                if len(msg.data) and msg.oldest_timestamp != self.msg.dataset[SENSOR_IDX[sensor_id]].oldest_timestamp:
                    self.msg.dataset[SENSOR_IDX[sensor_id]] = msg
            else:
                self.get_logger().error('Service call failed %r' % (future.exception(),))

        except Exception as e:
            self.get_logger().error('An error occurred in the service response handler: %r' % (e,))
        finally:
            self.wait_for_response[sensor_id]= False

def main():
    rclpy.init()

    minimal_client = MinimalSensorClientAsync()
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