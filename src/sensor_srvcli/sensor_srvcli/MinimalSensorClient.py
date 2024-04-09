import sys

from sensor_interfaces.srv import GetLoad
from sensor_interfaces.msg import SensorDataArray, MultipleSensorData
from builtin_interfaces.msg import Time as TimeMsg

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.clock import ROSClock
from rclpy.time import Time, Duration
from rclpy.client import Client
from rclpy.publisher import Publisher


class MinimalSensorClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.get_logger().set_level(LoggingSeverity.INFO)

        # Declare default parameters
        self.declare_parameter('sensors', ['sensor1'])
        
        # Get parameters
        self._sensors = self.get_parameter('sensors').value

        self._sensor_idx = {}
        for idx, sensor in enumerate(self._sensors):
            self._sensor_idx[sensor] = idx

        # Setup timers, publishers, clients, etc...
        self._cli = self.init_clients(self._sensors)
        self._req = GetLoad.Request()
        timer_period = 0.002 # seconds
        self._servicetimer = self.create_timer(timer_period, self.service_timer_callback)
        self._publishtimer = self.create_timer(timer_period, self.publish_timer_callback)
        self._publisher = self.create_publisher(MultipleSensorData, 
                                                'sensors', 
                                                10)
        self.update_trackers, self.batch_update_trackers = self.init_update_trackers(self._sensors)
        self._clock = ROSClock()

        # Init msg for sensor topic
        self._msg = MultipleSensorData(dataset = [SensorDataArray() for sensor_id in self._sensors])




    def init_clients(self, sensors: list[str]) -> list[Client]:
        clients = [self.create_client(GetLoad, f'get_{sensor_id}_data')for sensor_id in sensors]
        for idx, client in enumerate(clients):
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'get_{sensors[idx]}_data not available, waiting again...')
        
        self.wait_for_response ={}
        for sensor_id in sensors:
            self.wait_for_response[sensor_id] =  False

        return clients

    def init_update_trackers(self, sensors: list[str]) -> list[Publisher]:
        update_trackers = []
        batch_update_trackers = []
        for sensor_id in sensors:
            update_trackers.append(self.create_publisher(TimeMsg, 
                                                            f'{sensor_id}_update', 
                                                            10)
            )
            batch_update_trackers.append(self.create_publisher(TimeMsg, 
                                                            f'{sensor_id}_batch_update', 
                                                            10)
            )

        return update_trackers, batch_update_trackers
    
    def service_timer_callback(self):
        '''
        Send service requests on timer
        '''
        sec, nanosec = self._clock.now().seconds_nanoseconds()
        now = TimeMsg()
        now.sec = sec
        now.nanosec = nanosec

        for client, sensor_id in zip(self._cli, self._sensors):
            self.send_request(now, client, sensor_id)
        

    def publish_timer_callback(self):
        self._publisher.publish(self._msg)

    def send_request(self, time: TimeMsg, client: Client, sensor_id: str):
        '''
        Sends service requests and passes future to a callback, only sends a new request if the previous futureh as been handled
        '''
        if self.wait_for_response[sensor_id]:
            return
        
        self._req.req_time = time
        self.get_logger().debug(f'Sent request for {sensor_id}')
        future = client.call_async(self._req)
        self.wait_for_response[sensor_id] = True

        future_callback = lambda future: self.handle_service_response(future, sensor_id=sensor_id)
        rclpy.task.Future.add_done_callback(future, future_callback)

    def handle_service_response(self, future, sensor_id):
        '''
        process future from service response
        '''
        try:
            self.get_logger().debug('Received service response')
            # Make sure the future completed successfully
            if future.result() is not None:
                service_response = future.result()

                # Make new message only if service call was successful
                msg: SensorDataArray = service_response.data

                # Skip if data is empty or dataset is the same
                if len(msg.data) and msg.oldest_timestamp != self._msg.dataset[self._sensor_idx[sensor_id]].oldest_timestamp:
                    for i in range(len(msg.data)):
                        self.update_trackers[self._sensor_idx[sensor_id]].publish(self._clock.now().to_msg())                    
                    self.batch_update_trackers[self._sensor_idx[sensor_id]].publish(self._clock.now().to_msg())
                    self._msg.dataset[self._sensor_idx[sensor_id]] = msg

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