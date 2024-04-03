import sys

from sensor_srv.srv import GetLoad
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetLoad, 'get_load_cell_data')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetLoad.Request()

    def send_request(self, num_datapoints: int):
        self.req.num_datapoints = num_datapoints
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        f"Retrieved data")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()