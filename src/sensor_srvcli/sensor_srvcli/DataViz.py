import rclpy
from rclpy.node import Node


from sensor_interfaces.msg import SensorData, SensorDataArray
import matplotlib.pyplot as plt



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('visualizer')
        self.subscription = self.create_subscription(
            SensorData,
            'sensor',
            self.data_callback,
            10)

    def data_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()