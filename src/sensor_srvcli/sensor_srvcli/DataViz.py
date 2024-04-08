import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_interfaces.msg import MultipleSensorData
from builtin_interfaces.msg import Time
import numpy as np

class SensorDataPlotter(Node):
    def __init__(self):
        super().__init__('sensor_data_plotter')
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        self.subscription = self.create_subscription(
            MultipleSensorData,
            'sensors',
            self.listener_callback,
            10)

        self.plot_timer = self.create_timer(0.1, self.pause_for_plot)

        # Set up the plot
        self.fig, self.ax = plt.subplots(6, 2)
        self.ax_flat = self.ax.flatten()
        self.scatter_plots = [ax.scatter([], [], s=10) for ax in self.ax_flat]  # 's' is for size

        # Iterate through all axes to configure x and y labels
        for i, ax in enumerate(self.ax_flat):
            if i == 0:
                ax.set_title('sensor1')
            elif i == 1:
                ax.set_title('sensor2')

            ax.set_xlim(0, 10)  
            ax.set_ylim(0, 1) 
            # Label y-axis only for leftmost subplots
            if i % 2 == 0:  
                ax.set_ylabel("Val")  
            else:
                ax.set_yticklabels([])  #

            if i // 2 == len(self.ax) - 1: 
                ax.set_xlabel("Time [s]")  # Set the x-axis label
            else:
                ax.set_xticklabels([])  # Hide x-axis tick labels for others



        # Store the most recent data received from the callback
        self.current_data = None
        self.current_timestamps = None
        
        # Set up plot parameters with animation
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.plot_init, blit=True)

    def pause_for_plot(self):
        plt.pause(0.001)

    def plot_init(self):
        for scatter_plot in self.scatter_plots:
            scatter_plot.set_offsets(np.empty((0, 2)))
        return self.scatter_plots

    def update_plot(self, frame):
        # Use the current data to update the plots

        if self.current_data is not None and self.current_timestamps is not None:
            for i, scatter in enumerate(self.scatter_plots):
                data_index = i % 2  # Index for the current data set 
                row_index = i %  6  # Index for the row in the current data set

                times = self.current_timestamps[data_index]
                datapts = self.current_data[data_index][row_index]

                scatter.set_offsets(np.c_[times, datapts])

                latest_timestamp = self.current_timestamps[data_index][0]

                self.ax_flat[i].set_xlim(latest_timestamp - 1, latest_timestamp + 1)
                self.ax_flat[i].relim()  # Recalculate limits
                self.ax_flat[i].autoscale_view()  # Rescale the view

        return self.scatter_plots  

    def time_to_seconds(self, time_msg: Time):
        return time_msg.sec + time_msg.nanosec / 1_000_000_000

    def get_data_from_msg(self, msg: MultipleSensorData):
        dataset = msg.dataset
        sensor1, sensor2 = dataset[0].data, dataset[1].data

        sensor1_timestamps = np.zeros(len(sensor1))
        sensor1_data = np.zeros((6, len(sensor1)))

        for idx, data in enumerate(sensor1):
            sensor1_timestamps[idx] = self.time_to_seconds(data.timestamp)
            sensor1_data[:,idx] = data.data

        sensor2_timestamps = np.zeros(len(sensor2))
        sensor2_data = np.zeros((6, len(sensor2)))

        for idx, data in enumerate(sensor2):
            sensor2_timestamps[idx] = self.time_to_seconds(data.timestamp)
            sensor2_data[:,idx] = data.data

        if self.current_data is not None and self.current_timestamps is not None:
            sensor1_data = np.hstack((sensor1_data, self.current_data[0]))
            sensor2_data = np.hstack((sensor2_data, self.current_data[1]))

            sensor1_timestamps = np.hstack((sensor1_timestamps, self.current_timestamps[0]))
            sensor2_timestamps = np.hstack((sensor2_timestamps, self.current_timestamps[1]))

        self.current_data = [sensor1_data, sensor2_data]
        self.current_timestamps = [sensor1_timestamps, sensor2_timestamps]



    def listener_callback(self, msg):
        self.get_logger().info(f'Latest sensor1 timestamp: {msg.dataset[0].data[0].timestamp}')
        self.get_logger().info(f'Latest sensor2 timestamp: {msg.dataset[1].data[0].timestamp}')

        try:
            self.get_data_from_msg(msg)  # Process and store the data
        except Exception as e:
            self.get_logger().error(f"Failed to process message: {str(e)}")

        

def main(args=None):
    rclpy.init(args=args)
    sensor_data_plotter = SensorDataPlotter()

    # Show the plot
    plt.show(block=False)

    try:
        rclpy.spin(sensor_data_plotter)
    finally:
        # Clean up
        sensor_data_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()