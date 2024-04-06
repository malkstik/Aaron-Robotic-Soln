from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_srvcli',
            executable='server',
            name='minimal_service',
            parameters = [{'address': '127.0.0.3',
                           'port': 10000,
                           'num_samples': 10
            }]
        ),
        Node(
            package='sensor_srvcli',
            executable='client',
            name='minimal_client_async',
            parameters = [{

            }]
        ),

        # Node(
        #     package='sensor_srvcli',
        #     executable='visualizer',
        #     name='visualizer',
        #     parameters = [{

        #     }]

        # )
    ])
