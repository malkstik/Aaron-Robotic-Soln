from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_srvcli',
            executable='server',
            name='sensor1_server',
            parameters = [{'address': '127.0.0.3',
                           'port': 10000,
                           'num_samples': 10,
                           'polling_rate': 2000,
                           'dof': 6,
                           'sensor_id': 'sensor1'
            }]
        ),
        Node(
            package='sensor_srvcli',
            executable='server',
            name='sensor2_server',
            parameters = [{'address': '127.0.0.1',
                           'port': 10000,
                           'num_samples': 10,
                           'polling_rate': 4000,
                           'dof': 6,
                           'sensor_id': 'sensor2'
            }]
        ),
        Node(
            package='sensor_srvcli',
            executable='client',
            name='minimal_client_async',
            parameters = [{'sensors': ['sensor1', 'sensor2']
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
