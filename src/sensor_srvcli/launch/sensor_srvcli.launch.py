from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import yaml
import os


def generate_launch_description():
    # Read yaml in
    config_file = os.path.join(get_package_share_directory("sensor_srvcli"), "config","config.yaml")
    with open(config_file) as stream:
        config = yaml.safe_load(stream)

    # Make server nodes
    server_nodes = []
    sensor_list = []    
    for sensor_params in config['sensors']:
        sensor_id = sensor_params['sensor_id']
        server_nodes.append(Node(package = 'sensor_srvcli',
                                 executable = 'server',
                                 name= f'{sensor_id}_server',
                                 parameters = [sensor_params]
                                 )
        )
        #Save list of sensors for client node
        sensor_list.append(sensor_id)

    # Make Client Node
    client_node = Node(
                        package='sensor_srvcli',
                        executable='client',
                        name='sensor_client_async',
                        parameters = [{'sensors': sensor_list}]
    )

    # Visualization Node (Only works for two servers with 6DOF data)
    viz_node = Node(
                        package='sensor_srvcli',
                        executable='visualizer',
                        name='visualizer',                  
    )


    #Launch
    return LaunchDescription(
        server_nodes + 
        [
        client_node,
        # viz_node
        ]
    )