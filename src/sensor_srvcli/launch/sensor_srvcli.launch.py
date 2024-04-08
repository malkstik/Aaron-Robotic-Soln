from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import PathJoinSubstitution, Command

import yaml
import os
def generate_launch_description():

    config_file = os.path.join(get_package_share_directory("sensor_srvcli"), "config","config.yaml")
    with open(config_file) as stream:
        config = yaml.safe_load(stream)

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
        sensor_list.append(sensor_id)

    client_node = Node(
                        package='sensor_srvcli',
                        executable='client',
                        name='sensor_client_async',
                        parameters = [{'sensors': sensor_list}]
    )


    return LaunchDescription(
        server_nodes + \
        [client_node] 
    )