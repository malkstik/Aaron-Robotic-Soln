# Description

This code serves as a solution for the assignment provided by MachinaLabs, detailed at [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

Each sensor corresponds to a single server node. There is one client node that sends service requests to all available servers (as indicated by the sensors parameter). The client node publishes the most up to date data it has received from the servers to the `sensor` topic.

# Dependencies

This code has been developed using:

* Python 3.10.12
* NumPy 1.21.5
* ROS2 Humble
* Docker version 24.0.6, build ed223bc

# Instructions

This code has been developed fully from within a Docker container. Using it with a host machine has not been tested but should work.

Please see `launch/sensor_srvcli.launch.py` for an example of how to configure the network. 

To add new sensors, create a new node with appropriate parameters and update `src/sensor_srvcli/utils/sensor_config.py`. The keys refer to the `sensor_id`. The values are indices. Accordingly, new sensors should be the next integer not already in `SENSOR_IDX.values() `

## Using Docker

```
cd <path_to_repo>
./.docker/build.sh <path_to_repo> 
./.docker/run.sh <path_to_repo> latest

#In docker container
cd /home/Aaron/mnt
colcon build
source install/setup.sh
python3 sensor.py & ros2 launch sensor_srvcli sensor_srvcli.launch.py 
```

## Using host machine

```
cd <path_to_repo>
colcon build
source install/setup.sh
python3 sensor.py & ros2 launch sensor_srvcli sensor_srvcli.launch.py 
```
