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

Please see `launch/sensor_srvcli.launch.py`  and `config/config.yaml` for an example of how to configure the network. Configuring and adding new sensors is automatically handled by editing `config/config.yaml`.

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

# Additional Analytics

Configuring the services to be optimal requires some tuning. To support this the following features have been added:

### Data Visualization

DataViz.py contains a node that live plots the data obtained from the `sensors` topic. To enable this feature set `viz` to `True` in `config/config.yaml`.

Below is an example:

<img src="https://github.com/malkstik/Aaron-Robotic-Soln/blob/master/_images/CorrectLivePlot.png?raw=true" alt="drawing" width="600"/>

This is currently hard coded to only work when there are two 6DOF sensors. Each column corresponds to a single sensor. The sensor values are plotted on the y axis with the timestamp on the x axis.

### True Update Rate Monitoring

Much of the data published to the sensors topic is redundant. This is because the servers run slower than the 500Hz that the topic gets published at. To monitor the nonredundant updates, a timestamp is sent to <sensor_id>_update whenever new data is sent. This allows us to run

```
ros2 topic hz sensor_id>_update
```

to observe the true update rate.
