# Description

This code serves as a solution for the assignment provided by MachinaLabs, detailed at [https://github.com/Machina-Labs/robotic_hw](https://github.com/Machina-Labs/robotic_hw)

Each sensor corresponds to a single server node. There is one client node that sends service requests to all available servers, where there exists one server for each sensor. These requests return data in the form of a `SensorDataArray.msg`. The client node concatenates this data and publishes them in the form of a `MultipleSensorData.msg` to the `sensor` topic.

## Message Types

### MultipleSensorData.msg

This message type stores the **set** of `SensorDataArray.msg` corresponding to **each** `sensor_id`.

```
SensorDataArray[] dataset
```

### SensorDataArray.msg

This message type stores the **set** of `SensorData.msg` corresponding to a **single** `sensor_id`.

```
SensorData[] data
std_msgs/String sensor_id
builtin_interfaces/Time oldest_timestamp
```

### SensorData.msg

This message type stores the a **single** data point corresponding to a **single** `sensor_id`. The data field has equivalent length to the sensor's degrees of freedom.

```
float64[] data
builtin_interfaces/Time timestamp
std_msgs/String sensor_id
```

# Dependencies

This code has been developed using:

* Python 3.10.12
* NumPy 1.21.5
* ROS2 Humble
* Docker version 24.0.6, build ed223bc

# Instructions

This code has been developed fully from within a Docker container. Using it with a host machine has not been tested but should work.

Please see `launch/sensor_srvcli.launch.py`  and `config/config.yaml` for an example of how to configure the network. Configuring and adding new sensors is automatically handled by editing `config/config.yaml`. Note that data visualization does not dynamically support different configurations.

## Using Docker

source/opt/ros/humble/setup.bash is handled by ~/.bashrc

```
cd <path_to_repo>
./.docker/build.sh <path_to_repo>/.docker 
./.docker/run.sh <path_to_repo> latest

#In docker container
cd /home/Aaron/mnt
colcon build
source install/setup.sh

#The following command may be split up but requires multiple terminals. If split up, sensor.py must be executed before the launch file
python3 sensor.py & ros2 launch sensor_srvcli sensor_srvcli.launch.py 
```

## Using host machine

```
cd <path_to_repo>
colcon build
source install/setup.sh

#The following command may be split up but requires multiple terminals. If split up, sensor.py must be executed before the launch file
python3 sensor.py & ros2 launch sensor_srvcli sensor_srvcli.launch.py 
```

# Additional Analytics

Configuring the services to be optimal requires some tuning. To support this the following features have been added:

### Data Visualization

`src/sensor_srvcli/sensor_srvcli/DataViz.py` contains a node that live plots the data obtained from the `sensors` topic. To enable this feature set `viz` to `True` in `config/config.yaml`.

Below is an example:

<img src="https://github.com/malkstik/Aaron-Robotic-Soln/blob/master/_images/CorrectLivePlot.png?raw=true" alt="drawing" width="600"/>

This is currently hard coded to only work when there are two 6DOF sensors. Each column corresponds to a single sensor. The sensor values are plotted on the y axis with the timestamp on the x axis.

### True Update Rate Monitoring

Much of the data published to the `sensors` topic is redundant. This is because the servers run slower than the 500Hz that the topic gets published at. To monitor the nonredundant updates, a timestamp is sent to the `<sensor_id>_update` topic  and `<sensor_id>_batch_update` whenever new data is sent. This allows us to run

```
ros2 topic hz <sensor_id>_update
```

to observe the true update rate. This accounts for each individual datapoint that is newly sent to the `sensors` topic. This gives an idea of how much data flows 

Alternatively, to observe the batch update rate, we can run

```
ros2 topic hz <sensor_id>_batch_update
```

This measures the rate of distinct chunks of data coming in.

If a large density of data is required per time interval and these time intervals can be longer, then `num_samples` should be tuned to optimize the `<sensor_id>_update` rate. If a smaller density of data with more frequent updates is desired, then `num_samples` should be tuned to optimize the `<sensor_id>_batch_update` rate. There is necessarily a trade off between the two rates.
