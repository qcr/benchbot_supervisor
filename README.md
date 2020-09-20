**NOTE: this software is part of the BenchBot software stack, and not intended to be run in isolation. For a working BenchBot system, please install the BenchBot software stack by following the instructions [here](https://github.com/roboticvisionorg/benchbot).**

# BenchBot Supervisor

<p align="center"><img alt="benchbot_supervisor" src="./docs/benchbot_supervisor.jpg"/></p>

The BenchBot Supervisor is a HTTP server facilitating communication between user-facing interfaces like the [BenchBot API](https://github.com/roboticvisionorg/benchbot_api), & the low-level robot components like [BenchBot Simulator](https://github.com/roboticvisionorg/benchbot_simulator) or real robots. Communication is typically routed through a [BenchBot Robot Controller](https://github.com/RoboticVisionOrg/benchbot_robot_controller), which provides automated process management for low-level components and wraps ROS communications.

## Installing & running the BenchBot Supervisor

BenchBot Supervisor is a ROS package: it contains a ROS node which communicates downstream to low-level components, & contains a HTTP server for upstream communication. The package is installed like any other ROS package:

```
u@pc:~$ git clone https://github.com/roboticvisionorg/benchbot_supervisor
u@pc:~$ ln -sv "$(pwd)/benchbot_supervisor" <CATKIN_WS>/src/
u@pc:~$ cd <CATKIN_WS> && catkin_make
```

Once installed, the ROS node is run via:

```
u@pc:~$ rosrun benchbot_supervisor benchbot_supervisor
```

The following parameters are typically required for a useful instantiation of the supervisor:

- **task_name**: string describing the requested task (format is `'type:control_mode:localisation_mode'`)
- **robot_file**: filename for a YAML file describing a robot as a list of connections (e.g. `'carter.yaml'`)
- **observations_file**: filename for a YAML file listing which connections are observations (e.g. `'ground_truth.yaml'`)
- **actions_file**: filename for a YAML file listing which connections are actions (e.g. `'active.yaml'`)
- **environment_files**: colon separated list of filenames pointing to the environment metadata file of each environment intended to be run
- **simulator_address**: address of a running simulator controller (e.g. `'benchbot_simulator:10000'`)

As an example, the below command runs the supervisor for a scene change detection task, where active control is employed with ground truth localisation on a Carter robot, & environment miniroom:1:5 is used (a simulator is also available at address `'benchbot_simulator:10000'`):

```
u@pc:~$ rosrun benchbot_supervisor benchbot_supervisor \
    _task_name:='scd:active:ground_truth' \
    _robot_file:='carter.yaml' \
    _observations_file:='ground_truth.yaml' \
    _actions_file:='active.yaml' \
    _environment_files:='<envs_dir>/miniroom_1.yaml:<envs_dir>/miniroom_5.yaml' \
    _simulator_address:='benchbot_simulator:10000'
```

## Employing environment, robot, & task configurations

The BenchBot Supervisor requires configuration details for the environment, robot, & selected task so that it can manage each of the system components accordingly (i.e. API interaction & control of the simulator / real robot). Configuration details are either provided by YAML files, or descriptive strings as show in the above command.

### Using environment configurations

Environment configurations are created upon the creation of each environment, & not the role of the supervisor to create. Instead, the supervisor must be pointed to the configuration files using the `'environment_files'` argument. The URL in [this file](https://cloudstor.aarnet.edu.au/plus/s/egb4u65MVZEVkPB/download) points to the latest BenchBot environments, which includes their environment files in `./benchbot_data_files`.

### Defining robot configurations

The BenchBot Supervisor defines a robot as a series of directional "connections" either passing data from the robot up through the supervisor, or down through the supervisor to the robot. The snippet below for the Carter robot shows an example of both types of connection:
```yaml
# ./robots/carter.yaml

...
image_rgb:
  connection: "ros_to_api"
  ros_topic: "/camera/color/image_raw"
  ros_type: "sensor_msgs/Image"
  callback_supervisor: "supervisor_callbacks.encode_color_image"
  callback_api: "api_callbacks.decode_color_image"
...
move_distance:
  connection: "api_to_ros"
  ros_topic: "/cmd_vel"
  ros_type: "geometry_msgs/Twist"
  callback_supervisor: "supervisor_callbacks.move_distance"
...
```
The connection `image_rgb` passes an image from the robot up towards the BenchBot API, whereas the `move_distance` connection passes a movement command  from the API down to the robot. Callback definitions are used to handle the conversion process between ROS data, HTTP data, & the simple data required in the API. All callbacks are defined by a string which Python will dynamically attempt to import. For example, `'supervisor_callbacks.move_distance'` would be translated to:
```python
from supervisor_callbacks import move_distance
```

Callbacks at the API level (`callback_api`) are defined in [BenchBot API](https://github.com/roboticvisionorg/benchbot_api) & convert HTTP encoded data into easy-to-use Python data structures. Callbacks at the supervisor level (`callback_supervisor`) handle converting ROS data into HTTP encoded data, or vice versa, depending on the connection direction.

### Defining task configurations

Task requests are split into 3 parts: the task type, robot control mode, & localisation mode. The `task_name` string, an ordered colon separated string is used to declare these parts to the supervisor. 

The supervisor uses the robot control mode & localisation mode to determine which robot connections should be used for actions & observations respectively. The `actions_file` & `observations_file` arguments should correlate with the task selected in `task_name`. The actions & observations YAML files are simply lists of connection names that the supervisor should use for their respective purposes. For example, the `./actions/active_control.yaml` denotes the following connections as actions:
```yaml
# ./actions/active_control.yaml

["move_distance", "move_angle"]
```
and the below file lists what observation sources should be available in ground truth localisation mode:
```yaml
# ./observations/ground_truth.yaml

["image_depth", "image_depth_info", "image_rgb", "image_rgb_info", "laser", "poses"]
```

## Interacting with the BenchBot Supervisor

The supervisor includes a RESTful API for all interaction with a user-facing API. The RESTful API includes the following commands:

| Request Route | Response Format | Description |
| --------------|---------------|-------------|
| `/`           | <pre>Hello, I am the BenchBot supervisor</pre> | Arbitrary response to confirm connection. |
| `/config/`    | <pre>{<br> ...<br> 'param_name': param_value,<br> ...<br>}</pre> | Dictionary containing containing parameter values for all of supervisor configuration settings. Keys correspond to parameter name, & values to parameter value. |
| `/config/<config>` | `config_value` | Directly retrieve the value of a supervisor configuration parameter with name `'config'`. Returns `param_value` of `'config'`. |
| `/connections/<connection>` | `dict` | Returns the response of the connection (e.g. an `image_rgb` connection would return the image) as a `dict`. Format & style of the `dict` is defined by the methods described above in "Defining environment, robot, & task configurations". |
| `/robot/` | <pre>Hello, I am the BenchBot robot controller</pre> | Arbitrary response confirming a robot controller is available. |
| `/robot/<command>` | `dict` | Passes the command `command` down to a running robot controller manager. See [BenchBot Robot Controller](https://github.com/roboticvisionorg/benchbot_robot_controller) for documentation of supported commands & expected responses. |
