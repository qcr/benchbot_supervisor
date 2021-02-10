**NOTE: this software is part of the BenchBot software stack, and not intended to be run in isolation. For a working BenchBot system, please install the BenchBot software stack by following the instructions [here](https://github.com/roboticvisionorg/benchbot).**

# BenchBot Supervisor

<p align="center"><img alt="benchbot_supervisor" src="./docs/benchbot_supervisor.jpg" width="60%"/></p>

The BenchBot Supervisor is a HTTP server facilitating communication between user-facing interfaces like the [BenchBot API](https://github.com/roboticvisionorg/benchbot_api), and the low-level robot components like [BenchBot Simulator](https://github.com/roboticvisionorg/benchbot_simulator) or real robots. Communication is typically routed through a [BenchBot Robot Controller](https://github.com/RoboticVisionOrg/benchbot_robot_controller), which provides automated process management for low-level components and wraps all ROS communications.

## Installing and running the BenchBot Supervisor

BenchBot Supervisor is a Python package containing a `Supervisor` class that wraps a HTTP server for both upstream and downstream communication. Install by running the following in the root directory of where this repository was cloned:

```
u@pc:~$ pip install .
```

Once installed, the Python class can be used as follows:

```python
from benchbot_supervisor import Supervisor

s = Supervisor(...args...)
s.run()
```

The following parameters are typically required for a useful instantiation of the supervisor:

- **addons_path**: path to installed [BenchBot add-ons](https://github.com/roboticvisionorg/benchbot_addons) (is the same as the directory where `manager.py` can be found)
- **task_name**: string matching the `'name'` field of an installed task
- **robot_name**: string matching the `'name'` field of an installed robot
- **environment_names**: list of strings, each matching the `'name':'variant'` field combination of an installed environment (the `'name'` must be the same for all environments in the list)
- **port**: select a different port than the default (10000)

The module can also be executed directly, which makes the passing of arguments from the command line simple (see `python -m benchbot_supervisor --help` for argument details):

```
u@pc:~$ python -m benchbot_supervisor ...args...
```

As an example, the below command runs the supervisor for a scene change detection task, where active control is employed with ground truth localisation on a simulated Carter robot, and environments miniroom:1 and miniroom:5 are used:

```
u@pc:~$ python -m benchbot_supervisor \
    --task-name scd:active:ground_truth \
    --robot-name carter \
    --environment-names miniroom:1,miniroom:5
```

## Employing task, robot, and environment configurations

The BenchBot Supervisor requires configuration details for the selected tasks, robots, and environments. It uses these details to manage each of the system components, like API interaction and control of the simulator / real robot. Configuration details are provided by YAML files, which are referenced via their `'name'` field as shown above.

The [BenchBot Add-ons Manager](https://github.com/roboticvisionorg/benchbot_addons) manages the installation of, and access to, these files. See the documentation there for further details on configuration files. All you need to do to use add-ons with the supervisor is provide the location via the `'addons_path'` argument.

## Interacting with the BenchBot Supervisor

The supervisor includes a RESTful HTTP API for all interaction with a user-facing API. The RESTful API includes the following commands:

| Request Route                   | Response Format                                                  | Description                                                                                                                                                                                                                                 |
| ------------------------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/`                             | <pre>Hello, I am the BenchBot supervisor</pre>                   | Arbitrary response to confirm connection.                                                                                                                                                                                                   |
| `/config/`                      | <pre>{<br> ...<br> 'param_name': param_value,<br> ...<br>}</pre> | Dictionary containing containing parameter values for all of supervisor configuration settings. Keys correspond to parameter name, & values to parameter value.                                                                             |
| `/config/<config>`              | `config_value`                                                   | Directly retrieve the value of a supervisor configuration parameter with name `'config'`. Returns `param_value` of `'config'`.                                                                                                              |
| `/connections/<connection>`     | `dict`                                                           | Returns the response of the connection (e.g. an `image_rgb` connection would return the image) as a `dict`. Format & style of the `dict` is defined by the methods described above in "Defining environment, robot, & task configurations". |
| `/results_functions/`           | `list`                                                           | Returns a list of the results function names that can be remotely executed via the route below.                                                                                                                                             |
| `/results_functions/<function>` | `dict`                                                           | Calls results function with name `'function'`, and returns the result of the function call in the response's JSON body.                                                                                                                     |
| `/robot/`                       | <pre>Hello, I am the BenchBot robot controller</pre>             | Arbitrary response confirming a robot controller is available.                                                                                                                                                                              |
| `/robot/<command>`              | `dict`                                                           | Passes the command `command` down to a running robot controller manager. See [BenchBot Robot Controller](https://github.com/roboticvisionorg/benchbot_robot_controller) for documentation of supported commands & expected responses.       |
