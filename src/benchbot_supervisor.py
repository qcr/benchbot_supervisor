from copy import deepcopy
import flask
from gevent import event, pywsgi, signal
import importlib
import os
import pprint
import re
import requests
import rospkg
import rospy
import time
import traceback
import tf2_ros
import threading
import yaml

_PACKAGE_NAME = "benchbot_supervisor"

_SUPERVISOR_PORT = 10000

CONN_API_TO_ROS = 'api_to_ros'
CONN_ROS_TO_API = 'ros_to_api'
CONN_ROSCACHE_TO_API = 'roscache_to_api'

CONNS = [CONN_API_TO_ROS, CONN_ROS_TO_API, CONN_ROSCACHE_TO_API]


def _merge_dicts(dict_1, dict_2):
    # Note: duplicate keys in dict_2 will overwrite dict_1
    out = dict_1.copy()
    out.update(dict_2)
    return out


def _open_yaml_file(filename, key=None):
    # Accepts default folder, relative to root of this package, or absolute
    # path
    if filename.startswith("/"):
        abs_file_path = filename
    elif filename.startswith("./"):
        abs_file_path = os.path.join(rospkg.RosPack().get_path(_PACKAGE_NAME),
                                     filename[2:])
    elif key is not None and key.endswith('_file'):
        abs_file_path = os.path.join(
            rospkg.RosPack().get_path(_PACKAGE_NAME),
            re.match('^(.*?)s*_file', key).groups()[0] + 's', filename)
    else:
        return None
    with open(abs_file_path, 'r') as f:
        return yaml.safe_load(f)


def _to_simple_dict(data):
    out = {}
    if hasattr(data, '__slots__'):
        for k in data.__slots__:
            if hasattr(getattr(data, k), '__slots__'):
                out[k] = _to_simple_dict(getattr(data, k))
            else:
                out[k] = getattr(data, k)
    else:
        out = data
    return out


# TODO: this does not clean up ROS publishers / subscribers properly. Will need
# to do this if configurations plan to be changed dynamically
class Supervisor(object):
    _BLANK_CONFIG = {
        'actions': [],
        'environment_names': [],
        'observations': [],
        'robot': {},
        'task_name': ''
    }

    def __init__(self, port=_SUPERVISOR_PORT):
        print("Initialising supervisor...")
        # Configuration parameters
        self.supervisor_address = 'http://0.0.0.0:' + str(port)
        self.task_file = None
        self.task_name = None
        self.robot_file = None
        self.actions_file = None
        self.observations_file = None
        self.environment_files = None
        self.environment_data = None
        self.environment_name = None  # Name of currently loaded environment
        self.config = None

        # Current state
        self.connections = {}
        self.state = {}
        self.tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Configure the Supervisor with provided arguments
        print("Configuring the supervisor...")
        self.configure(
            task_file=rospy.get_param("~task_file", None),
            task_name=rospy.get_param("~task_name", None),
            robot_file=rospy.get_param("~robot_file", None),
            actions_file=rospy.get_param("~actions_file", None),
            observations_file=rospy.get_param("~observations_file", None),
            environment_files=rospy.get_param("~environment_files", None))

    @staticmethod
    def _attempt_connection_imports(connection_data):
        topic_class = None
        if 'ros_type' in connection_data:
            x = connection_data['ros_type'].split('/')
            topic_class = getattr(importlib.import_module(x[0] + '.msg'), x[1])

        callback_supervisor_fn = None
        if 'callback_supervisor' in connection_data:
            callback_supervisor_fn = Supervisor._dynamic_callback_import(
                connection_data['callback_supervisor'])

        callback_caching_fn = None
        if 'callback_caching' in connection_data:
            callback_caching_fn = Supervisor._dynamic_callback_import(
                connection_data['callback_caching'])

        return (topic_class, callback_supervisor_fn, callback_caching_fn)

    @staticmethod
    def _dynamic_callback_import(callback_string):
        x = callback_string.rsplit('.', 1)
        return getattr(importlib.import_module(x[0]), x[1])

    def _call_connection(self, connection_name, data=None):
        if (self.connections[connection_name]['type'] in [
                CONN_ROS_TO_API, CONN_ROSCACHE_TO_API
        ]):
            # Overwrite the data because it is an observation (data should be
            # none anyway with an observation as we do not 'parameterise' an
            # observation)
            self.connections[connection_name]['condition'].acquire()
            data = deepcopy(self.connections[connection_name]['data'])
            self.connections[connection_name]['condition'].release()

            return (data
                    if self.connections[connection_name]['callback_supervisor']
                    is None else
                    self.connections[connection_name]['callback_supervisor'](
                        data, self))
        elif self.connections[connection_name]['type'] == CONN_API_TO_ROS:
            if self.connections[connection_name]['callback_supervisor'] is None:
                self.connections[connection_name]['ros'].publish(data)
            else:
                self.connections[connection_name]['callback_supervisor'](
                    data, self.connections[connection_name]['ros'], self)
        else:
            print("UNIMPLEMENTED CONNECTION CALL: %s" %
                  self.connections[connection_name]['type'])

    def _generate_subscriber_callback(self, connection_name):

        def __cb(data):
            if (self.connections[connection_name]['type'] ==
                    CONN_ROSCACHE_TO_API):
                data = self.connections[connection_name]['callback_caching'](
                    data, self.connections[connection_name]['data'])
            self.connections[connection_name]['condition'].acquire()
            self.connections[connection_name]['data'] = data
            self.connections[connection_name]['condition'].notify()
            self.connections[connection_name]['condition'].release()

        return __cb

    def _is_finished(self):
        return (False if 'trajectory_pose_next' not in self.environment_data[
            self.environment_name] else
                self.environment_data[self.environment_name]
                ['trajectory_pose_next'] >= len(self.environment_data[
                    self.environment_name]['trajectory_poses']))

    def _load_config_from_file(self, key):
        # Bit rough... but eh... that's why its hidden
        if self.config is None:
            self.config = self._BLANK_CONFIG.copy()
        if getattr(self, key) is not None:
            self.config[(key[:-5] if key.endswith('_file') else
                         key)] = _open_yaml_file(getattr(self, key), key)

    def _query_robot(self, command):
        return requests.get(self.config['robot']['address'] + (
            '' if self.config['robot']['address'].endswith('/') else '/') +
                            command).json()

    def _register_connection(self, connection_name, connection_data):
        # Pull out imported components from the connection data
        topic_class, callback_supervisor_fn, callback_caching_fn = (
            Supervisor._attempt_connection_imports(connection_data))

        # Register the connection with the supervisor
        self.connections[connection_name] = {
            'type': connection_data['connection'],
            'callback_supervisor': callback_supervisor_fn,
            'callback_caching': callback_caching_fn,
            'ros': None,
            'data': None,
            'condition': threading.Condition()
        }

        # Construct connections if possible
        if topic_class != None:
            if connection_data['connection'] in [
                    CONN_ROS_TO_API, CONN_ROSCACHE_TO_API
            ]:
                self.connections[connection_name]['ros'] = rospy.Subscriber(
                    connection_data['ros_topic'], topic_class,
                    self._generate_subscriber_callback(connection_name))
            elif connection_data['connection'] == CONN_API_TO_ROS:
                self.connections[connection_name]['ros'] = rospy.Publisher(
                    connection_data['ros_topic'], topic_class, queue_size=1)
            else:
                print("UNIMPLEMENTED POST CONNECTION: %s" %
                      connection_data['connection'])

    def configure(self,
                  task_file=None,
                  task_name=None,
                  robot_file=None,
                  actions_file=None,
                  observations_file=None,
                  environment_files=None):
        self.task_file = task_file
        self.task_name = task_name
        self.robot_file = robot_file
        self.actions_file = actions_file
        self.observations_file = observations_file
        self.environment_files = (None if environment_files is None else
                                  environment_files.split(':'))

        self.load()

        print("Starting a supervisor with the following configuration:\n")
        pprint.pprint(self.config)

    def load(self):
        # Process all of the configuration parameters
        # Note: task file is loaded first, then anything specified explicitly
        # (i.e. every other parameter) is loaded overwriting values from the
        # task_file
        self.config = self._BLANK_CONFIG.copy()
        for k in [
                'task_file', 'robot_file', 'actions_file', 'observations_file'
        ]:
            self._load_config_from_file(k)
        if self.task_name is not None:
            self.config['task_name'] = self.task_name
        if self.environment_files is not None:
            self.environment_data = {}
            self.config['environment_names'] = []
            for f in self.environment_files:
                with open(f, 'r') as fd:
                    d = yaml.safe_load(fd)
                self.environment_data[d['environment_name']] = d
                self.config['environment_names'].append(d['environment_name'])

        # Ensure we have a usable robot address
        if 'address' not in self.config['robot']:
            raise ValueError("ERROR: No address was received for the robot!")
        elif not self.config['robot']['address'].startswith("http://"):
            self.config['robot']['address'] = ('http://' +
                                               self.config['robot']['address'])

        # Validate that we can satisfy all action & observation requests
        for x in self.config['actions'] + self.config['observations']:
            if (self.config['robot']['connections'] is None or
                    x not in self.config['robot']['connections']):
                raise ValueError(
                    "An action / observation was defined using the connection '%s',"
                    " which was not declared for the robot in file '%s'" %
                    (x, self.robot_file))

        # Update ROS connections
        for k, v in self.config['robot']['connections'].items():
            if 'connection' in v and v['connection'] in CONNS:
                self._register_connection(k, v)
            else:
                raise ValueError(
                    "Robot connection definition %s has "
                    "unsupported connection type: %s" %
                    (k, v['connection'] if 'connection' in v else None))

    def run(self):
        # Setup all of the supervisor managament functions
        supervisor_flask = flask.Flask(__name__)

        @supervisor_flask.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify("Hello, I am the BenchBot supervisor")

        @supervisor_flask.route('/config/', methods=['GET'])
        def __config_full():
            return flask.jsonify(self.config)

        @supervisor_flask.route('/config/<config>', methods=['GET'])
        def __config(config):
            if config in self.config:
                return flask.jsonify(self.config[config])
            else:
                rospy.logerr("Requested non-existent config: %s" % config)
                flask.abort(404)

        @supervisor_flask.route('/connections/<connection>', methods=['GET'])
        def __connection_get(connection):
            # TODO there needs to be better error checking for when no message
            # has been received on a ROS topic!!! (at the moment all we get is
            # an unhelpful null & success...)
            if connection not in self.connections:
                rospy.logerr("Requested non-existent connection: %s" %
                             connection)
                flask.abort(404)
            try:
                return flask.jsonify(
                    _to_simple_dict(
                        self._call_connection(
                            connection,
                            data=flask.request.get_json(silent=True))))
            except Exception as e:
                rospy.logerr("Supervisor failed on processing connection "
                             "'%s' with error:\n%s" % (connection, repr(e)))
                flask.abort(500)

        @supervisor_flask.route('/robot/', methods=['GET'])
        def __robot_check():
            try:
                return flask.jsonify(
                    requests.get(self.config['robot']['address']).json())
            except Exception as e:
                rospy.logerr("Supervisor failed to contact the robot: %s" % e)
                flask.abort(500)

        @supervisor_flask.route('/robot/<command>', methods=['GET'])
        def __robot_get(command):
            try:
                resp = self._query_robot(command)
                if (resp.values()[0] and
                        command in ['next', 'reset', 'restart']):
                    self.environment_name = (
                        self.config['environment_names'][self._query_robot(
                            'map_selection_number')['map_selection_number']])
                    self.environment_data[
                        self.environment_name]['trajectory_pose_next'] = 0
                return resp
            except Exception as e:
                rospy.logerr("Supervisor received the following error when "
                             "issuing command '%s' to the robot: %s" %
                             (command, e))
                flask.abort(500)

        @supervisor_flask.route('/status/<command>', methods=['GET'])
        def __status_get(command):
            if self.environment_name is None:
                self.environment_name = (
                    self.config['environment_names'][self._query_robot(
                        'map_selection_number')['map_selection_number']])
            if command == 'is_finished':
                return flask.jsonify({'is_finished': self._is_finished()})
            elif command == 'environment_name':
                return flask.jsonify(
                    {'environment_name': self.environment_name})
            else:
                rospy.logerr("Requested non-existent status: %s" % command)
                flask.abort(404)

        # Configure our server
        supervisor_server = pywsgi.WSGIServer(
            re.split('http[s]?://', self.supervisor_address)[-1],
            supervisor_flask)
        evt = event.Event()
        signal.signal(signal.SIGINT, evt.set)
        signal.signal(signal.SIGQUIT, evt.set)
        signal.signal(signal.SIGTERM, evt.set)

        # Start the server & wait until the robot controller is contactable
        supervisor_server.start()
        print("\n\nSupervisor is now available @ '%s' ..." %
              self.supervisor_address)

        print("\nWaiting until a robot controller is found @ '%s' ..." %
              self.config['robot']['address'])
        connected = False
        while not connected:
            try:
                self._query_robot('/')
                connected = True
            except:
                pass
            time.sleep(3)
        print("READY")

        # Run the server in a blocking manner until the Supervisor is closed
        evt.wait()
        print("\n\nShutting down supervisor & exiting ...")
        supervisor_server.stop()
