from __future__ import print_function

import flask
from gevent import event, pywsgi, signal
import os
import pprint
import re
import requests
import rospkg
import rospy
import sys
import time
import traceback
import yaml

_PACKAGE_NAME = "benchbot_supervisor"

_SUPERVISOR_PORT = 10000


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
        self.config = None

        # Current state
        self.state = {}

        # Configure the Supervisor with provided arguments
        print("Configuring the supervisor...")
        self.configure(
            task_file=rospy.get_param("~task_file", None),
            task_name=rospy.get_param("~task_name", None),
            robot_file=rospy.get_param("~robot_file", None),
            actions_file=rospy.get_param("~actions_file", None),
            observations_file=rospy.get_param("~observations_file", None),
            environment_files=rospy.get_param("~environment_files", None))

    def _load_config_from_file(self, key):
        # Bit rough... but eh... that's why its hidden
        if self.config is None:
            self.config = self._BLANK_CONFIG.copy()
        if getattr(self, key) is not None:
            self.config[(key[:-5] if key.endswith('_file') else
                         key)] = _open_yaml_file(getattr(self, key), key)

    def _robot(self, command, data=None):
        return (requests.get if data is None else requests.post)(
            re.sub('//$', '/', '%s/' % self.config['robot']['address']) +
            command, **({} if data is None else {
                'json': data
            })).json()

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
            for i, f in enumerate(self.environment_files):
                with open(f, 'r') as fd:
                    d = yaml.safe_load(fd)
                d['order'] = i
                d['path'] = f
                self.environment_data[d['environment_name']] = d
                self.config['environment_names'].append(d['environment_name'])
        if 'start_cmds' in self.config['robot']:
            self.config['robot']['start_cmds'] = [
                c.strip() for c in self.config['robot']['start_cmds']
            ]

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

        @supervisor_flask.route('/connections/<connection>',
                                methods=['GET', 'POST'])
        def __connection_get(connection):
            # TODO there needs to be better error checking for when no message
            # has been received on a ROS topic!!! (at the moment all we get is
            # an unhelpful null & success...)
            if connection not in self.config['robot']['connections']:
                rospy.logerr("Requested undefined connection: %s" % connection)
                flask.abort(404)
            try:
                return self._robot(
                    '/connections/%s' % connection,
                    data=(flask.request.get_json()
                          if flask.request.method == 'POST' else None))
            except Exception as e:
                rospy.logerr("Supervisor failed on processing connection "
                             "'%s' with error:\n%s" % (connection, repr(e)))
                flask.abort(500)

        @supervisor_flask.route('/robot/', methods=['GET'])
        def __robot_check():
            try:
                return flask.jsonify(self._robot('/'))
            except Exception as e:
                rospy.logerr("Supervisor failed to contact the robot: %s" % e)
                flask.abort(500)

        @supervisor_flask.route('/robot/<command>', methods=['GET'])
        def __robot_get(command):
            try:
                return self._robot(command)
            except Exception as e:
                rospy.logerr("Supervisor received the following error when "
                             "issuing command '%s' to the robot: %s" %
                             (command, e))
                flask.abort(500)

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

        print("\nWaiting until a robot controller is found @ '%s' ... " %
              self.config['robot']['address'],
              end='')
        sys.stdout.flush()
        connected = False
        while not connected:
            try:
                self._robot('/')
                connected = True
            except Exception as e:
                pass
            time.sleep(1)
        print("Found")

        # TODO we need to ensure map file is loaded if sending to a remote!
        print("\nSending environment data & robot config to controller ... ",
              end='')
        sys.stdout.flush()
        self._robot('/configure', {
            'environments': self.environment_data,
            'robot': self.config['robot']
        })
        print("Ready\n")

        # Run the server in a blocking manner until the Supervisor is closed
        evt.wait()
        print("\n\nShutting down supervisor & exiting ...")
        supervisor_server.stop()
