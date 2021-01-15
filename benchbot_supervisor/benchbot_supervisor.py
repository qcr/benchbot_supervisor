from __future__ import print_function

import flask
from gevent import event, pywsgi, signal
import os
import pprint
import re
import requests
import sys
import time
import yaml

DEFAULT_PORT = 10000

FILE_PATH_KEY = '_file_path'


def _merge_dicts(dict_1, dict_2):
    # Note: duplicate keys in dict_2 will overwrite dict_1
    out = dict_1.copy()
    out.update(dict_2)
    return out


# TODO: this does not clean up ROS publishers / subscribers properly. Will need
# to do this if configurations plan to be changed dynamically
class Supervisor(object):
    _BLANK_CONFIG = {'environments': [], 'robot': {}, 'task': {}}

    def __init__(self,
                 port=DEFAULT_PORT,
                 task_file=None,
                 task_name=None,
                 result_format_file=None,
                 robot_file=None,
                 actions_file=None,
                 observations_file=None,
                 environment_files=None):
        print("Initialising supervisor...")

        # Configuration parameters (these are mostly set by the 'configure()'
        # function, but we need to declare them as class members here)
        self.supervisor_address = 'http://0.0.0.0:' + str(port)
        self.task_file = None
        self.result_format_file = None
        self.robot_file = None
        self.environment_files = None

        # Derived configuration variables
        self.config = None
        self.environment_data = None

        # Current state
        self.state = {}

        # Configure the Supervisor with provided arguments
        print("Configuring the supervisor...")
        self.configure(task_file, result_format_file, robot_file,
                       environment_files)

    def _load_config_from_file(self, key, files, force_list=False):
        if not isinstance(files, list):
            files = [files]
        use_list = force_list or len(files) > 1

        data = []
        for f in files:
            with open(f, 'r') as fp:
                data.append(yaml.safe_load(fp))
            data[-1][FILE_PATH_KEY] = f

        if self.config is None:
            self.config = self._BLANK_CONFIG.copy()
        self.config[key] = data if use_list else data[0]

    def _robot(self, command, data=None):
        return (requests.get if data is None else requests.post)(
            re.sub('//$', '/', '%s/' % self.config['robot']['address']) +
            command, **({} if data is None else {
                'json': data
            })).json()

    def configure(self, task_file, result_format_file, robot_file,
                  environment_files):
        self.task_file = task_file
        self.result_format_file = result_format_file
        self.robot_file = robot_file
        self.environment_files = (None if environment_files is None else
                                  environment_files.split(':'))

        self.load()

        print("Starting a supervisor with the following configuration:\n")
        pprint.pprint(self.config, depth=3)

    def load(self):
        # Load all of the configuration data provided in the selected YAML files
        self._load_config_from_file('task', self.task_file)
        self._load_config_from_file('results', self.result_format_file)
        self._load_config_from_file('robot', self.robot_file)
        self._load_config_from_file('environments',
                                    self.environment_files,
                                    force_list=True)

        # Perform any required manual cleaning / sanitising of data
        if 'start_cmds' in self.config['robot']:
            self.config['robot']['start_cmds'] = [
                c.strip() for c in self.config['robot']['start_cmds']
            ]
        if 'address' not in self.config['robot']:
            raise ValueError("ERROR: No address was received for the robot!")
        elif not self.config['robot']['address'].startswith("http://"):
            self.config['robot']['address'] = ('http://' +
                                               self.config['robot']['address'])

        # Confirm the requested robot can satisfy all actions & observations
        # requested by the task
        for x in (self.config['task']['actions'] +
                  self.config['task']['observations']):
            if (self.config['robot']['connections'] is None
                    or x not in self.config['robot']['connections']):
                raise ValueError(
                    "The task '%s' requires an action / observation called '%s',"
                    " which isn't declared for the robot in file '%s'" %
                    (self.config['task']['name'], x, self.robot_file))

    def run(self):
        # Setup all of the supervisor managament functions
        supervisor_flask = flask.Flask(__name__)

        @supervisor_flask.route('/', methods=['GET'])
        def __hello():
            return flask.jsonify("Hello, I am the BenchBot supervisor")

        @supervisor_flask.route('/config/', methods=['GET'])
        def __config_full():
            return flask.jsonify(self.config)

        @supervisor_flask.route('/config/<path:config>', methods=['GET'])
        def __config(config):
            c = self.config
            for k in config.split('/'):
                if k in c:
                    c = c[k]
                else:
                    print("ERROR: Requested non-existent config: %s" % config)
                    flask.abort(404)
            return flask.jsonify(c)

        @supervisor_flask.route('/connections/<connection>',
                                methods=['GET', 'POST'])
        def __connection_get(connection):
            # TODO there needs to be better error checking for when no message
            # has been received on a ROS topic!!! (at the moment all we get is
            # an unhelpful null & success...)
            if connection not in self.config['robot']['connections']:
                print("ERROR: Requested undefined connection: %s" % connection)
                flask.abort(404)
            try:
                return self._robot(
                    '/connections/%s' % connection,
                    data=(flask.request.get_json()
                          if flask.request.method == 'POST' else None))
            except Exception as e:
                print("ERROR: Supervisor failed on processing connection "
                      "'%s' with error:\n%s" % (connection, repr(e)))
                flask.abort(500)

        @supervisor_flask.route('/robot/', methods=['GET'])
        def __robot_check():
            try:
                return flask.jsonify(self._robot('/'))
            except Exception as e:
                print("ERROR: Supervisor failed to contact the robot: %s" % e)
                flask.abort(500)

        @supervisor_flask.route('/robot/<command>', methods=['GET'])
        def __robot_get(command):
            try:
                return self._robot(command)
            except Exception as e:
                print("ERROR: Supervisor received the following error when "
                      "issuing command '%s' to the robot: %s" % (command, e))
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
              self.config['robot']['address'])
        connected = False
        while not connected:
            try:
                self._robot('/')
                connected = True
            except Exception as e:
                pass
            time.sleep(1)
        print("\tFound")

        # TODO we need to ensure map file is loaded if sending to a remote!
        print("Sending environment data & robot config to controller ... ")
        self._robot('/configure', self.config)
        print("\tReady\n")

        # Run the server in a blocking manner until the Supervisor is closed
        evt.wait()
        print("\n\nShutting down supervisor & exiting ...")
        supervisor_server.stop()
