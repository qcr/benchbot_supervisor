from __future__ import print_function

import flask
from gevent import event, pywsgi, signal
import importlib
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
                 addons_path,
                 port=DEFAULT_PORT,
                 task_name=None,
                 results_format_name=None,
                 robot_name=None,
                 environment_names=None):
        # Configuration parameters
        self.supervisor_address = 'http://0.0.0.0:' + str(port)
        self.task_name = task_name
        self.results_format_name = results_format_name
        self.robot_name = robot_name
        self.environment_names = ([] if environment_names is None else
                                  environment_names)
        self.addons_path = addons_path

        # Derived configuration variables
        self.config = None

        # Current state
        self.state = {}
        self.results_functions = {}

        # Attempt to attach to a manager from benchbot_addons (supervisor can't
        # interact with any content without this connect)
        print("Initialising supervisor...")
        sys.path.insert(0, addons_path)
        self.addons = importlib.import_module('benchbot_addons.manager')
        del sys.path[0]

        # Configure the Supervisor with provided arguments
        print("\nConfiguring the supervisor...")
        self.load()

        # At this point we have a running supervisor ready for use
        print("Starting a supervisor with the following configuration:\n")
        pprint.pprint(self.config, depth=3)

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

    def load(self):
        # Load all of the configuration data provided in the selected YAML files
        if self.config is None:
            self.config = self._BLANK_CONFIG.copy()
        self.config['task'] = self.addons.get_match("tasks",
                                                    [("name", self.task_name)],
                                                    return_data=True)
        self.config['results'] = self.addons.get_match(
            "formats", [("name", self.results_format_name)], return_data=True)
        self.config['robot'] = self.addons.get_match(
            "robots", [("name", self.robot_name)], return_data=True)
        self.config['environments'] = [
            self.addons.get_match("environments",
                                  [("name", self.addons.env_name(e)),
                                   ("variant", self.addons.env_variant(e))],
                                  return_data=True)
            for e in self.environment_names
        ]

        # Load the helper functions for results creation
        if 'functions' in self.config['results']:
            sys.path.insert(
                0, os.path.dirname(self.config['results'][FILE_PATH_KEY]))
            self.results_functions = {
                k: getattr(importlib.import_module(re.sub('\.[^\.]*$', "", v)),
                           re.sub('^.*\.', "", v))
                for k, v in self.config['results']['functions'].items()
            }
            del sys.path[0]

        # Perform any required manual cleaning / sanitising of data
        if 'scene_count' not in self.config['task']:
            self.config['task']['scene_count'] = 1
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

        @supervisor_flask.route('/results_functions/', methods=['GET'])
        def __results_functions_list():
            try:
                return flask.jsonify(list(self.results_functions.keys()))
            except Exception as e:
                print("ERROR: Supervisor failed to list available results "
                      "functions with error:\n%s" % repr(e))

        @supervisor_flask.route('/results_functions/<function>',
                                methods=['GET'])
        def __results_function(function):
            try:
                data = flask.request.get_json()
                data = {} if data is None else data
                if 'args' not in data:
                    data['args'] = []
                if 'kwargs' not in data:
                    data['kwargs'] = {}
                return flask.jsonify(self.results_functions[function](
                    *data['args'], **data['kwargs']))
            except Exception as e:
                print("ERROR: Supervisor failed to call results function "
                      "'%s' with error:\n%s" % (function, repr(e)))

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
