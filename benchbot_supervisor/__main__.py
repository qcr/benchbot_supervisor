import argparse

from .benchbot_supervisor import Supervisor, DEFAULT_PORT

if __name__ == '__main__':
    # Parse all supported arguments
    p = argparse.ArgumentParser(description="Runs the BenchBot Supervisor.")
    p.add_argument('--port',
                   help="Supervisor port number (default: 10000)",
                   default=DEFAULT_PORT)
    p.add_argument('--task-file', help="File containing a task specification")
    p.add_argument(
        '--results-format-file',
        help="File containing specification for the task's results format")
    p.add_argument('--robot-file', help="File containing a robot definition")
    p.add_argument(
        '--environment-files',
        help=
        "List files (colon separated), each file containing the definition for "
        "a selected environment")
    p.add_argument(
        '--addons-path',
        help="Path where the benchbot_addons Python package can be found")
    args = p.parse_args()

    # Start the supervisor
    s = Supervisor(port=args.port,
                   task_file=args.task_file,
                   results_format_file=args.results_format_file,
                   robot_file=args.robot_file,
                   environment_files=args.environment_files,
                   addons_path=args.addons_path)
    s.run()
