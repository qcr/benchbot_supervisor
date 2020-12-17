import argparse

from .benchbot_supervisor import Supervisor, DEFAULT_PORT

if __name__ == '__main__':
    # Parse all supported arguments
    p = argparse.ArgumentParser(description="Runs the BenchBot Supervisor.")
    p.add_argument('--port',
                   help="Supervisor port number (default: 10000)",
                   default=DEFAULT_PORT)
    p.add_argument('--task-file', help="File containing a task specification")
    p.add_argument('--robot-file', help="File containing a robot definition")
    p.add_argument(
        '--environment-files',
        help=
        "List files (colon separated), each file containing the definition for "
        "a selected environment")
    args = p.parse_args()

    # Start the supervisor
    s = Supervisor(port=args.port,
                   task_file=args.task_file,
                   robot_file=args.robot_file,
                   environment_files=args.environment_files)
    s.run()
