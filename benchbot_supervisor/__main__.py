import argparse

from .benchbot_supervisor import Supervisor, DEFAULT_PORT

if __name__ == '__main__':
    # Parse all supported arguments
    p = argparse.ArgumentParser(description="Runs the BenchBot Supervisor.")
    p.add_argument('--port',
                   help="Supervisor port number (default: 10000)",
                   default=DEFAULT_PORT)
    p.add_argument('--task-name', help="Name of task to run")
    p.add_argument('--robot-name', help="Name of the robot to run")
    p.add_argument(
        '--environment-names',
        help="List of environment scenes to run wit this task (comma-separated)"
    )
    p.add_argument(
        '--addons-path',
        help="Path where the benchbot_addons Python package can be found")
    args = p.parse_args()

    # Start the supervisor
    s = Supervisor(port=args.port,
                   task_name=args.task_name,
                   robot_name=args.robot_name,
                   environment_names=args.environment_names.split(','),
                   addons_path=args.addons_path)
    s.run()
