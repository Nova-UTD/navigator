"""This module contains the Navigator CLI."""

import argparse
import builtins
import subprocess
import sys


def err_fatal(msg):
    """Prints an error message and exits with status code 1.
    
    @param msg[str]: Error message to print.
    """
    builtins.print(f"[❌ Navigator CLI Error]: {msg}", file=sys.stderr)
    exit(1)


def print(msg):
    """Prints a message with the Navigator CLI prefix.
    
    @param msg[str]: Message to print.
    """
    builtins.print(f"[Navigator CLI]: {msg}")


LAUNCHES = {
    "vehicle": "launches/launch.vehicle.py",
    "carla": "launches/launch.carla.py",
    "perception": "launches/launch.perception.py"
}


def launch(name):
    """Launches a launch file.
    
    @param name[str]: Name of launch file to launch. Must be a key in LAUNCHES.
    
    @raise err_fatal: If the launch name is not found in LAUNCHES, exits with status code 1.
    """

    if not name in LAUNCHES:
        err_fatal("Launch name not found")

    try:
        subprocess.run(["ros2", "launch", LAUNCHES[name]])
    except KeyboardInterrupt:
        print("Launch interrupted")


parser = argparse.ArgumentParser(
    prog="navigator",
    description="Navigator CLI",
)

# Add nested commands (e.g. navigator [cmd] [...args]).
# At least one nested command is required.
subparsers = parser.add_subparsers(required=True)

# Add launch command.
# Usage: navigator launch [launch_name]
launch_parser = subparsers.add_parser("launch", help="launches an existing launch file")
launch_parser.add_argument("launch_name", help="name of launch", choices=LAUNCHES.keys())

if __name__ == "__main__":
    args = parser.parse_args()
    launch(args.launch_name)