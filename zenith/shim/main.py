import argparse
import sys

from subsystem import SubsystemMap
from launch import LaunchFileFromExisting, Metadata, LaunchFileNode

from command_handlers import create, list

parser = argparse.ArgumentParser(
    prog="navigator",
    description="Navigator CLI",
)

# Add nested commands (e.g. navigator [cmd] [...args]).
# At least one nested command is required.
subparsers = parser.add_subparsers(required=True, dest="action")

# Add launch command.
# Usage: navigator launch [launch_name]
subsystem_parser = subparsers.add_parser("subsystem", help="launches an existing launch file")

launch_parser = subparsers.add_parser("launch", help="launches an existing launch file")
launch_subparser = launch_parser.add_subparsers(required=True, dest="launch_action")
launch_subparser.add_parser("add-node", help="add a node to a launch file")
launch_subparser.add_parser("remove-node", help="remove a node in a launch file")
launch_subparser.add_parser("list", help="list valid launch files")

launch_create_subparser = launch_subparser.add_parser("create", help="create a new launch file")
launch_create_subparser.add_argument("path", help="path of launch file to create")
launch_create_subparser.add_argument("metadata", help=f"JSON of metadata: {Metadata.help()}")
launch_create_subparser.add_argument("nodes", help=f"JSON node list: [{LaunchFileNode.help()}]")

launch_list_parser = launch_subparser.add_parser("list", help="load a launch file")
launch_list_parser.add_argument("dir", help="directory of launch files")

launch_load_parser = launch_subparser.add_parser("load", help="load a launch file")
launch_load_parser.add_argument("file", help="path to launch file")


def main():
    args = parser.parse_args()
    if args.action == "subsystem":
        print(SubsystemMap().to_json(), file=sys.stdout)
    elif args.action == "launch":
        if args.launch_action == "create":
            create(args.path, args.metadata, args.nodes)
        if args.launch_action == "list":
            list(args.dir)
        if args.launch_action == "add-node":
            print("add-node")
        elif args.launch_action == "remove-node":
            print("remove-node")
        elif args.launch_action == "load":
            file = open(args.file, "r")
            contents = file.read()
            launch_file = LaunchFileFromExisting(contents)
            print(launch_file.to_json())
            file.close()

if __name__ == '__main__':
    main()