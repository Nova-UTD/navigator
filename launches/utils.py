"""This module contains utilities for launch files."""

import sys

from launch.launch_description import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def err_fatal(msg):
    """Prints an error message and exits with status code 1."""
    print(f"[❌ Navigator Launch Error]: {msg}", file=sys.stderr)
    exit(1)


class IncludeError(Exception):
    """Raised when an included launch file is not found."""
    pass


def try_get_launch_description_from_include(file_path: str) -> LaunchDescription:
    """Attempts to get a launch description from an included launch file.

    @param file_path: Path to the launch file to include.
    @return LaunchDescription: Launch description from the included launch file.

    @raise IncludeError: If the launch file is not found.
    """

    # Include vehicle launch file and extract launch entities for reuse.
    included_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(file_path))

    try:
        # First entity in the launch file should be the launch description.
        launch_descrption = included_launch.get_sub_entities()[0]
        return launch_descrption
    except IndexError:
        raise IncludeError(f"launch description '{file_path}' not found. Ensure the launch file is configured properly.")
