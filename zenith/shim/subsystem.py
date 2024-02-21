import os
import subprocess
import sys
import json
from collections import defaultdict

# Alias for a map of package -> list of executables.
PkgMap = defaultdict[str, list[str]]


class SubsystemMap:
    """! A map of subsystems to packages and their executables."""

    subsystems: defaultdict[str, PkgMap]

    def __init__(self) -> None:
        self.subsystems = defaultdict(lambda: defaultdict(list))

        # Get all executables and the packages they belong to.
        pkg_map = get_ros2_pkg_map()
        # Add each package and its executables to the subsystem map.
        for pkg, executables in pkg_map.items():
            subsystem = get_pkg_subsystem(pkg)
            self._add_executables(subsystem, pkg, executables)

    def _add_executables(
        self, subsystem: str, pkg: str, executables: list[str]
    ) -> None:
        """! Add a package and its executables to a subsystem.
        @param subsystem[str]   The subsystem name.
        @param pkg[str]         The package name.
        @param executables[list[str]]   The list of executables.
        """
        self.subsystems[subsystem][pkg] = executables

    def to_json(self) -> str:
        """! Convert the subsystem map to JSON.
        @return str  The JSON string."""
        return json.dumps(self.subsystems, indent=4)


def get_ros2_pkg_map() -> PkgMap:
    """! Get all executables and the packages they belong to.
    @return PkgMap   Mapping from package -> list of executables.
    """
    try:
        # Sample command output:
        #  pkg1 exec1
        #  pkg1 exec2
        #  pkg2 exec1
        #  pkg3 exec1
        # Convert to map:
        #  pkg1: [exec1, exec2]
        #  pkg2: [exec1]
        #  pkg3: [exec1]
        pkg_executables = (
            subprocess.check_output(
                ["ros2", "pkg", "executables"],
                stderr=subprocess.STDOUT,
            )
            .decode("utf-8")
            .splitlines()
        )

        # Mapping from package -> list of executables.
        pkg_map: PkgMap = defaultdict(list)
        for line in pkg_executables:
            pkg, executable = line.split()
            pkg_map[pkg].append(executable)

        return pkg_map
    except subprocess.CalledProcessError as e:
        print(
            f"Zenith shim error: 'ros2 pkg executables' failed with exit code: {e.returncode}: '{e.output}'. Ensure that ROS2 is installed and sourced",
            file=sys.stderr,
        )
        sys.exit(1)


def find_directory(component: str, root: str = ".") -> str | None:
    """! Find a directory in the root directory.
    @param component[str]   The directory to find.
    @param root[str]        The root directory to search in.
    @return str | None      The path to the directory if found, None otherwise.
    """
    for path, dirs, _ in os.walk(root):
        if component in dirs:
            return os.path.join(path, component)


def get_pkg_subsystem(pkg: str) -> str:
    """! Get the subsystem of a package.
    @param pkg[str]  The package name.
    @return str      The subsystem name.
    """
    pkg_path = find_directory(pkg, root="../src")
    # If the package is not in the source directory, it must have been
    # installed externally.
    if pkg_path is None:
        return "system-install"

    # Remove last component of the path (which is the package name)
    subsystem_path = pkg_path.split("/")[:-1]
    subsystem = "misc"  # Default subsystem to "misc" for miscellaneous.
    # Traverse backwards until we find the "src" directory.
    # Any children directories of "src" are considered subsystems.
    while len(subsystem_path) > 0 and subsystem_path[-1] != "src":
        subsystem = subsystem_path[-1]
        subsystem_path = subsystem_path[:-1]

    return subsystem
