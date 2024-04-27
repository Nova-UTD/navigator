"""Module for building and modifying launch files. Prefer this module over launch_file."""

from __future__ import annotations
import os

from .launch_file import LaunchFileBuffer, LaunchFileNode, Metadata
from .template import LAUNCH_FILE_TEMPLATE


class InitException(Exception):
    """Exception raised for errors in the initialization of the LaunchFileBuilder."""
    pass


class BuildException(Exception):
    """Exception raised for errors in the building of the launch file."""
    pass


class UnableToOverwrite(Exception):
    """Exception raised when trying to overwrite a file that already exists."""
    pass


class LaunchFileBuilder:
    """LaunchFileBuilder allows for parsing, creation, and modification of launch files. 

    This should always be used instead of directly modifying the LaunchFileBuffer.
    """

    def __init__(self, path: str):
        """Initializes the LaunchFileBuilder.

        Args:
            path (str): The path to the launch file.

        Raises:
            InitException: If the launch file fails to initialize.
            OSError: If the file cannot be read.
        """
        self.path = path

        # Try to read the file.
        # If it doesn't exist, create a new one from the template.
        try:
            if not os.path.exists(path):
                self.buffer = LaunchFileBuffer(LAUNCH_FILE_TEMPLATE)
                self.metadata = Metadata()
                self.nodes = []
            else:
                file = open(path, "r")
                self.buffer = LaunchFileBuffer(file.read())
                self.metadata = self.buffer._read_metadata()
                self.nodes = self.buffer._read_launch_list()
                file.close()
        except OSError as e:
            raise e
        except Exception as e:
            raise InitException(
                f"Failed to initialize launch file from path: {path}"
            ) from e

    def set_metadata(self, metadata: Metadata) -> LaunchFileBuilder:
        self.metadata = metadata
        return self

    def add_node(self, node: LaunchFileNode) -> LaunchFileBuilder:
        self.nodes.append(node)
        return self

    def remove_node(self, node: LaunchFileNode) -> LaunchFileBuilder:
        self.nodes.remove(node)
        return self

    def set_nodes(self, nodes: list[LaunchFileNode]) -> LaunchFileBuilder:
        self.nodes = nodes
        return self

    def set_path(self, path: str) -> LaunchFileBuilder:
        self.path = path
        return self

    def valid(self) -> bool:
        return self.buffer.valid()

    def get_metadata(self) -> Metadata:
        return self.metadata

    def get_nodes(self) -> list[LaunchFileNode]:
        return self.nodes

    def get_path(self) -> str:
        return self.path

    def build_and_write(self, overwrite=False) -> None:
        """Builds the launch file and writes it to the path.

        Args:
            overwrite (bool, optional): Whether to overwrite the file if it already exists. Defaults to False.
        
        Raises:
            UnableToOverwrite: If the file already exists and overwrite is False.
            BuildException: If the launch file fails to build.
        """
        if not overwrite and os.path.exists(self.path):
            raise UnableToOverwrite(
                "Launch already exists, consider using an overwriteable method"
            )

        try:
            file = open(self.path, "wt")
            file.write(self.buffer.generate_file(self.metadata, self.nodes))
            file.close()
        except Exception as e:
            raise BuildException("Failed to build launch file") from e
