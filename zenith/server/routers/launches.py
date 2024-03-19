import os

from fastapi import APIRouter
from pydantic import BaseModel

from launch import LaunchFileFromExisting


router = APIRouter()


class LaunchNode(BaseModel):
    """! A launch file node in the launches.
    @param package[str]   The package name.
    @param executable[str]   The executable name.
    """

    package: str
    executable: str


class Metadata(BaseModel):
    name: str


class LaunchEntry(BaseModel):
    """! A launch file entry in the launches.
    @param metadata[dict]   The launch file metadata.
    @param nodes[list[dict]]   The list of nodes in the launch file.
    """

    metadata: Metadata
    path: str
    nodes: list[LaunchNode]


@router.get("/launches")
def root(dir: str) -> list[LaunchEntry]:
    """
    Sample response:
    [
      {
        "metadata": {
          "name": "Launch File"
        },
        "path": "/home/gekevin/dev/nova/navigator/launches/tmp.py",
        "nodes": [
          {
            "package": "joy_translation",
            "executable": "joy_translation_node"
          },
          {
            "package": "rviz",
            "executable": "rviz"
          },
      },
    ]
    """
    response: list[LaunchEntry] = []

    for dirpath, _, filenames in os.walk(dir):
        for filename in filenames:
            if filename.endswith(".py"):
                path = os.path.join(dirpath, filename)
                with open(path) as f:
                    launch_file = LaunchFileFromExisting(f.read())
                    if not launch_file.valid():
                        continue

                    abs_path = os.path.abspath(path)

                    response.append(
                        LaunchEntry(
                            metadata=Metadata(name=launch_file.metadata.name),
                            path=abs_path,
                            nodes=[
                                LaunchNode(
                                    package=node.package, executable=node.executable
                                )
                                for node in launch_file.nodes
                            ],
                        )
                    )

    return response
