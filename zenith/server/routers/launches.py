import sys
import os
import traceback

from typing import Callable
from http import HTTPStatus

from fastapi import APIRouter, HTTPException, Response, Request
from fastapi.routing import APIRoute
from pydantic import BaseModel

from launch import (
    LaunchFileFromExisting,
    LaunchFileFromScratch,
    LaunchFileNode,
    Metadata as LaunchMetadata,
)
from launch import builder


class PrometheusRoute(APIRoute):
    def get_route_handler(self) -> Callable:
        original_route_handler = super().get_route_handler()

        async def custom_route_handler(request: Request) -> Response:
            app = request.app
            try:
                return await original_route_handler(request)
            except HTTPException as exc:
                exc_type, exc_value, exc_tb = sys.exc_info()
                error = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
                app.logger.warning("{}".format(error))
                raise exc

        return custom_route_handler


router = APIRouter(route_class=PrometheusRoute)


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


class LaunchCreate(BaseModel):
    """! A launch file entry in the launches.
    @param path[str]   The path to the launch file.
    @param metadata[dict]   The launch file metadata.
    @param nodes[list[dict]]   The list of nodes in the launch file.
    """

    path: str
    metadata: Metadata
    nodes: list[LaunchNode]


@router.post("/launches", status_code=201)
def create_launch(launch: LaunchCreate):
    """
    Sample request:
    {
      "metadata": {
        "name": "Launch File"
      },
      "nodes": [
        {
          "package": "joy_translation",
          "executable": "joy_translation_node"
        },
        {
          "package": "rviz",
          "executable": "rviz"
        }
      ]
    }
    """
    try:
        launch_file = builder.LaunchFileBuilder(launch.path)
        launch_file.set_metadata(LaunchMetadata(name=launch.metadata.name)).set_nodes(
            [
                LaunchFileNode(package=node.package, executable=node.executable)
                for node in launch.nodes
            ]
        ).build()
    except builder.UnableToOverwrite as e:
        raise HTTPException(status_code=HTTPStatus.CONFLICT, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    return {"path": os.path.abspath(launch.path)}


class UpdateLaunch(BaseModel):
    """! A launch file entry in the launches.
    @param path[str]    The path to the launch file.
    @param metadata[dict]   The launch file metadata.
    @param nodes[list[dict]]   The list of nodes in the launch file.
    """

    path: str
    metadata: Metadata | None
    nodes: list[LaunchNode] | None


@router.patch("/launches", status_code=204)
def update_launch(update: UpdateLaunch):
    try:
        launch_builder = builder.LaunchFileBuilder(update.path, overwrite=True)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    if update.metadata:
        launch_builder.set_metadata(LaunchMetadata(name=update.metadata.name))

    if update.nodes:
        for node in update.nodes:
            launch_builder.add_node(
                LaunchFileNode(package=node.package, executable=node.executable)
            )

    try:
        launch_builder.build()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e
