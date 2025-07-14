"""This module contains /launches routes for the Zenith API."""

import sys
import os
import traceback

from typing import Callable
from http import HTTPStatus

from fastapi import APIRouter, HTTPException, Response, Request
from fastapi.routing import APIRoute
from pydantic import BaseModel, Field

from launch import (
    LaunchFileNode,
    Metadata as LaunchMetadata,
)
from launch import builder


class TracebackRoute(APIRoute):
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


router = APIRouter(route_class=TracebackRoute)


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

                launch_file = builder.LaunchFileBuilder(path)
                if not launch_file.valid():
                    continue

                abs_path = os.path.abspath(path)
                name = launch_file.get_metadata().name
                nodes = launch_file.get_nodes()

                response.append(
                    LaunchEntry(
                        metadata=Metadata(name=name),
                        path=abs_path,
                        nodes=[
                            LaunchNode(package=node.package, executable=node.executable)
                            for node in nodes
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
    except OSError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    try:
        launch_file.set_metadata(LaunchMetadata(name=launch.metadata.name)).set_nodes(
            [
                LaunchFileNode(package=node.package, executable=node.executable)
                for node in launch.nodes
            ]
        ).build_and_write(overwrite=False)
    except builder.UnableToOverwrite as e:
        raise HTTPException(status_code=HTTPStatus.CONFLICT, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e

    return {"path": os.path.abspath(launch.path)}


class UpdateLaunch(BaseModel):
    """! A launch file entry in the launches.
    @param path[str]    The path to the launch file.
    @param metadata[dict]   The launch file metadata.
    @param nodes[list[dict]]   The list of nodes in the launch file.
    """

    path: str
    metadata: Metadata | None = Field(None, description="New launch metadata.")
    add_nodes: list[LaunchNode] | None = Field(
        None, description="Launch nodes to append."
    )
    remove_nodes: list[LaunchNode] | None = Field(
        None, description="Launch nodes to remove"
    )
    new_path: str | None = Field(None, description="New path for the launch file.")


@router.patch("/launches", status_code=204)
def update_launch(update: UpdateLaunch):
    """
    Sample requests:

    Name change:
    {
        "path": "/navigator/launches/launch.x.py",
        "metadata": { "name": "rawr" },
    }

    Add nodes and change path:
    {
        "path": "/navigator/launches/launch.x.py",
        "new_path": "/navigator/launches/launch.x_changed.py"
        "nodes": [ { "package": "p1", "executable": "e1" }, { "package": "p2", "executable": "e2" } ]
    }
    """
    try:
        launch_builder = builder.LaunchFileBuilder(update.path)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    if update.metadata:
        launch_builder.set_metadata(LaunchMetadata(name=update.metadata.name))

    if update.add_nodes:
        for node in update.add_nodes:
            launch_builder.add_node(
                LaunchFileNode(package=node.package, executable=node.executable)
            )

    if update.remove_nodes:
        for node in update.remove_nodes:
            try:
                launch_builder.remove_node(
                    LaunchFileNode(package=node.package, executable=node.executable)
                )
            except ValueError as e:
                # Node is not in the list.
                raise HTTPException(status_code=400, detail=str(e)) from e

    try:
        launch_builder.build_and_write(overwrite=True)
        if update.new_path:
            os.rename(update.path, update.new_path)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e


@router.post("/launches/copy", status_code=201)
def copy_launch(old_path: str, new_path: str, new_name: str, overwrite: bool = True):
    """
    Duplicates existing launch file.

    Sample request:
    URI: /launches?old_path=/navigator/launch.x.py&new_path=/navigator/launch.z.py&new_name=z
    """
    try:
        launch_builder = builder.LaunchFileBuilder(old_path)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e)) from e

    try:
        # Set the new path and write the launch file.
        launch_builder.set_path(new_path).set_metadata(
            LaunchMetadata(name=new_name)
        ).build_and_write(overwrite)
    except builder.UnableToOverwrite as e:
        raise HTTPException(status_code=HTTPStatus.CONFLICT, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e)) from e

    return {"path": os.path.abspath(launch_builder.get_path())}
