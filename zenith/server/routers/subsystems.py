"""This module contains /subsystems routes for the Zenith API."""

from fastapi import APIRouter, HTTPException
from subsystem import SubsystemMap, ROS2CommandException
from pydantic import BaseModel


router = APIRouter()


class Package(BaseModel):
    name: str
    executables: list[str]


class Subsystem(BaseModel):
    name: str
    packages: list[Package]


@router.get("/subsystems")
def root() -> list[Subsystem]:
    try:
        sm = SubsystemMap.get()
    except ROS2CommandException as e:
        raise HTTPException(status_code=500, detail=str(e))

    response: list[Subsystem] = []
    for subsystem, pkg_map in sm.subsystems.items():
        packages: list[Package] = []
        for pkg, executables in pkg_map.items():
            package = Package(name=pkg, executables=executables)
            packages.append(package)
        response.append(Subsystem(name=subsystem, packages=packages))
    return response
