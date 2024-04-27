from __future__ import annotations
import importlib
from typing import TypeVar
import numpy as np

T = TypeVar('T')

NATIVE_CLASSES: dict[str, type] = {}

def to_native(msg: object) -> object:
    """Convert rosbags message to native message.

    Args:
        msg: Rosbags message.

    Returns:
        Native message.

    """
    #print(dir(msg))
    msgtype: str = msg.__msgtype__  # type: ignore[attr-defined]
    if msgtype not in NATIVE_CLASSES:
        pkg, name = msgtype.rsplit('/', 1)
        NATIVE_CLASSES[msgtype] = getattr(importlib.import_module(pkg.replace('/', '.')), name)

    fields = {}
    for name, field in msg.__dataclass_fields__.items():  # type: ignore[attr-defined]
        if 'ClassVar' in field.type:
            continue
        value = getattr(msg, name)
        # I had to switch around the order of the if/else clauses to get it working
        if isinstance(value, list):
            value = [to_native(x) for x in value]
        elif '__msg__' in field.type:
            value = to_native(value)
        elif isinstance(value, np.ndarray):
            value = value.tolist()
        fields[name] = value

    return NATIVE_CLASSES[msgtype](**fields)