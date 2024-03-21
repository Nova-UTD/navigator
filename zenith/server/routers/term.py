from __future__ import annotations

import asyncio
import fcntl
import pty
import os
import termios
import threading
import select
import struct
import subprocess

from typing import Literal, Annotated, Union

from fastapi import APIRouter, WebSocket
from pydantic import BaseModel, Field, TypeAdapter, ValidationError


router = APIRouter()


class Terminal:
    def __init__(self, websocket: WebSocket):
        self.websocket = websocket
        self.master_fd = None

    async def spawn(self):
        child_pid, master_fd = pty.fork()
        self.master_fd = master_fd
        is_child = child_pid == 0
        if is_child:
            self._spawn_shell()
        else:
            asyncio.create_task(self.forward_pty_output())

    async def forward_pty_output(self):
        if not self.master_fd:
            raise Exception("Master fd not set")

        MAX_READ_BYTES = 1024 * 20
        while True:
            await asyncio.sleep(0.01)
            try:
                readable_bits, _, _ = select.select([self.master_fd], [], [], 0)
                if readable_bits:
                    output = os.read(self.master_fd, MAX_READ_BYTES)
                    await self.websocket.send_text(output.decode("utf-8", "ignore"))
            except OSError:
                break

    def resize(self, rows: int, cols: int):
        """
        struct winsize {
            unsigned short ws_row;
            unsigned short ws_col;
            unsigned short ws_xpixel; /* unused */
            unsigned short ws_ypixel; /* unused */
        };
        """
        winsize = struct.pack("HHHH", rows, cols, 0, 0)
        fcntl.ioctl(self.master_fd, termios.TIOCSWINSZ, winsize)

    def write_to_pty(self, data: str):
        os.write(self.master_fd, data.encode())

    def _spawn_shell(self):
        default_shell = os.environ.get("SHELL", "/bin/bash")
        subprocess.run(default_shell)


TERM = None | Terminal
TERM_LOCK = threading.Lock()


class ResizePayload(BaseModel):
    type: Literal["resize"]
    rows: int
    cols: int


class InputMessage(BaseModel):
    type: Literal["input"]
    key: str


Message = Annotated[Union[InputMessage, ResizePayload], Field(discriminator="type")]


class WSErrorMessage(BaseModel):
    error: str


@router.websocket("/term/ws")
async def term_ws(websocket: WebSocket):
    await websocket.accept()

    if TERM_LOCK.locked():
        error = WSErrorMessage(error="Another terminal is already running")
        await websocket.send_json(error.model_dump())
        return

    with TERM_LOCK:
        TERM = Terminal(websocket)
        await TERM.spawn()

        while True:
            raw_msg = await websocket.receive_text()
            try:
                ta = TypeAdapter(Message)
                msg = ta.validate_json(raw_msg)
            except ValidationError as e:
                error = WSErrorMessage(error=f"Invalid message: {e}. Got: {raw_msg}")
                await websocket.send_json(error.model_dump())
                continue

            if isinstance(msg, InputMessage):
                TERM.write_to_pty(msg.key)
            elif isinstance(msg, ResizePayload):
                TERM.resize(msg.rows, msg.cols)
