import asyncio
import fcntl
import pty
import os
import termios
import threading
import select
import struct
import subprocess

from typing import Literal

from fastapi import APIRouter, WebSocket
from pydantic import BaseModel


router = APIRouter()


def set_winsize(fd, row, col, xpix=0, ypix=0):
    winsize = struct.pack("HHHH", row, col, xpix, ypix)
    fcntl.ioctl(fd, termios.TIOCSWINSZ, winsize)


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
            set_winsize(master_fd, 20, 20)
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
                    await self.websocket.send_text(output.decode())
                    await self.websocket.send_bytes(output)
            except OSError:
                break

    def write_to_pty(self, data: str):
        os.write(self.master_fd, data.encode())

    def _spawn_shell(self):
        default_shell = os.environ.get("SHELL", "/bin/bash")
        subprocess.run(default_shell)


TERM = None | Terminal
TERM_LOCK = threading.Lock()


class Message(BaseModel):
    type: Literal["input"] | Literal["resize"] | Literal["close"]
    data: str


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
            msg: Message = await websocket.receive_json()

            if msg.type == "input":
                input_data = msg.type
                TERM.write_to_pty(input_data)
