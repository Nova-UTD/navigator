import subprocess
import re

def launch_parse(message: str) -> list:
    level = message.split(']')[0][1:]
    node = message.split(']')[1][2:]
    info = message.split(': ')[1]
    print(level + " | " + node + " | " + info)
    return [level, node, info]

def start_main_launch() -> None:
    process = subprocess.run(
        "ros2 launch ros_launch.py",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True
    )
    for line in process.stdout.splitlines():
        launch_parse(line.decode('utf-8'))

if __name__ == "__main__":
    start_main_launch()