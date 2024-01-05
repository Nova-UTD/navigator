#!/bin/sh
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
cd object_detection/
python3 obj.py
