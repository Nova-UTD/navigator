#!/bin/sh
cd models/research/
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
cd object_detection/
python3 objDec.py
