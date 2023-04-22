#
# Copyright (c) 2022 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
# GNSS global plan

int8 VOID=-1
int8 LEFT=1
int8 RIGHT=2
int8 STRAIGHT=3
int8 LANEFOLLOW=4
int8 CHANGELANELEFT=5
int8 CHANGELANERIGHT=6

std_msgs/Header header

int8[] road_options
sensor_msgs/NavSatFix[] coordinates
