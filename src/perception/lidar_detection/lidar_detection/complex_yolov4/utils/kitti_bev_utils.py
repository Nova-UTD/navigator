"""
# -*- coding: utf-8 -*-
-----------------------------------------------------------------------------------
# Refer: https://github.com/ghimiredhikura/Complex-YOLOv3
"""

import sys
import numpy as np

sys.path.append('../')

import lidar_detection.complex_yolov4.utils.kitti_config as cnf


def removePoints(PointCloud, BoundaryCond):
    # Boundary condition
    minX = BoundaryCond['minX']
    maxX = BoundaryCond['maxX']
    minY = BoundaryCond['minY']
    maxY = BoundaryCond['maxY']
    minZ = BoundaryCond['minZ']
    maxZ = BoundaryCond['maxZ']

    # Remove the point out of range x,y,z
    mask = np.where((PointCloud[:, 0] >= minX) & (PointCloud[:, 0] <= maxX) & (PointCloud[:, 1] >= minY) & (
            PointCloud[:, 1] <= maxY) & (PointCloud[:, 2] >= minZ) & (PointCloud[:, 2] <= maxZ))
    PointCloud = PointCloud[mask]

    PointCloud[:, 2] = PointCloud[:, 2] - minZ

    return PointCloud


def makeBVFeature(PointCloud_, Discretization, bc):
    Height = cnf.BEV_HEIGHT + 1
    Width = cnf.BEV_WIDTH + 1

    # Discretize Feature Map
    PointCloud = np.copy(PointCloud_)
    PointCloud[:, 0] = np.int_(np.floor(PointCloud[:, 0] / Discretization))
    PointCloud[:, 1] = np.int_(np.floor(PointCloud[:, 1] / Discretization) + Width / 2)

    # sort-3times
    indices = np.lexsort((-PointCloud[:, 2], PointCloud[:, 1], PointCloud[:, 0]))
    PointCloud = PointCloud[indices]

    # Height Map
    heightMap = np.zeros((Height, Width))

    _, indices = np.unique(PointCloud[:, 0:2], axis=0, return_index=True)
    PointCloud_frac = PointCloud[indices]
    # some important problem is image coordinate is (y,x), not (x,y)
    max_height = float(np.abs(bc['maxZ'] - bc['minZ']))
    heightMap[np.int_(PointCloud_frac[:, 0]), np.int_(PointCloud_frac[:, 1])] = PointCloud_frac[:, 2] / max_height

    # Intensity Map & DensityMap
    intensityMap = np.zeros((Height, Width))
    densityMap = np.zeros((Height, Width))

    _, indices, counts = np.unique(PointCloud[:, 0:2], axis=0, return_index=True, return_counts=True)
    PointCloud_top = PointCloud[indices]

    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64))

    intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
    densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

    RGB_Map = np.zeros((3, Height - 1, Width - 1))
    RGB_Map[2, :, :] = densityMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # r_map
    RGB_Map[1, :, :] = heightMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # g_map
    RGB_Map[0, :, :] = intensityMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # b_map

    return RGB_Map


# bev image coordinates format
def get_corners(x, y, w, l, yaw):
    bev_corners = np.zeros((4, 2), dtype=np.float32)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    # front left
    bev_corners[0, 0] = x - w / 2 * cos_yaw - l / 2 * sin_yaw
    bev_corners[0, 1] = y - w / 2 * sin_yaw + l / 2 * cos_yaw

    # rear left
    bev_corners[1, 0] = x - w / 2 * cos_yaw + l / 2 * sin_yaw
    bev_corners[1, 1] = y - w / 2 * sin_yaw - l / 2 * cos_yaw

    # rear right
    bev_corners[2, 0] = x + w / 2 * cos_yaw + l / 2 * sin_yaw
    bev_corners[2, 1] = y + w / 2 * sin_yaw - l / 2 * cos_yaw

    # front right
    bev_corners[3, 0] = x + w / 2 * cos_yaw - l / 2 * sin_yaw
    bev_corners[3, 1] = y + w / 2 * sin_yaw + l / 2 * cos_yaw

    return bev_corners

# From Nova, converts ComplexYolo 2D boxes into BoundingBox3D ros2 msg
def get_corners_3d(x, y, z, w, l, h, yaw):
    bev_corners = np.zeros((8, 3), dtype=np.float64)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    # front right
    bev_corners[[0,1], 0] = x + w / 2 * cos_yaw - l / 2 * sin_yaw
    bev_corners[[0,1], 1] = y + w / 2 * sin_yaw + l / 2 * cos_yaw

    # front left
    bev_corners[[2,3], 0] = x - w / 2 * cos_yaw - l / 2 * sin_yaw
    bev_corners[[2,3], 1] = y - w / 2 * sin_yaw + l / 2 * cos_yaw

    # rear right
    bev_corners[[4,5], 0] = x + w / 2 * cos_yaw + l / 2 * sin_yaw
    bev_corners[[4,5], 1] = y + w / 2 * sin_yaw - l / 2 * cos_yaw

    # rear left
    bev_corners[[6,7], 0] = x - w / 2 * cos_yaw + l / 2 * sin_yaw
    bev_corners[[6,7], 1] = y - w / 2 * sin_yaw - l / 2 * cos_yaw

    # top corners
    bev_corners[[1,2,5,6], 2] = z + h

    # bottom corners
    bev_corners[[0,3,4,7], 2] = z

    return bev_corners


def inverse_yolo_target(targets, img_size, bc):
    labels = []
    for t in targets:
        y, x, l, w, im, re, c_conf, *_, c = t # swaps x and y
        z, h = 0.0, 1.5
        if c == 1:
            h = 1.8
        elif c == 2:
            h = 1.4

        y = (y / img_size) * (bc["maxY"] - bc["minY"]) + bc["minY"]
        x = (x / img_size) * (bc["maxX"] - bc["minX"]) + bc["minX"]
        w = (w / img_size) * (bc["maxY"] - bc["minY"])
        l = (l / img_size) * (bc["maxX"] - bc["minX"])
        w -= 0.3
        l -= 0.3
        labels.append([c, c_conf, x, y, z, w, l, h, - np.arctan2(im, re) - 2 * np.pi])

    return np.array(labels)
