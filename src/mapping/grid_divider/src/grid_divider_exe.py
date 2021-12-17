#!/usr/bin/python3

import getopt, sys
import numpy as np
from tqdm import tqdm
import math
import os, pathlib

def main(argv):
    if len(sys.argv) < 3:
        print("Usage: {} inputfile.pcd cellSize".format(sys.argv[0]))
        sys.exit(2)
    filename = sys.argv[1]
    cellSize = int(sys.argv[2])
    # print(filename)
    print("Reading from file. This might take a while...")
    points = np.loadtxt(filename, skiprows=11)
    min_x = np.amin(points[:, 0])
    min_y = np.amin(points[:, 1])
    max_x = np.amax(points[:, 0])
    max_y = np.amax(points[:, 1])
    # print(points[:, 0])
    # print(points[:, 1])
    # print("Min: ({},{}), Max: ({},{})".format(min_x, min_y, max_x, max_y))

    # Starting with the min corner (min x, min y)
    cellCorner = [min_x, min_y]
    cell_i = 0
    cell_width = math.ceil((max_x/cellSize))
    for i in tqdm(range(cell_width)): # Cell columns (x)
        points_in_column_mask = (points[:,0]>=cellCorner[0]) & \
                                (points[:,0]<cellCorner[0]+cellSize)
        points_in_column = points[points_in_column_mask]
        cellCorner[1] = min_y
        cell_j = 0
        while(cellCorner[1] < max_y):
            # find all points with x >= cellCorner, x < cellCorner+cellSize
                # y >= cellCorner[1], y < cellCorner+cellSize
            # move points into separate array, save to A_B.pcd
            points_in_cell_mask = (points_in_column[:,1]>=cellCorner[1]) & \
                                (points_in_column[:,1]<cellCorner[1]+cellSize)
            points_in_cell = points_in_column[points_in_cell_mask]
            #print("Writing cell to {}_{}.pcd".format(cell_i, cell_j))
            saveToPcd(points_in_cell, "{}_{}.pcd".format(cell_i, cell_j))
            cell_j += 1
            cellCorner[1] += cellSize
        cell_i += 1
        cellCorner[0] += cellSize
    print(cell_i, cell_j)

def removeGroundPointsWithPDAL(filename):
    # Get path to pipeline.json
    script_path = os.path.dirname(os.path.realpath(__file__))
    pipeline_path = os.join(Path(script_path).parents[0], "pipeline.json")
    # Translate to LAS format tmp.las
    # Apply pipeline
    print(pipeline_path)
    # return res
            
def saveToPcd(point_array, filename):
    header = generateHeader(point_array)
    np.savetxt(filename, point_array, header=header, fmt="%f", comments="")
    return

def generateHeader(point_array):
    # The awful indention is necessary for proper string formatting...
    return """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii""".format(np.shape(point_array)[0],np.shape(point_array)[0])

if __name__ == "__main__":
    main(sys.argv)

# Read PCD file
# Convert points into numpy array
# Find min x, min y, max x, max y
# with open()