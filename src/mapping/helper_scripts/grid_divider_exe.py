#!/usr/bin/python3

"""
This is a quick-and-dirty script that accepts a single unfiltered PCD file,
divides it into uniform cells, and then removes ground points from each cell.

With the FORM_COMBINED_MAP flag, the script will recombine the filtered cells
to produce a single filtered map. This is only needed if you wish to see the
full result instead of seeing individual cells. You can convert the combined
result from .las to .pcd using the "pdal translate" command. PDAL messes up
pcd headers during conversion, so compare the result header to a working pcd
file's to fix. You just need to change the sizes from 8 to 4 and the X Y Z to
x y z.

Remember that pcd files can be visualized using the pcl_viewer tool.

Script usage: grid_divider_exe.py [input.pcd] [cell size]
"""

import getopt, sys
import numpy as np
from tqdm import tqdm
import math
import os, pathlib
import json
import subprocess

MIN_FILTER_SIZE = 500 # Number of points that cell must contain for it to be filtered
FORM_COMBINED_MAP = True

PIPELINE_STR = """
{
    "pipeline":[
        {
            "type":"readers.las",
            "filename":"tmp.las"
        },
        {
            "type":"filters.assign",
            "assignment":"Classification[:]=0"
        },
        {
            "type":"filters.elm"
        },
        {
            "type":"filters.outlier"
        },
        {
            "type":"filters.smrf",
            "ignore":"Classification[7:7]",
            "slope":0.2,
            "window":16,
            "threshold":0.55,
            "scalar":1.2
        },
        {
            "type":"filters.range",
            "limits":"Classification![2:2]"
        },
        {
            "type":"filters.range",
            "limits":"Classification![7:7]"
        },
        {
            "type":"writers.pcd",
            "filename":"res.pcd",
            "order": "X,Y,Z",
            "keep_unspecified": false
        }
    ]
}
"""
pipeline_json = json.loads(PIPELINE_STR)

def main(argv):
    if len(sys.argv) < 3:
        print("Usage: {} inputfile.pcd cellSize".format(sys.argv[0]))
        sys.exit(2)
    filename = sys.argv[1]
    cellSize = int(sys.argv[2])

    print("Reading from file. This might take a while...")
    points = np.loadtxt(filename, skiprows=11)
    min_x = np.amin(points[:, 0])
    min_y = np.amin(points[:, 1])
    max_x = np.amax(points[:, 0])
    max_y = np.amax(points[:, 1])
    print("Loaded {}x{} PCD with min ({},{})".format(max_x-min_x, max_y-min_y, min_x, min_y))

    # Starting with the min corner (min x, min y)
    cellCorner = [min_x, min_y]
    cell_i = 0
    rejected_count = 0
    cell_width = math.ceil(((max_x-min_x)/cellSize))
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
            saveToPcd(points_in_cell[:,0:3], "{}_{}.pcd".format(cell_i, cell_j))
            if points_in_cell.size > MIN_FILTER_SIZE:
                # print(points_in_cell.size)
                removeGroundPointsWithPDAL("{}_{}.pcd".format(cell_i, cell_j))

                fixfile = open("{}_{}.pcd".format(cell_i, cell_j),'r')
                lines = fixfile.readlines()
                fixfile.close()
                fixfile = open("{}_{}.pcd".format(cell_i, cell_j),'w')
                header = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {}
DATA ascii""".format(len(lines)-10,len(lines)-10)
                fixfile.write(header+"\n")
                fixfile.writelines(lines[10:])
                fixfile.close()
            else:
                rejected_count += 1
            
            cell_j += 1
            cellCorner[1] += cellSize
        cell_i += 1
        cellCorner[0] += cellSize
    print("Generated {}x{} grid, with {}/{} cells unfiltered, from ({},{}) to ({},{})".format(cell_i, cell_j, rejected_count, cell_i*cell_j, min_x, min_y, max_x, max_y))
    
    if FORM_COMBINED_MAP:
        formCombinedMap()


def formCombinedMap():
    # Get all pcd files in cwd containing just numbers and an underscore
    files = [f for f in os.listdir('.') if (os.path.isfile(f) and f[-4:]=='.pcd' and f[0].isnumeric())]
    # print(files)

    # Convert to LAS format for PDAL
    lasfiles = []
    for file in files:
        lasfile = file.split('.')[0]+'.las'
        bashCommand = "pdal translate {} {}".format(file, lasfile)
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        lasfiles.append(lasfile)

    # print(lasfiles)

    lasfiles.append("combined.las")
    file_pipeline = json.dumps(lasfiles)
    # print(file_pipeline)
    with open("combined_pipeline.json", 'w') as file:
        file.write(file_pipeline)

    # Apply ground smoothing pipeline, which overwrites file with filtered version
    bashCommand = "pdal pipeline combined_pipeline.json"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

def removeGroundPointsWithPDAL(filename):
    # Convert to LAS format for PDAL
    bashCommand = "pdal translate {} tmp.las".format(filename)
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()

    # Write to pipeline.json
    pipeline_json["pipeline"][7]["filename"] = filename
    with open("pipeline.json", 'w') as file:
        json.dump(pipeline_json, file)

    # print(pipeline_json)

    # Apply ground smoothing pipeline, which overwrites file with filtered version
    bashCommand = "pdal pipeline pipeline.json"
    process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    
            
def saveToPcd(point_array, filename):
    header = generateHeader(point_array)
    np.savetxt(filename, point_array, header=header, fmt="%f", comments="")
    return

def generateHeader(point_array):
    # The awful indention is necessary for proper string formatting...
    return """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
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