#!/usr/bin/python3

import getopt, sys
import numpy as np
from tqdm import tqdm

def main(argv):
    if len(sys.argv) < 2:
        print("Usage: {} inputfile.pcd".format(sys.argv[0]))
        sys.exit(2)
    filename = sys.argv[1]
    print(filename)
    inputfile = open(filename)
    lines = inputfile.readlines()
    main_header = lines[0:10]
    point_array = np.array([])
    for line in tqdm(lines[11:]):
        x, y, z, i = line.split(" ")
        point_array = np.append(point_array, [x, y, z, i])
    print(point_array.size())

if __name__ == "__main__":
    main(sys.argv)

# Read PCD file
# Convert points into numpy array
# Find min x, min y, max x, max y
# with open()