class_list = ["Car", "Pedestrian", "Cyclist"]

CLASS_NAME_TO_ID = {
    'Car': 0,
    'Pedestrian': 1,
    'Cyclist': 2,
    'Van': 0,
    'Person_sitting': 1,
}

# Front side (of vehicle) Point Cloud boundary for BEV
boundary = {
    "minX": 0,
    "maxX": 50,
    "minY": -25,
    "maxY": 25,
    "minZ": -1.23, #-0.73, #-2.73, # -(height to ground + 1)
    "maxZ": 2.77 #3.27 #1.27
}

# Back back (of vehicle) Point Cloud boundary for BEV
boundary_back = {
    "minX": -50,
    "maxX": 0,
    "minY": -25,
    "maxY": 25,
    "minZ": -2.73,
    "maxZ": 1.27
}

BEV_WIDTH = 608  # across y axis -25m ~ 25m
BEV_HEIGHT = 608  # across x axis 0m ~ 50m

DISCRETIZATION = (boundary["maxX"] - boundary["minX"]) / BEV_HEIGHT

colors = [[0, 255, 255], [0, 0, 255], [255, 0, 0]]
