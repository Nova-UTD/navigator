uint8 CAR=0
uint8 BIKE=1
uint8 PEDESTRIAN=2
uint8 OTHER=99

# car, pedestrian, etc.
uint8 label

# The numeric ID of the detected object
uint32 id

# The confidence score of the detected object in the range [0-1]
float32 confidence

# A 3D bounding box
BoundingBox2D bounding_box
