uint8 PEDESTRIAN=0
uint8 CYCLIST=1
uint8 CAR=2

# pedestrian, cyclist, car
uint8 label

# unique identifier for the object
uint32 id

# The confidence score of the detected object in the range [0-1]
float32 confidence_score

# A 3D bounding box
BoundingBox3D bounding_box