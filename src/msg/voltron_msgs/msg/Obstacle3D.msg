uint8 PERSON=0
uint8 BICYLCLE=1
uint8 CAR=2
uint8 MOTORBIKE=3
uint8 BUS=4
uint8 TRUCK=5
uint8 OTHER=99

# car, pedestrian, etc.
uint8 label

# The numeric ID of the detected object
uint32 id

# The confidence score of the detected object in the range [0-1]
float32 confidence

# linear velocity of the object
geometry_msgs/Vector3 velocity

# A 3D bounding box
BoundingBox3D bounding_box
