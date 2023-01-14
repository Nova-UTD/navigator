#It contains masses for DST occupancy grid calculation and prediction.
#Occ - occupied masses.
#Free - free masses.
#Stored in a row-major order, starting with (0,0). Values in the range [0,1].

Header header # ROS defined header containing timestamp and sequence id

float32[] occ
float32[] free
int32 width
int32 height
