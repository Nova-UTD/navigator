# -------------- /pointCloud.msg ----------------
# Custom ROS message for point clouds

Header header # ROS defined header containing timestamp and sequence id

# information about points

int64 num_points # Number of points defined in points array
lidar_msgs/point[] points # Array of points

# information about lasers

float64[] range_m
float64[] theta
float64[] phi
int64[] firing
int64[] sorted

# information about lidars

int64 num_lidars
int64[] lidar_id
