std_msgs/Header header
uint32 width
uint32 height
float32 resolution
geometry_msgs/Pose origin

# Row-major, size width*height. -1 = unknown/non-drivable, 0..N-1 = lane index left-to-right.
int16[] cell_lane_id

# Row-major, size width*height. Per-cell confidence, 0-100.
uint8[] cell_confidence

int32 total_lane_count
int32 ego_lane_index
float32 ego_lane_width_m
