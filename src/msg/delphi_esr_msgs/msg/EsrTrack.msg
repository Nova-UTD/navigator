std_msgs/Header header

# ESR Track
string        canmsg

uint8         id

# Lateral rate in m/s
# + means counter clockwise
# -8 -- 7.75 m/s
float32       lat_rate

# Boolean indicator that number of detections associated with the track have changed
# 0 = no change
# 1 = change
bool          grouping_changed

# Boolean indicator for oncoming object
bool          oncoming

# Measurement status
# 0 = no target
# 1 = new target
# 2 = new updated target
# 3 = updated target
# 4 = coasted target
# 5 = merged target
# 6 = invalid coasted target
# 7 = new coasted target
uint8         status

# Angle to object
# 0 = toward front of vehicle parallel to vehicle centerline
# -51.2 -- 51.1 in degrees (not radian)
float32       angle

# Range in meters 
# 0 -- 204.7 m
float32       range

# Boolean indicator whether the track is a bridge object
bool          bridge_object

# "must equal rolling count of all messages in this burst"
bool          rolling_count

# specialized for volvo, not used
float32       width

# Acceleration in m/s/s 
# + is away from vehicle
# -25.6 -- 25.55 m/s/s
float32       range_accel

# Indicates which mode updated the fused track
# 0 = No MR / LR update
# 1 = MR update only
# 2 = LR update only
# 3 = Both MR / LR update
uint8         med_range_mode

# Velocity in m/s 
# + is away from vehicle
# -81.92 -- 81.91 m/s
float32       range_rate
