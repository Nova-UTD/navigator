# Stores the amplitude and track ID for the most "relevant" radar detection

# ROS defined header containing timestamp and sequence id
std_msgs/Header header

# to match the corresponding track ID from the radar
int8 track_id

# in decibels
float32 amplitude
