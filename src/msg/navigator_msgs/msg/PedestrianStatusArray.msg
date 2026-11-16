# PedestrianStatusArray.msg
# Array of pedestrian statuses published each frame by pedestrian_crossing_node.

# frame_id is the frame the corridor geometry and predicted trajectories are
# expressed in — the frame of /planning/path (base_link in the current stack).
std_msgs/Header header

# One entry per active tracked pedestrian. May contain a single entry with
# pedestrian_id == PedestrianStatus.FAILSAFE_ID when inputs have dropped out.
PedestrianStatus[] statuses
