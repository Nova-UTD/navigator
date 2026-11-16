# PedestrianStatus.msg
# Status signal for a single tracked pedestrian.
#
# Published by pedestrian_crossing_node. Consumed by the behavior layer
# (see pedestrian_crossing/pedestrian_handler.py).

# Unique tracker ID (matches Object3D.id from /tracked/objects3d).
# The reserved value 4294967295 (uint32 max) marks a synthetic fail-safe
# entry emitted when a required input drops out — it is not a real track.
uint32 pedestrian_id

# Reserved ID used for fail-safe entries.
uint32 FAILSAFE_ID=4294967295

# Derived signal from TTC thresholds:
#   0 = CLEAR  (TTC > 5s or no path intersection predicted)
#   1 = SLOW   (TTC 2-5s, speed proportional to TTC)
#   2 = STOP   (TTC < 2s, pedestrian in or imminently entering path)
uint8 status

uint8 CLEAR=0
uint8 SLOW=1
uint8 STOP=2

# Crossing-intent state from the per-pedestrian state machine. Independent
# of TTC: describes what the pedestrian appears to be doing, where `status`
# describes what the vehicle should do about it.
#   0 = IRRELEVANT      (off to the side, not engaging with the road)
#   1 = UNCERTAIN       (in the approach zone, intent unclear)
#   2 = INTENT_TO_CROSS (facing the road and moving toward it)
#   3 = CROSSING        (inside the path corridor)
#   4 = CLEARED         (was crossing, has since exited the corridor)
uint8 intent

uint8 IRRELEVANT=0
uint8 UNCERTAIN=1
uint8 INTENT_TO_CROSS=2
uint8 CROSSING=3
uint8 CLEARED=4

# Time-to-collision in seconds: time until the predicted trajectory
# intersects the vehicle's path corridor.
# -1.0 means no intersection predicted within the prediction horizon.
float32 ttc

# True when the camera detection layer (/pedestrians) contributed to this
# entry. False means the entry is derived from the 3D tracker alone, which
# is the normal case when pedestrian_intent_to_enter_road is not running.
bool camera_corroborated

# Predicted future positions of this pedestrian (constant velocity model).
# Points at PREDICTION_DT intervals up to the prediction horizon.
# Same frame as the enclosing PedestrianStatusArray header.
geometry_msgs/Point[] predicted_trajectory
