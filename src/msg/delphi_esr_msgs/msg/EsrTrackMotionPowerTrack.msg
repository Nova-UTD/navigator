# ESR TrackMotionPower, track-specific information
uint8  id

# Boolean "Fast movable flag"
# 0 = not movable
# 1 = movable
bool   movable_fast

# Boolean "Slow movable flag"
# 0 = not movable
# 1 = movable
bool   movable_slow

# Boolean Moving Flag
# 0 = stationary
# 1 = moving
bool   moving

# Track amplitude -10 -- 21 dB
# Original CAN data has offset of -10dB. 
# 0 -- 31 corresponds to -10 -- 21 dB
int8   power
