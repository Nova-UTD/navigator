# Center pose of the box
geometry_msgs/Pose center

#      1 ------- 2
#     /.        /|
#    0 ------- 3 |
#    | .       | |
#    | 5.......| 6
#    |.        |/
#    4 ------- 7
geometry_msgs/Point[8] corners # DEPRECATED 3/2/21 WSH

# A vector from the box center to a corner.
# The box's dimensions are equal to twice the value of each component.
# Ex: size.x is half the length,
#     size.y is half the width,
#     size.z is half the height.
#
# This vector is independent of the coordinate frame.
# The pose (above) is dependent on the coordinate frame.
geometry_msgs/Vector3 size 
