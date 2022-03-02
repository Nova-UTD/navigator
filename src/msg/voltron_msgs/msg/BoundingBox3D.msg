
# center pose of the box
geometry_msgs/Pose center

#      1 ------- 2
#     /.        /|
#    0 ------- 3 |
#    | .       | |
#    | 5.......| 6
#    |.        |/
#    4 ------- 7
geometry_msgs/Point[8] corners # DEPRECATED 3/2/21 WSH

# Size of the bounding box, in meters
geometry_msgs/Vector3 size 
