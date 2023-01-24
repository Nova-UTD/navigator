from leaderboard.autoagents.ros2_agent import ROS2Agent
from leaderboard.autoagents.autonomous_agent import Track


def get_entry_point():
    return 'NavigatorAgent'


class NavigatorAgent(ROS2Agent):

    def get_ros_entrypoint(self):
        return {
            "package": "carla_interface",
            "launch_file": "carla.launch.py",
            "parameters": {}
        }

    def setup(self, path_to_conf_file):
        # At a minimum, this method sets the Leaderboard modality. In this case, SENSORS
        self.track = Track.MAP

    def sensors(self):

        # Relative position of ROS base_link to the CARLA ego origin
        # NOTE: CARLA uses the UE4 coordniate system. Not the same as ROS!
        # tl;dr y axis is inverted, creating a left-handed system.
        # See: https://carla.readthedocs.io/en/latest/python_api/#carlarotation
        # NOTE: ROTATIONS ARE IN RADIANS. Ignore the CARLA docs!
        base_link_pos = [0.7, 0.0, 0.28]

        sensors = [
            {'type': 'sensor.camera.rgb', 'id': 'rgb_left',
             'x': base_link_pos[0]+0.0,
             'y': base_link_pos[1]-0.15,
             'z': base_link_pos[2]+1.6,
             'roll': 0.0, 'pitch': 0.0, 'yaw': -0.9,
             'width': 1024, 'height': 512, 'fov': 120},

            {'type': 'sensor.camera.rgb', 'id': 'rgb_right',
             'x': base_link_pos[0]+0.0,
             'y': base_link_pos[1]+0.15,
             'z': base_link_pos[2]+1.6,
             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.9,
             'width': 1024, 'height': 512, 'fov': 120},

            {'type': 'sensor.lidar.ray_cast', 'id': 'lidar',
             'x': base_link_pos[0]+0.0,
             'y': base_link_pos[1]+0.0,
             'z': base_link_pos[2]+1.8,
             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            # {'type': 'sensor.other.radar', 'id': 'RADAR',
            # 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'fov': 30},
            {'type': 'sensor.other.gnss', 'id': 'gnss',
             'x': base_link_pos[0]+0.0,
             'y': base_link_pos[1]+0.0,
             'z': base_link_pos[2]+0.0, },
            {'type': 'sensor.other.imu', 'id': 'imu',
             'x': base_link_pos[0]+0.0,
             'y': base_link_pos[1]+0.0,
             'z': base_link_pos[2]+0.0,
             'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            {'type': 'sensor.opendrive_map',
                'id': 'map', 'reading_frequency': 1},
            {'type': 'sensor.speedometer', 'id': 'speed'},
        ]
        return sensors

    def destroy(self):
        print("Navigator Agent is shutting down. Goodbye.")
        return super().destroy()
