from leaderboard.autoagents.ros2_agent import ROS2Agent
from leaderboard.autoagents.autonomous_agent import Track

def get_entry_point():
    return 'NavigatorAgent'

class NavigatorAgent(ROS2Agent):
    def get_ros_entry_point(self):
        return {
            "package": "my_ros_agent",
            "launch_file": "my_ros_agent.launch",
            "parameters": {}
        }

    def setup(self, path_to_conf_file):
        self.track = Track.MAP

    def destroy(self): # Not sure if this is right. WSH.
        """
        Destroy (clean-up) the agent
        :return:
        """

        super.destroy() 