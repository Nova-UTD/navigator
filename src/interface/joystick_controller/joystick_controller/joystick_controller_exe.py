import rclpy
import os
from rclpy.node import Node
import threading

from sensor_msgs.msg import Joy
from playsound import playsound

class JoystickController(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_received,
            10)
        self.subscription  # prevent unused variable warning
        self.alert_timer = self.create_timer(0.4, self.playAlert)
        self.deadman_pressed = False

    def joy_received(self, msg):
        self.deadman_pressed = (msg.buttons[2] == 1)
        

    def playAlert(self):
        if not self.deadman_pressed:
            print('\a')
            # script_path = os.path.dirname(os.path.realpath(__file__))
            # threading.Thread(target=playsound, args=(script_path+'/released_alert.mp3', False), daemon=True).start()

def main(args=None):
    rclpy.init(args=args)

    joystick_controller = JoystickController()

    rclpy.spin(joystick_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()