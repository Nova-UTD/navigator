import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from navigator_msgs.msg import VehicleControl, PedalPosition, SteeringPosition


class KeyboardController(Node):

    def __init__(self):
        #Initialize the VehicleControl Variables
        self.throttle = 0.0
        self.steer = 0.0
        self.brake = 0.0
        self.reverse = False
        self.pressed_keys = set()
        # Initialize Pygame
        pygame.init()

        #Set up controller screen
        screen = pygame.display.set_mode((400, 300))

        #Set up node
        super().__init__("keyboard_controller")
        self.command_publisher = self.create_publisher(VehicleControl, "/vehicle/control",1)
        self.input_timer = self.create_timer(0.03  , self.pygame_loop)
        self.control_timer = self.create_timer(0.03, self.control_callback)
        
    #publishes vehicle information to the vehicle control topic
    def control_callback(self):
        control_msg = VehicleControl()
        control_msg.throttle = self.throttle
        control_msg.brake = self.brake 
        control_msg.steer = self.steer
        control_msg.reverse = self.reverse
        self.command_publisher.publish(control_msg)
    
    #handles Pygame controls
    def handle_controls(self):
        if pygame.K_w in self.pressed_keys:
            self.throttle += 0.05
        elif pygame.K_s in self.pressed_keys:
            self.throttle -= 0.05
            self.brake += 0.05
        elif pygame.K_a in self.pressed_keys:
            self.steer -= 0.03
        elif pygame.K_d in self.pressed_keys:
            self.steer += 0.03
        elif pygame.K_x in self.pressed_keys:
            self.reverse = True
            self.throttle = 0.3
        elif pygame.K_q in self.pressed_keys: 
            pygame.quit()
        else: 
            self.throttle = 0.0
            self.steer = 0.0
            self.brake = 0.0
            self.reverse = False
    
    #main pygame loop
    def pygame_loop(self):
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT:
                break
            if event.type == pygame.KEYDOWN: 
                self.pressed_keys.add(event.key)
            if event.type == pygame.KEYUP:
                self.pressed_keys.discard(event.key)
                
        self.handle_controls()
    

def main(args=None):
    rclpy.init(args = args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    
    keyboard_controller.destroy_node()
    # Shutdown ROS and Pygame
    rclpy.shutdown()
    pygame.quit()

if __name__ == "__main__":
    main()
